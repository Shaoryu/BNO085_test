#include "mbed.h"
#include "BNO085.h"
#include <cstdio>

using namespace std::chrono_literals;

// ==========================================
// ピン配置設定 (Nucleo-F446RE)
// ==========================================
// I2C1 (CN5 / Arduinoピン配置互換):
//   SDA: D14 (PB_9)
//   SCL: D15 (PB_8)
// センサ制御ピン:
//   INT: D2  (PA_10) - データレディ割り込み
//   RST: D3  (PB_3)  - ハードウェアリセット
// ユーザーインターフェース:
//   LED: LED1 (PA_5)
//   BTN: BUTTON1 (PC_13, 青色ボタン)
// ==========================================

static constexpr PinName PIN_SDA = D14;
static constexpr PinName PIN_SCL = D15;
static constexpr PinName PIN_INT = D2;
static constexpr PinName PIN_RST = D3;

// BNO085 インスタンス (I2C 400kHz Fast-mode)
static BNO085 imu(PIN_SDA, PIN_SCL, PIN_INT, PIN_RST, 0x4A, 400000);

// 姿勢センサ専用のRTOSワーカースレッド
static Thread sensor_thread(osPriorityHigh, 2048, nullptr, "bno085_worker");

// スレッド間データ共有用のミューテックスと構造体
struct SensorSnapshot {
    Quaternion q;
    EulerAngles euler;
    float cont_yaw;
    float rel_yaw;
    float yaw_rate;
    float rate_hz;
    uint8_t accuracy;
};
static Mutex data_mutex;
static SensorSnapshot latest_data;

// センサ収集スレッド関数 (割り込み駆動型・超低レイテンシ)
void sensor_worker_task()
{
    while (true) {
        // INTピンの立ち下がり割り込みを待機 (最大50msタイムアウト)
        // センサからデータが届くまでスレッドは自動的にスリープし、CPU負荷0%
        if (imu.waitForUpdate(50)) {
            data_mutex.lock();
            latest_data.q        = imu.getQuaternion();
            latest_data.euler    = imu.getEulerAngles();
            latest_data.cont_yaw = imu.getContinuousYawDeg();
            latest_data.rel_yaw  = imu.getRelativeYawDeg();
            latest_data.yaw_rate = imu.getYawVelocityDeg();
            latest_data.rate_hz  = imu.getSampleRateHz();
            latest_data.accuracy = imu.getAccuracy();
            data_mutex.unlock();
        }
    }
}

int main()
{
    DigitalOut led(LED1);
    InterruptIn user_btn(BUTTON1);

    printf("\r\n==================================================\r\n");
    printf(" Nucleo-F446RE + BNO085 (Mbed OS 6 Professional)\r\n");
    printf(" Interrupt-Driven High-Rate IMU Tracker\r\n");
    printf("==================================================\r\n");

    // 青色ボタン押下でYawを0度に原点復帰 (Tare)
    user_btn.fall([]() {
        imu.resetYawZero();
        printf("\r\n>>> [TARE] Yaw angle reset to 0.00 deg <<<\r\n\r\n");
    });

    printf("Booting BNO085 in Game Rotation Vector mode (100Hz)...\r\n");

    // 100Hz (10ms周期) でセンサ初期化
    if (!imu.begin(BNO085::REPORT_GAME_ROTATION_VECTOR, 10)) {
        printf("ERROR: BNO085 initialization failed! Check wiring / pull-up resistors.\r\n");
        while (true) {
            led = !led;
            ThisThread::sleep_for(100ms);
        }
    }

    printf("Initialization successful!\r\n");
    printf("- RTOS Sensor Thread Priority: High (Interrupt-driven)\r\n");
    printf("- Singularity fallback logic: Active (Gimbal-lock free)\r\n");
    printf("- Press Nucleo Blue Button (B1) to reset Yaw (Tare)\r\n\r\n");

    // センサ専用スレッドを開始
    sensor_thread.start(sensor_worker_task);

    // メインスレッド：表示・通信タスク (20Hz / 50ms周期)
    while (true) {
        SensorSnapshot snap;
        data_mutex.lock();
        snap = latest_data;
        data_mutex.unlock();

        // 人間が読みやすいテキストフォーマット出力
        printf("[IMU] Yaw:%7.2f | ContYaw:%8.2f | Rate:%+7.1f d/s | Pitch:%6.2f | Roll:%6.2f | %4.1fHz | Acc:%u\r\n",
               snap.euler.yaw_deg,
               snap.cont_yaw,
               snap.yaw_rate,
               snap.euler.pitch_deg,
               snap.euler.roll_deg,
               snap.rate_hz,
               snap.accuracy);

        // ※シリアルプロッター（Arduino IDEプロッターやTeleplot等）で波形表示したい場合は
        // 上のprintfをコメントアウトし、下の行のコメントを解除してください：
        // printf(">yaw:%.2f,cont_yaw:%.2f,yaw_rate:%.2f,pitch:%.2f,roll:%.2f\n",
        //        snap.euler.yaw_deg, snap.cont_yaw, snap.yaw_rate, snap.euler.pitch_deg, snap.euler.roll_deg);

        led = !led;
        ThisThread::sleep_for(50ms);
    }
}
