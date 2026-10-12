#ifndef BNO085_H
#define BNO085_H

#include "mbed.h"
#include <cmath>
#include <cstdint>
#include <chrono>

/**
 * @brief クオータニオン (四元数) 構造体
 */
struct Quaternion {
    float w;
    float x;
    float y;
    float z;

    constexpr Quaternion() noexcept : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    constexpr Quaternion(float _w, float _x, float _y, float _z) noexcept : w(_w), x(_x), y(_y), z(_z) {}

    // ノルム (大きさ)
    float norm() const noexcept {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // 正規化
    Quaternion normalized() const noexcept {
        float n = norm();
        if (n < 1e-6f) return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        return Quaternion(w / n, x / n, y / n, z / n);
    }

    // 共役クオータニオン (逆回転)
    Quaternion conjugate() const noexcept {
        return Quaternion(w, -x, -y, -z);
    }

    // クオータニオン積 (回転合成)
    Quaternion operator*(const Quaternion& q) const noexcept {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }
};

/**
 * @brief オイラー角 (ロール・ピッチ・ヨー) 構造体
 */
struct EulerAngles {
    float yaw;       // [rad] -pi ~ +pi
    float pitch;     // [rad] -pi/2 ~ +pi/2
    float roll;      // [rad] -pi ~ +pi

    float yaw_deg;   // [deg] -180 ~ +180
    float pitch_deg; // [deg] -90 ~ +90
    float roll_deg;  // [deg] -180 ~ +180
};

/**
 * @brief BNO085 IMU / センサハブ制御クラス (Mbed OS 6 プロフェッショナル仕様)
 */
class BNO085 : private mbed::NonCopyable<BNO085> {
public:
    enum ReportType {
        REPORT_ROTATION_VECTOR = 0x05,      // 9軸フュージョン (地磁気あり・絶対方位)
        REPORT_GAME_ROTATION_VECTOR = 0x08  // 6軸フュージョン (加速度+ジャイロ・磁気外乱耐性高)
    };

    /**
     * @brief コンストラクタ
     * @param sda_pin I2C SDAピン (Nucleo-F446RE: D14 / PB_9)
     * @param scl_pin I2C SCLピン (Nucleo-F446RE: D15 / PB_8)
     * @param int_pin 割り込み/データレディピン INT (例: D2 / PA_10)
     * @param rst_pin リセットピン RST (例: D3 / PB_3)
     * @param i2c_addr 7-bit I2Cアドレス (デフォルト: 0x4A, SA0=HIGH時は 0x4B)
     * @param i2c_freq_hz I2Cクロック周波数 (推奨 400kHz: 400000)
     */
    BNO085(PinName sda_pin, PinName scl_pin, PinName int_pin, PinName rst_pin,
           uint8_t i2c_addr = 0x4A, uint32_t i2c_freq_hz = 400000);

    /**
     * @brief センサ初期化
     * @param report_type レポート種類 (デフォルト: REPORT_GAME_ROTATION_VECTOR)
     * @param report_interval_ms レポート周期 [ms] (デフォルト: 10ms = 100Hz)
     * @return true: 成功, false: 失敗
     */
    bool begin(ReportType report_type = REPORT_GAME_ROTATION_VECTOR, uint32_t report_interval_ms = 10);

    /**
     * @brief ハードウェアリセット実行
     */
    void hardwareReset();

    /**
     * @brief 指定したレポートを有効化
     * @param report_id レポートID (0x05 or 0x08)
     * @param interval_us レポート更新間隔 [マイクロ秒]
     */
    bool enableReport(uint8_t report_id, uint32_t interval_us);

    /**
     * @brief センサからの受信データを処理 (ノンブロッキング・ポーリング)
     * @return true: 新しい姿勢データを受信して更新した, false: 新規データなし
     */
    bool update();

    /**
     * @brief データ準備完了までRTOSスリープ待機し、到着次第即座に更新 (割り込み駆動型)
     * @param timeout_ms 最大待機時間 [ms] (デフォルト: 50ms)
     * @return true: 更新成功, false: タイムアウト
     */
    bool waitForUpdate(uint32_t timeout_ms = 50);

    // ==========================================
    // 姿勢データアクセサ
    // ==========================================
    Quaternion getQuaternion() const noexcept { return _quat; }
    EulerAngles getEulerAngles() const noexcept { return _euler; }

    float getYawDeg() const noexcept { return _euler.yaw_deg; }
    float getYawRad() const noexcept { return _euler.yaw; }
    float getPitchDeg() const noexcept { return _euler.pitch_deg; }
    float getRollDeg() const noexcept { return _euler.roll_deg; }

    /**
     * @brief 連続Yaw角 [deg] を取得 (±180°の境界ジャンプをアンラップした累積角度)
     */
    float getContinuousYawDeg() const noexcept { return _continuous_yaw_deg; }
    float getContinuousYawRad() const noexcept { return _continuous_yaw_rad; }

    /**
     * @brief 現在のYaw角を0度としてリセット (Tare)
     */
    void resetYawZero();

    /**
     * @brief ゼロ点オフセット適用後の相対Yaw角 [deg] (-180 ~ +180)
     */
    float getRelativeYawDeg() const noexcept;

    /**
     * @brief 推定Yaw角速度 [deg/s] を取得 (微分 + 1次LPF)
     */
    float getYawVelocityDeg() const noexcept { return _yaw_velocity_deg; }

    /**
     * @brief 実測サンプリング周波数 [Hz] を取得
     */
    float getSampleRateHz() const noexcept { return _actual_rate_hz; }

    /**
     * @brief センサ精度のステータスを取得 (0: Unreliable, 1: Low, 2: Medium, 3: High)
     */
    uint8_t getAccuracy() const noexcept { return _accuracy; }

    /**
     * @brief センサからのデータ準備完了 (INTピンがLOW) かどうか
     */
    bool hasData() noexcept { return _int_pin.read() == 0; }

private:
    mbed::I2C _i2c;
    mbed::InterruptIn _int_pin;
    mbed::DigitalOut _rst_pin;
    uint8_t _i2c_addr_8bit;

    // RTOS 割り込み通知用 EventFlags
    rtos::EventFlags _event_flags;
    static constexpr uint32_t FLAG_DATA_READY = (1UL << 0);

    // SHTP シーケンス番号管理
    uint8_t _seq_num[6];

    // 受信バッファ
    static constexpr size_t MAX_PACKET_SIZE = 128;
    uint8_t _rx_buffer[MAX_PACKET_SIZE];

    // 姿勢データ
    Quaternion _quat;
    EulerAngles _euler;
    uint8_t _accuracy;

    // 連続Yaw & 角速度推定用
    float _prev_yaw_rad;
    float _continuous_yaw_rad;
    float _continuous_yaw_deg;
    float _yaw_velocity_deg;
    float _yaw_offset_deg;
    bool _is_first_yaw;

    // タイミング計測用
    Kernel::Clock::time_point _last_update_time;
    float _actual_rate_hz;
    uint32_t _sample_count;
    Kernel::Clock::time_point _rate_calc_time;

    // 内部通信 & 数学ヘルパー
    void onInterruptRise();
    void onInterruptFall();
    bool receivePacket(uint8_t &channel, uint16_t &payload_len, uint32_t timeout_ms = 50);
    bool sendPacket(uint8_t channel, const uint8_t *payload, uint16_t payload_len);
    void computeEulerAngles();
    void updateContinuousYawAndVelocity(float dt_sec);
    void drainIncomingPackets(uint32_t drain_duration_ms = 150);
};

#endif // BNO085_H
