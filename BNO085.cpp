#include "BNO085.h"

using namespace std::chrono_literals;

// SHTP チャンネル定義
static constexpr uint8_t CHANNEL_COMMAND    = 0;
static constexpr uint8_t CHANNEL_EXECUTABLE = 1;
static constexpr uint8_t CHANNEL_CONTROL    = 2;
static constexpr uint8_t CHANNEL_REPORTS    = 3;
static constexpr uint8_t CHANNEL_WAKE       = 4;
static constexpr uint8_t CHANNEL_GYRO       = 5;

// SHTP レポートコマンド
static constexpr uint8_t SHTP_REPORT_SET_FEATURE_COMMAND = 0xFD;
static constexpr uint8_t SHTP_REPORT_BASE_TIMESTAMP      = 0xFB;

#ifndef M_PI
static constexpr float M_PI = 3.14159265358979323846f;
#endif

BNO085::BNO085(PinName sda_pin, PinName scl_pin, PinName int_pin, PinName rst_pin,
               uint8_t i2c_addr, uint32_t i2c_freq_hz)
    : _i2c(sda_pin, scl_pin),
      _int_pin(int_pin),
      _rst_pin(rst_pin, 1),
      _i2c_addr_8bit(static_cast<uint8_t>(i2c_addr << 1)),
      _accuracy(0),
      _prev_yaw_rad(0.0f),
      _continuous_yaw_rad(0.0f),
      _continuous_yaw_deg(0.0f),
      _yaw_velocity_deg(0.0f),
      _yaw_offset_deg(0.0f),
      _is_first_yaw(true),
      _actual_rate_hz(0.0f),
      _sample_count(0)
{
    _i2c.frequency(i2c_freq_hz);

    // INTピンの割り込み設定 (立ち下がりでデータレディ)
    _int_pin.mode(PinMode::PullUp);
    _int_pin.fall(mbed::callback(this, &BNO085::onInterruptFall));

    for (size_t i = 0; i < sizeof(_seq_num); ++i) {
        _seq_num[i] = 0;
    }

    _last_update_time = Kernel::Clock::now();
    _rate_calc_time = _last_update_time;
}

void BNO085::onInterruptFall()
{
    _event_flags.set(FLAG_DATA_READY);
}

void BNO085::hardwareReset()
{
    _rst_pin = 0;
    ThisThread::sleep_for(10ms);
    _rst_pin = 1;
    ThisThread::sleep_for(50ms);
}

void BNO085::drainIncomingPackets(uint32_t drain_duration_ms)
{
    auto start_time = Kernel::Clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(Kernel::Clock::now() - start_time).count() < drain_duration_ms) {
        if (_int_pin.read() == 0) {
            uint8_t ch = 0;
            uint16_t len = 0;
            receivePacket(ch, len, 20);
        } else {
            ThisThread::sleep_for(5ms);
        }
    }
}

bool BNO085::begin(ReportType report_type, uint32_t report_interval_ms)
{
    // 1. ハードウェアリセット
    hardwareReset();

    // 2. センサ起動直後のアナウンスパケット等を読み捨ててフラッシュ
    drainIncomingPackets(200);

    // 3. 要求されたレポートを有効化
    uint32_t interval_us = report_interval_ms * 1000;
    bool success = enableReport(static_cast<uint8_t>(report_type), interval_us);

    ThisThread::sleep_for(50ms);
    _last_update_time = Kernel::Clock::now();
    _rate_calc_time = _last_update_time;

    return success;
}

bool BNO085::enableReport(uint8_t report_id, uint32_t interval_us)
{
    // Set Feature Command (17バイトのペイロード)
    uint8_t payload[17];
    memset(payload, 0, sizeof(payload));

    payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    payload[1] = report_id;
    payload[2] = 0; // Feature flags

    // Change sensitivity
    payload[3] = 0;
    payload[4] = 0;

    // Report Interval [マイクロ秒] (Little-endian 32bit)
    payload[5] = static_cast<uint8_t>(interval_us & 0xFF);
    payload[6] = static_cast<uint8_t>((interval_us >> 8) & 0xFF);
    payload[7] = static_cast<uint8_t>((interval_us >> 16) & 0xFF);
    payload[8] = static_cast<uint8_t>((interval_us >> 24) & 0xFF);

    // Batch interval = 0 (即時送信)
    payload[9]  = 0;
    payload[10] = 0;
    payload[11] = 0;
    payload[12] = 0;

    // Sensor-specific configuration
    payload[13] = 0;
    payload[14] = 0;
    payload[15] = 0;
    payload[16] = 0;

    return sendPacket(CHANNEL_CONTROL, payload, sizeof(payload));
}

bool BNO085::sendPacket(uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
    auto start_time = Kernel::Clock::now();
    while (_int_pin.read() != 0) {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(Kernel::Clock::now() - start_time).count() > 50) {
            break;
        }
        ThisThread::sleep_for(1ms);
    }

    uint16_t packet_len = payload_len + 4;
    uint8_t tx_buf[MAX_PACKET_SIZE];
    if (packet_len > sizeof(tx_buf)) {
        return false;
    }

    // 4バイトSHTPヘッダ
    tx_buf[0] = static_cast<uint8_t>(packet_len & 0xFF);
    tx_buf[1] = static_cast<uint8_t>((packet_len >> 8) & 0xFF);
    tx_buf[2] = channel;
    tx_buf[3] = _seq_num[channel]++;

    memcpy(&tx_buf[4], payload, payload_len);

    int err = _i2c.write(_i2c_addr_8bit, reinterpret_cast<const char*>(tx_buf), packet_len, false);
    return (err == 0);
}

bool BNO085::receivePacket(uint8_t &channel, uint16_t &payload_len, uint32_t timeout_ms)
{
    auto start_time = Kernel::Clock::now();
    while (_int_pin.read() != 0) {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(Kernel::Clock::now() - start_time).count() > timeout_ms) {
            return false;
        }
        ThisThread::sleep_for(1ms);
    }

    uint8_t header[4];
    int err = _i2c.read(_i2c_addr_8bit, reinterpret_cast<char*>(header), 4, false);
    if (err != 0) {
        return false;
    }

    uint16_t packet_len = (header[0] | (static_cast<uint16_t>(header[1]) << 8)) & ~0x8000;
    if (packet_len <= 4 || packet_len > MAX_PACKET_SIZE) {
        return false;
    }

    channel = header[2];
    payload_len = packet_len - 4;

    err = _i2c.read(_i2c_addr_8bit, reinterpret_cast<char*>(_rx_buffer), payload_len, false);
    return (err == 0);
}

bool BNO085::update()
{
    if (_int_pin.read() != 0) {
        return false;
    }

    uint8_t channel = 0;
    uint16_t payload_len = 0;
    if (!receivePacket(channel, payload_len, 5)) {
        return false;
    }

    if (channel != CHANNEL_REPORTS || payload_len < 5) {
        return false;
    }

    size_t offset = 0;
    if (_rx_buffer[0] == SHTP_REPORT_BASE_TIMESTAMP) {
        if (payload_len < 10) {
            return false;
        }
        offset = 5;
    }

    uint8_t report_id = _rx_buffer[offset];

    if (report_id == REPORT_ROTATION_VECTOR || report_id == REPORT_GAME_ROTATION_VECTOR) {
        if (offset + 12 > payload_len) {
            return false;
        }

        _accuracy = _rx_buffer[offset + 2] & 0x03;

        int16_t raw_i    = static_cast<int16_t>(_rx_buffer[offset + 4]  | (_rx_buffer[offset + 5] << 8));
        int16_t raw_j    = static_cast<int16_t>(_rx_buffer[offset + 6]  | (_rx_buffer[offset + 7] << 8));
        int16_t raw_k    = static_cast<int16_t>(_rx_buffer[offset + 8]  | (_rx_buffer[offset + 9] << 8));
        int16_t raw_real = static_cast<int16_t>(_rx_buffer[offset + 10] | (_rx_buffer[offset + 11] << 8));

        constexpr float q_scale = 1.0f / 16384.0f;
        _quat.x = static_cast<float>(raw_i) * q_scale;
        _quat.y = static_cast<float>(raw_j) * q_scale;
        _quat.z = static_cast<float>(raw_k) * q_scale;
        _quat.w = static_cast<float>(raw_real) * q_scale;

        // クオータニオンからオイラー角を計算 (特異点フォールバック式付き)
        computeEulerAngles();

        // 時間差分 (dt) の計測
        auto now = Kernel::Clock::now();
        float dt_sec = std::chrono::duration_cast<std::chrono::microseconds>(now - _last_update_time).count() * 1e-6f;
        _last_update_time = now;

        // 連続Yaw角 & 角速度の更新
        updateContinuousYawAndVelocity(dt_sec);

        // サンプリング周波数計測
        _sample_count++;
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - _rate_calc_time).count();
        if (elapsed_ms >= 1000) {
            _actual_rate_hz = static_cast<float>(_sample_count * 1000) / static_cast<float>(elapsed_ms);
            _sample_count = 0;
            _rate_calc_time = now;
        }

        return true;
    }

    return false;
}

bool BNO085::waitForUpdate(uint32_t timeout_ms)
{
    if (_int_pin.read() != 0) {
        uint32_t flags = _event_flags.wait_any(FLAG_DATA_READY, timeout_ms, true);
        if (flags & osFlagsError) {
            return false;
        }
    }
    return update();
}

void BNO085::computeEulerAngles()
{
    // クオータニオンの正規化
    float norm = _quat.norm();
    if (norm < 1e-6f) {
        return;
    }
    float w = _quat.w / norm;
    float x = _quat.x / norm;
    float y = _quat.y / norm;
    float z = _quat.z / norm;

    // Pitch (Y軸周り) の sin 値
    float sinp = 2.0f * (w * y - z * x);

    // ジンバルロック特異点ガード (|sinp| >= 0.9995f, ピッチ約 ±88.2度以上)
    if (std::abs(sinp) >= 0.9995f) {
        if (sinp > 0.0f) {
            // Pitch = +90度 (天頂特異点)
            _euler.pitch = M_PI / 2.0f;
            _euler.roll  = 0.0f;
            _euler.yaw   = 2.0f * std::atan2(z, w);
        } else {
            // Pitch = -90度 (天底特異点)
            _euler.pitch = -M_PI / 2.0f;
            _euler.roll  = 0.0f;
            _euler.yaw   = -2.0f * std::atan2(z, w);
        }
    } else {
        // 通常域：Tait-Bryan Z-Y-X
        _euler.pitch = std::asin(sinp);

        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        _euler.roll = std::atan2(sinr_cosp, cosr_cosp);

        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        _euler.yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    constexpr float rad_to_deg = 180.0f / M_PI;
    _euler.roll_deg  = _euler.roll  * rad_to_deg;
    _euler.pitch_deg = _euler.pitch * rad_to_deg;
    _euler.yaw_deg   = _euler.yaw   * rad_to_deg;
}

void BNO085::updateContinuousYawAndVelocity(float dt_sec)
{
    constexpr float rad_to_deg = 180.0f / M_PI;

    if (_is_first_yaw) {
        _prev_yaw_rad = _euler.yaw;
        _continuous_yaw_rad = _euler.yaw;
        _continuous_yaw_deg = _euler.yaw_deg;
        _yaw_velocity_deg = 0.0f;
        _is_first_yaw = false;
        return;
    }

    // 前回からの角度変化
    float diff = _euler.yaw - _prev_yaw_rad;

    // -pi ~ +pi の不連続性 (折り返し) をアンラップ
    if (diff > M_PI) {
        diff -= 2.0f * M_PI;
    } else if (diff < -M_PI) {
        diff += 2.0f * M_PI;
    }

    _continuous_yaw_rad += diff;
    _continuous_yaw_deg = _continuous_yaw_rad * rad_to_deg;
    _prev_yaw_rad = _euler.yaw;

    // 角速度推定 (1次ローパスフィルタ適用: 時定数 約25ms)
    if (dt_sec > 1e-4f && dt_sec < 0.2f) {
        float raw_vel = (diff * rad_to_deg) / dt_sec;
        constexpr float alpha = 0.35f;
        _yaw_velocity_deg = alpha * raw_vel + (1.0f - alpha) * _yaw_velocity_deg;
    }
}

void BNO085::resetYawZero()
{
    _yaw_offset_deg = _euler.yaw_deg;
    _continuous_yaw_rad = 0.0f;
    _continuous_yaw_deg = 0.0f;
    _prev_yaw_rad = _euler.yaw;
    _yaw_velocity_deg = 0.0f;
}

float BNO085::getRelativeYawDeg() const noexcept
{
    float rel = _euler.yaw_deg - _yaw_offset_deg;
    if (rel > 180.0f) {
        rel -= 360.0f;
    } else if (rel < -180.0f) {
        rel += 360.0f;
    }
    return rel;
}
