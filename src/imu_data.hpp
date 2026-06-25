#pragma once

#include <cstdint>
#include <iomanip>
#include <ostream>

/**
 * @brief Raw serial port rotation quaternion with acc. c-struct.
 */
#pragma pack(push, 1)
typedef struct Raw_RotationVectorWAcc {
    float i; 
    float j; 
    float k; 
    float real; 
    float accuracy; 
    uint64_t timestamp;
} Raw_RotationVectorWAcc_t;
#pragma pack(pop)

/**
 * @brief Raw serial port accelerometer vector c-struct.
 */
#pragma pack(push, 1)
typedef struct Raw_Accelerometer {
    float x;
    float y;
    float z;
    uint64_t timestamp;
} Raw_Accelerometer_t;
#pragma pack(pop)

/**
 * @brief Raw 3-axis accelerometer measurement from the BNO085.
 */
struct AccelerometerData {
    uint32_t timestamp_us{0};  ///< Sensor timestamp in microseconds (CLOCK_MONOTONIC)
    float    x{0.0f};          ///< X-axis acceleration (m/s^2)
    float    y{0.0f};          ///< Y-axis acceleration (m/s^2)
    float    z{0.0f};          ///< Z-axis acceleration (m/s^2)
    uint8_t  accuracy{0};      ///< Calibration accuracy: 0=unreliable, 1=low, 2=medium, 3=high

    /**
     * @brief Formats one field per line followed by a blank line.
     * @param[in,out] os Output stream.
     * @param[in]     d  Data to format.
     * @return Reference to @p os.
     */
    friend std::ostream& operator<<(std::ostream& os, const AccelerometerData& d) {
        os << std::fixed << std::setprecision(4)
           << "[Accelerometer] (m/s^2)  t=" << d.timestamp_us << "us\n"
           << "  x:        " << d.x        << "\n"
           << "  y:        " << d.y        << "\n"
           << "  z:        " << d.z        << "\n"
           << "  accuracy: " << static_cast<int>(d.accuracy) << "\n";
        return os;
    }
};

/**
 * @brief Gravity-compensated linear acceleration from the BNO085.
 */
struct LinearAccelData {
    uint32_t timestamp_us{0};  ///< Sensor timestamp in microseconds (CLOCK_MONOTONIC)
    float    x{0.0f};          ///< X-axis linear acceleration (m/s^2)
    float    y{0.0f};          ///< Y-axis linear acceleration (m/s^2)
    float    z{0.0f};          ///< Z-axis linear acceleration (m/s^2)
    uint8_t  accuracy{0};      ///< Calibration accuracy: 0=unreliable, 1=low, 2=medium, 3=high

    /**
     * @brief Formats one field per line followed by a blank line.
     * @param[in,out] os Output stream.
     * @param[in]     d  Data to format.
     * @return Reference to @p os.
     */
    friend std::ostream& operator<<(std::ostream& os, const LinearAccelData& d) {
        os << std::fixed << std::setprecision(4)
           << "[Linear Acceleration] (m/s^2)  t=" << d.timestamp_us << "us\n"
           << "  x:        " << d.x        << "\n"
           << "  y:        " << d.y        << "\n"
           << "  z:        " << d.z        << "\n"
           << "  accuracy: " << static_cast<int>(d.accuracy) << "\n";
        return os;
    }
};

/**
 * @brief Calibrated gyroscope measurement from the BNO085.
 */
struct GyroscopeData {
    uint32_t timestamp_us{0};  ///< Sensor timestamp in microseconds (CLOCK_MONOTONIC)
    float    x{0.0f};          ///< X-axis angular velocity (rad/s)
    float    y{0.0f};          ///< Y-axis angular velocity (rad/s)
    float    z{0.0f};          ///< Z-axis angular velocity (rad/s)

    /**
     * @brief Formats one field per line followed by a blank line.
     * @param[in,out] os Output stream.
     * @param[in]     d  Data to format.
     * @return Reference to @p os.
     */
    friend std::ostream& operator<<(std::ostream& os, const GyroscopeData& d) {
        os << std::fixed << std::setprecision(4)
           << "[Gyroscope] (rad/s)  t=" << d.timestamp_us << "us\n"
           << "  x: " << d.x << "\n"
           << "  y: " << d.y << "\n"
           << "  z: " << d.z << "\n";
        return os;
    }
};

/**
 * @brief Calibrated magnetometer measurement from the BNO085.
 */
struct MagnetometerData {
    uint32_t timestamp_us{0};  ///< Sensor timestamp in microseconds (CLOCK_MONOTONIC)
    float    x{0.0f};          ///< X-axis magnetic field strength (uT)
    float    y{0.0f};          ///< Y-axis magnetic field strength (uT)
    float    z{0.0f};          ///< Z-axis magnetic field strength (uT)
    uint8_t  accuracy{0};      ///< Calibration accuracy: 0=unreliable, 1=low, 2=medium, 3=high

    /**
     * @brief Formats one field per line followed by a blank line.
     * @param[in,out] os Output stream.
     * @param[in]     d  Data to format.
     * @return Reference to @p os.
     */
    friend std::ostream& operator<<(std::ostream& os, const MagnetometerData& d) {
        os << std::fixed << std::setprecision(4)
           << "[Magnetometer] (uT)  t=" << d.timestamp_us << "us\n"
           << "  x:        " << d.x        << "\n"
           << "  y:        " << d.y        << "\n"
           << "  z:        " << d.z        << "\n"
           << "  accuracy: " << static_cast<int>(d.accuracy) << "\n";
        return os;
    }
};

/**
 * @brief Absolute-orientation quaternion from the BNO085 (uses magnetometer for heading).
 */
struct RotationVectorData {
    uint32_t timestamp_us{0};  ///< Sensor timestamp in microseconds (CLOCK_MONOTONIC)
    float    i{0.0f};          ///< Quaternion i (x) component
    float    j{0.0f};          ///< Quaternion j (y) component
    float    k{0.0f};          ///< Quaternion k (z) component
    float    real{0.0f};       ///< Quaternion real (w) component
    float    accuracy{0.0f};   ///< Estimated heading accuracy (rad)

    /**
     * @brief Formats one field per line followed by a blank line.
     * @param[in,out] os Output stream.
     * @param[in]     d  Data to format.
     * @return Reference to @p os.
     */
    friend std::ostream& operator<<(std::ostream& os, const RotationVectorData& d) {
        os << std::fixed << std::setprecision(4)
           << "[Rotation Vector] (quaternion)  t=" << d.timestamp_us << "us\n"
           << "  i:        " << d.i        << "\n"
           << "  j:        " << d.j        << "\n"
           << "  k:        " << d.k        << "\n"
           << "  real:     " << d.real     << "\n"
           << "  accuracy: " << d.accuracy << " rad\n";
        return os;
    }
};

/**
 * @brief Relative-orientation quaternion from the BNO085 (no magnetometer, no absolute heading).
 */
struct GameRotationVectorData {
    uint32_t timestamp_us{0};  ///< Sensor timestamp in microseconds (CLOCK_MONOTONIC)
    float    i{0.0f};          ///< Quaternion i (x) component
    float    j{0.0f};          ///< Quaternion j (y) component
    float    k{0.0f};          ///< Quaternion k (z) component
    float    real{0.0f};       ///< Quaternion real (w) component

    /**
     * @brief Formats one field per line followed by a blank line.
     * @param[in,out] os Output stream.
     * @param[in]     d  Data to format.
     * @return Reference to @p os.
     */
    friend std::ostream& operator<<(std::ostream& os, const GameRotationVectorData& d) {
        os << std::fixed << std::setprecision(4)
           << "[Game Rotation Vector] (quaternion)  t=" << d.timestamp_us << "us\n"
           << "  i:    " << d.i    << "\n"
           << "  j:    " << d.j    << "\n"
           << "  k:    " << d.k    << "\n"
           << "  real: " << d.real << "\n";
        return os;
    }
};
