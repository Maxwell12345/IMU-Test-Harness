#ifndef SERIAL_DATA_MODEL_HPP
#define SERIAL_DATA_MODEL_HPP

typedef struct RotationVectorWAcc {
    float i;  /**< @brief Quaternion component i */
    float j;  /**< @brief Quaternion component j */
    float k;  /**< @brief Quaternion component k */
    float real;  /**< @brief Quaternion component, real */
    float accuracy;  /**< @brief Accuracy estimate [radians] */
    uint64_t timestamp;
} RotationVectorWAcc_t;

typedef struct LinearAcceleration {
    float x;
    float y;
    float z;
    uint64_t timestamp;
} LinearAcceleration_t;

#endif