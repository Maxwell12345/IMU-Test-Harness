#include <thread>

#include "IMUSerialPortReader.hpp"

IMUSerialPortReader::IMUSerialPortReader(const _ImuSerialPort& config, std::unique_ptr<SerialPortBase> port){
    // Set the CRC-16/CCITT w Polynomial=16
    this->m_cm.cm_width = 16;
    this->m_cm.cm_poly = 0x1021L;
    this->m_cm.cm_init = 0xFFFFL;
    this->m_cm.cm_refin = 0;
    this->m_cm.cm_refot = 0;
    this->m_cm.cm_xorot = 0x0000L;

    auto f = [this](SerialPortBase& _port){
        this->Callback(_port);
    };
    m_serialComService = std::make_unique<SerialComService>(config.path,
                                                            config.baudRate,
                                                            std::move(port));
    m_serialComService->InstallCallback(f);
}

void IMUSerialPortReader::InstallCallback(std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> callback) {
    this->m_callback = callback;
}

void IMUSerialPortReader::Start() {
    if (!m_callback) {
        throw std::runtime_error("IMU callback not installed");
    }

    this->m_serialComService->Start();
}

void IMUSerialPortReader::Stop() {
    this->m_serialComService->Stop();
}

void IMUSerialPortReader::Callback(SerialPortBase& port) {
    try {
        unsigned char message[75] = {};

        port.ReadExact(message, 1);

        if (!this->IsStartEncoder(message[0])) {
            return;
        }

        port.ReadExact(message + 1, 2);

        _IMU_MESSAGE_TYPES_ type = this->GetMessageType(message[1]);
        unsigned int len = this->GetMessageLength(message[2]);

        if (len > 73) {
            return;
        }

        port.ReadExact(message + 3, len);

        unsigned char checksum[2] = {};
        port.ReadExact(checksum, 2);

        if (!this->ValidateMessage(checksum, message, 3 + len)) {
            return;
        }

        switch (type) {
            case _IMU_MESSAGE_TYPES_::ACCELERATION: {
                if (len != sizeof(Raw_Accelerometer)) {
                    return;
                }

                Raw_Accelerometer accel = {};
                std::memcpy(&accel, message + 3, sizeof(accel));

                if (this->m_callback) {
                    this->m_callback(std::nullopt, accel);
                }

                return;
            }

            case _IMU_MESSAGE_TYPES_::ROTATION_VECTOR: {
                if (len != sizeof(Raw_RotationVectorWAcc)) {
                    return;
                }

                Raw_RotationVectorWAcc rot = {};
                std::memcpy(&rot, message + 3, sizeof(rot));

                if (this->m_callback) {
                    this->m_callback(rot, std::nullopt);
                }

                return;
            }

            default:
                return;
        }
    }
    catch(const std::exception& e) {
        std::cout << "Callback in IMU com port class threw an exception: " << e.what() << std::endl;
    }
}

unsigned long IMUSerialPortReader::CalculateCRC16CCITTFalseChecksum(const unsigned char* payload, unsigned long len) {
    cm_ini(&this->m_cm);
    cm_blk(&this->m_cm, const_cast<unsigned char*>(payload), len);
    return cm_crc(&this->m_cm);
}
