#include "IMUSerialPortReader.hpp"

IMUSerialPortReader::IMUSerialPortReader(std::string path, unsigned int baudRate, std::unique_ptr<SerialPortBase> port)
{
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
    m_serialComService = std::make_unique<SerialComService>(path,
                                                            baudRate,
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
    unsigned char byte = 0;

    port.ReadExact(&byte, 1);

    if (!this->IsStartEncoder(byte)) {
        return;
    }

    port.ReadExact(&byte, 1);
    _IMU_MESSAGE_TYPES_ type = this->GetMessageType(byte);

    port.ReadExact(&byte, 1);
    unsigned int len = this->GetMessageLength(byte);

    std::unique_ptr<unsigned char[]> msg(new unsigned char[len]);
    port.ReadExact(msg.get(), len);

    unsigned char checksum[2]{};
    port.ReadExact(checksum, 2);

    if (!this->ValidateMessage(checksum, msg.get(), len)) {
        return;
    }

    switch (type) { 
        case _IMU_MESSAGE_TYPES_::ACCELERATION: { 
            Raw_Accelerometer accel{}; 
            std::memcpy(&accel, msg.get(), sizeof(Raw_Accelerometer)); 
            this->m_callback(std::nullopt, accel); 
            break; 
        } 
        case _IMU_MESSAGE_TYPES_::ROTATION_VECTOR: { 
            Raw_RotationVectorWAcc rot{}; 
            std::memcpy(&rot, msg.get(), sizeof(Raw_RotationVectorWAcc)); 
            this->m_callback(rot, std::nullopt); 
            break; 
        } 
        default: throw std::runtime_error("Unsupported IMU message type"); }
}

unsigned long IMUSerialPortReader::CalculateCRC16CCITTFalseChecksum(const unsigned char* payload, unsigned long len) {
    cm_ini(&this->m_cm);
    cm_blk(&this->m_cm, const_cast<unsigned char*>(payload), len);
    return cm_crc(&this->m_cm);
}
