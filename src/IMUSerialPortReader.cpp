#include <boost/shared_ptr.hpp>
#include "IMUSerialPortReader.hpp"

_IMUSerialPort::_IMUSerialPort(const std::string& path, unsigned int baudRate) : m_serial(m_io) {
    this->Open(path);
    this->SetBaudRate(baudRate);

    // Set the CRC-16/CCITT w Polynomial=16
    this->m_cm.cm_width = 16;
    this->m_cm.cm_poly = 0x1021L;
    this->m_cm.cm_init = 0xFFFFL;
    this->m_cm.cm_refin = 0;
    this->m_cm.cm_refot = 0;
    this->m_cm.cm_xorot = 0x0000L;
}

_IMUSerialPort::~_IMUSerialPort() {
    this->Close();
}

void _IMUSerialPort::Open(const std::string& port) {
    if (this->m_serial.is_open()) {
        return;
    }

    this->m_serial.open(port);
}

void _IMUSerialPort::SetBaudRate(unsigned int rate) {
    this->m_serial.set_option(boost::asio::serial_port_base::baud_rate(rate));
}

void _IMUSerialPort::Callback() {
    unsigned char message[50] = {};

    this->ReadExact(message, 1);

    if (!this->IsStartEncoder(message[0])) {
        return;
    }

    this->ReadExact(message + 1, 2);

    _IMU_MESSAGE_TYPES_ type = this->GetMessageType(message[1]);
    unsigned int len = this->GetMessageLength(message[2]);

    if (len > sizeof(message) - 3) {
        return;
    }

    this->ReadExact(message + 3, len);

    unsigned char checksum[2] = {};
    this->ReadExact(checksum, 2);

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

void _IMUSerialPort::ReadExact(unsigned char* data, std::size_t len) {
    boost::system::error_code ec;

    boost::asio::read(this->m_serial, boost::asio::buffer(data, len), boost::asio::transfer_exactly(len), ec);

    if (ec) {
        throw boost::system::system_error(ec);
    }
}

void _IMUSerialPort::SetCompletedPayloadCallback(std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> callback) {
    this->m_callback = callback;
}

void _IMUSerialPort::Close() {
    if (this->m_serial.is_open()) {
        boost::system::error_code ec;
        this->m_serial.close(ec);
    }
}

unsigned long _IMUSerialPort::CalculateCRC16CCITTFalseChecksum(const unsigned char* payload, unsigned long len) {
    cm_ini(&this->m_cm);
    cm_blk(&this->m_cm, const_cast<unsigned char*>(payload), len);
    return cm_crc(&this->m_cm);
}

IMUSerialPortReader::IMUSerialPortReader(std::string path, unsigned int baudRate)
    : m_serialPort(std::make_shared<_IMUSerialPort>(path, baudRate)),
      m_serialComService(path, baudRate, m_serialPort),
      m_hasCallback(false)
{
}

void IMUSerialPortReader::InstallVectorCallback(std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> callback) {
    this->m_serialPort->SetCompletedPayloadCallback(callback);
    this->m_hasCallback = true;
}

void IMUSerialPortReader::Start() {
    if (!this->m_hasCallback) {
        throw std::runtime_error("IMU callback not installed");
    }

    this->m_serialComService.Start();
}

void IMUSerialPortReader::Stop() {
    this->m_serialComService.Stop();
}
