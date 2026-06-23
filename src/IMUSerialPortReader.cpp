#include "IMUSerialPortReader.hpp"
#include <boost/shared_ptr.hpp>

_IMUSerialPort::_IMUSerialPort(const std::string &path, unsigned int baudRate) : m_serial(m_io) {
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

_IMUSerialPort::~_IMUSerialPort() { this->Close(); }

void _IMUSerialPort::Open(const std::string &port) { this->m_serial.open(port); }

void _IMUSerialPort::SetBaudRate(unsigned int rate) {
  this->m_serial.set_option(boost::asio::serial_port_base::baud_rate(rate));
}

void _IMUSerialPort::Callback() {
  unsigned char byte = 0;

  this->ReadExact(&byte, 1);

  if (!this->IsStartEncoder(byte)) {
    return;
  }

  this->ReadExact(&byte, 1);
  _IMU_MESSAGE_TYPES_ type = this->GetMessageType(byte);

  this->ReadExact(&byte, 1);
  unsigned int len = this->GetMessageLength(byte);

  std::unique_ptr<unsigned char[]> msg(new unsigned char[len]);
  this->ReadExact(msg.get(), len);

  unsigned char checksum[2]{};
  this->ReadExact(checksum, 2);

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
  default:
    throw std::runtime_error("Unsupported IMU message type");
  }
}

void _IMUSerialPort::ReadExact(unsigned char *data, std::size_t len) {
  boost::system::error_code ec;

  boost::asio::read(this->m_serial, boost::asio::buffer(data, len), boost::asio::transfer_exactly(len), ec);

  if (ec) {
    throw boost::system::system_error(ec);
  }
}

void _IMUSerialPort::SetCompletedPayloadCallback(
    std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> callback) {
  this->m_callback = callback;
}

void _IMUSerialPort::Close() {
  if (this->m_serial.is_open()) {
    boost::system::error_code ec;
    this->m_serial.close(ec);
  }
}

unsigned long _IMUSerialPort::CalculateCRC16CCITTFalseChecksum(const unsigned char *payload, unsigned long len) {
  cm_ini(&this->m_cm);
  cm_blk(&this->m_cm, const_cast<unsigned char *>(payload), len);
  return cm_crc(&this->m_cm);
}

IMUSerialPortReader::IMUSerialPortReader(std::string path, unsigned int baudRate)
    : m_serialPort(std::make_shared<_IMUSerialPort>(path, baudRate)), m_serialComService(path, baudRate, m_serialPort),
      m_hasCallback(false) {}

IMUSerialPortReader::IMUSerialPortReader(std::shared_ptr<SerialPortBase> serialPort, std::string path,
                                         unsigned int baudRate)
    : m_serialPort(std::move(serialPort)), m_serialComService(path, baudRate, m_serialPort), m_hasCallback(false) {}
void IMUSerialPortReader::InstallVectorCallback(
    std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> callback) {
  if (auto imuPort = std::dynamic_pointer_cast<_IMUSerialPort>(m_serialPort)) {

    // this->m_serialPort->SetCompletedPayloadCallback(callback);
    imuPort->SetCompletedPayloadCallback(callback);
  }
  this->m_hasCallback = true;
}

void IMUSerialPortReader::Start() {
  if (!this->m_hasCallback) {
    throw std::runtime_error("IMU callback not installed");
  }

  this->m_serialComService.Start();
}

void IMUSerialPortReader::Stop() { this->m_serialComService.Stop(); }
