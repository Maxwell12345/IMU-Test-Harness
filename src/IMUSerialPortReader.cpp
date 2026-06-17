#include <boost/shared_ptr.hpp>
#include "IMUSerialPortReader.hpp"

IMUSerialPortReader::IMUSerialPortReader(std::string path, unsigned int baudRate, std::unique_ptr<SerialPortBase> serialPort)
    : m_serialComService(path, baudRate, std::move(serialPort))
{
    this->m_cm.cm_width = 16;
    this->m_cm.cm_poly = 0x1021L;
    this->m_cm.cm_init = 0xFFFFL;
    this->m_cm.cm_refin = 0;
    this->m_cm.cm_refot = 0;
    this->m_cm.cm_xorot = 0x0000L;
}

unsigned long IMUSerialPortReader::CalculateCRC16CCITTFalseChecksum(unsigned char* payload, unsigned long len) {
    cm_ini(&this->m_cm);
    cm_blk(&this->m_cm, payload, len);
    return cm_crc(&this->m_cm);
}