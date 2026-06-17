#include <boost/shared_ptr.hpp>
#include "IMUSerialPortReader.hpp"

IMUSerialPortReader::IMUSerialPortReader(std::string path, unsigned int baudRate)
    : m_serialPort(_IMUSerialPort()), m_serialComService(path, baudRate, m_serialPort)
{
    this->m_serialPort.SetCompletedPayloadCallback();
    this->m_cm.cm_width = 16;
    this->m_cm.cm_poly = 0x1021L;
    this->m_cm.cm_init = 0xFFFFL;
    this->m_cm.cm_refin = 0;
    this->m_cm.cm_refot = 0;
    this->m_cm.cm_xorot = 0x0000L;

    

}

void IMUSerialPortReader::Start() {
    
}


void IMUSerialPortReader::Stop() {

}
