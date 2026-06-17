#ifndef IMU_SERIAL_PORT_READER_HPP
#define IMU_SERIAL_PORT_READER_HPP

#include <gtest/gtest_prod.h> 
#include <optional>
#include "SerialComService.hpp"
#include "SerialPort/SerialPortBase.hpp"
#include "imu_data.hpp"
#include "crc/crc.h"

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <array>

enum _IMU_MESSAGE_TYPES_ {
    ACCELERATION = 0,
    ROTATION_VECTOR = 1
}

class _IMUSerialPort : public SerialPortBase {
public:
    /**
     * @brief Constructor
     *
     * @param [in] path is the file descriptor to the serial com port
     *                  such as /dev/ttyUSB0, /dev/ttyACM0, /dev/serial/by-id/..., COM3,
     *                  or \\\\.\\COM10.
     * 
     * @param [in] baudRate how fast data transfer takes place in bits per second,
     *                      typically 9600 or 115200.
     *
     * @exception boost::system::system_error if the serial port is not available
     *            or configuration fails.
     */
    _IMUSerialPort(const std::string& path, unsigned int baudRate) : m_serial(m_io) {
        this->Open(path);
        this->SetBaudRate(baudRate);
    }

    ~_IMUSerialPort() override {
        this->Close();
    }

    /**
     * @brief Opens the serial port.
     *
     * @param [in] port is the file descriptor or COM port name.
     * 
     * @return
     *
     * @exception boost::system::system_error if the serial port cannot be opened.
     */
    void Open(const std::string& port) override {
        this->m_serial.open(port);
    }

    /**
     * @brief Sets the baud rate for the serial port.
     *
     * @param [in] rate is the baud rate in bits per second.
     * 
     * @return
     *
     * @exception boost::system::system_error if the baud rate cannot be set.
     */
    void SetBaudRate(unsigned int rate) override {
        this->m_serial.set_option(boost::asio::serial_port_base::baud_rate(rate));
    }

    /**
     * @brief Reads bytes from the serial port.
     * 
     * @return
     *
     * @exception boost::system::system_error if the serial port read fails.
     */
    void Callback() override {
        std::array<char, 1> buffer{};

        boost::system::error_code ec;
        std::size_t n = this->m_serial.read_some(boost::asio::buffer(buffer), ec);

        if (ec) {
            throw boost::system::system_error(ec);
        }

        if (buffer == 0xFF) {
            n = this->m_serial.read_some(boost::asio::buffer(buffer), ec);

            if (ec) {
                throw boost::system::system_error(ec);
            }

            _IMU_MESSAGE_TYPES_ type;

            switch (buffer)
            {
            case 0x00:
                type = _IMU_MESSAGE_TYPES_::ACCELERATION;
                break;
                
            case 0x01:
                type = _IMU_MESSAGE_TYPES_::ROTATION_VECTOR;
                break;
            
            default:
                throw std::runtime_error("Unsupported type");
                break;
            }

            n = this->m_serial.read_some(boost::asio::buffer(buffer), ec);
            if (ec) {
                throw boost::system::system_error(ec);
            }

            unsigned int len = (unsigned int) buffer;

            std::array<char, len> msg{};
            n = this->m_serial.read_some(boost::asio::buffer(msg), ec);
            if (ec) {
                throw boost::system::system_error(ec);
            }

            std::array<char, 2> checksum{};
            n = this->m_serial.read_some(boost::asio::buffer(checksum), ec);
            if (ec) {
                throw boost::system::system_error(ec);
            }

            unsigned long realChecksum = CalculateCRC16CCITTFalseChecksum(reinterpret_cast<unsigned char*>(msg.data()), len);

            if (realChecksum != reinterpret_cast<unsigned char*>(checksum.data())) {

            }


        }

        std::cout.write(.data(), n);
        std::cout.flush();
    }

    /**
     * @brief Passes in completed IMU measurement char* into parent.
     * 
     * @return
     *
     * @exception boost::system::system_error if the serial port read fails.
     */
    void SetCompletedPayloadCallback(std::function<void(char*)> callback) {
        this->m_callback = callback;
    }

    /**
     * @brief Closes the serial port.
     * 
     * @return
     */
    void Close() override {
        if (this->m_serial.is_open()) {
            boost::system::error_code ec;
            this->m_serial.close(ec);
        }
    }

private:

    /**
     * @brief Calculates the checksum for a byte array with CRC-16/CCITT-False w/ Polynomial=16 standard.
     * 
     * @param payload is the byte array you want the checksum out of.
     * @param len is the length of the memory allocated to the payload.
     * 
     * @remark This has zero safety and is prone to segment faults. It is on you to ensure that the len matches the pointer memory allocation.
     */
    unsigned long CalculateCRC16CCITTFalseChecksum(unsigned char* payload, unsigned long len) {
        cm_ini(&this->m_cm);
        cm_blk(&this->m_cm, payload, len);
        return cm_crc(&this->m_cm);
    }

private:
    boost::asio::io_context m_io;
    boost::asio::serial_port m_serial;

    std::function<void(char*)> m_callback;

    cm_t m_cm;

    FRIEND_TEST(IMUSerialPortReaderTest, ValidateCalculateCRC16CCITTFalseChecksum);
};

class IMUSerialPortReader {
public:
    /**
     * @brief Constructor
     *
     * @param [in] path is the file descriptor to the serial com port (ie /dev/serial/by-id/..., etc)
     * @param [in] baudRate how fast data transfer takes place (bits per second) typically 9600 or 
     * 
     * @exception std::exception if serial port is not available or errors out.
     */
    IMUSerialPortReader(std::string path, unsigned int baudRate);
    
    /**
     * @brief Callback to pass IMU data.
     * 
     * @param [in] callback is the callback lambda you want data passed into.
     * 
     * @return
     * 
     * @remark Beware of timing. There will be a delay between when the measurement was produced and real machine time. Use the timestamp in the payloads.
     */
    void InstallVectorCallback(std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)>);

    /**
     * @brief Starts the serial port listener thread, decodes IMU serial data, and passes entries to callback.
     * 
     * @return
     */
    void Start();

    /**
     * @brief Stops the serial port listening service.
     * 
     * @return
     */
    void Stop();

private:
    SerialComService m_serialComService;

    std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> m_callback;
    std::atomic<bool> m_hasCallback;

    std::unique_ptr<SerialPortBase> m_serialPort;
};

#endif // IMU_SERIAL_PORT_READER_HPP