#ifndef IMU_SERIAL_PORT_READER_HPP
#define IMU_SERIAL_PORT_READER_HPP

#include <gtest/gtest_prod.h> 
#include <optional>
#include "SerialComService.hpp"
#include "SerialPortBase.hpp"
#include "imu_data.hpp"
#include "crc/crc.h"

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <array>

enum _IMU_MESSAGE_TYPES_ {
    ACCELERATION = 0,
    ROTATION_VECTOR = 1
};

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
    _IMUSerialPort(const std::string& path, unsigned int baudRate);

    ~_IMUSerialPort() override;

    /**
     * @brief Opens the serial port.
     *
     * @param [in] port is the file descriptor or COM port name.
     * 
     * @return
     *
     * @exception boost::system::system_error if the serial port cannot be opened.
     */
    void Open(const std::string& port) override;

    /**
     * @brief Sets the baud rate for the serial port.
     *
     * @param [in] rate is the baud rate in bits per second.
     * 
     * @return
     *
     * @exception boost::system::system_error if the baud rate cannot be set.
     */
    void SetBaudRate(unsigned int rate) override;

    /**
     * @brief Reads bytes from the serial port.
     * 
     * @return
     *
     * @exception boost::system::system_error if the serial port read fails.
     */
    void Callback() override;
    /**
     * @brief Passes in completed IMU measurement char* into parent.
     * 
     * @return
     *
     * @exception boost::system::system_error if the serial port read fails.
     */
    void SetCompletedPayloadCallback(std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> callback);

    /**
     * @brief Closes the serial port.
     * 
     * @return
     */
    void Close() override;

private:

    /**
     * @brief Calculates the checksum for a byte array with CRC-16/CCITT-False w/ Polynomial=16 standard.
     * 
     * @param [in] payload is the byte array you want the checksum out of.
     * @param [in] len is the length of the memory allocated to the payload.
     * 
     * @remark This has zero safety and is prone to segment faults. It is on you to ensure that the len matches the pointer memory allocation.
     */
    unsigned long CalculateCRC16CCITTFalseChecksum(const unsigned char* payload, unsigned long len);

    /**
     * @brief Checks for the start encoder of our custom protocol (0xFF).
     * 
     * @param [in] byte is the current byte being read on the serial port.
     * 
     * @return true if byte is start encoder.
     * @return false if byte is not start encoder.
     * 
     * @exception std::exception / segment fault (FATAL) when handling byte reading.
     * 
     * @remark
     */
    inline bool IsStartEncoder(const unsigned char &byte) {
        return byte == 0xA5;
    }

    /**
     * @brief Extracts and maps message type after start encoder found.
     * 
     * @param [in] byte is the current byte being read on the serial port.
     * 
     * @return _IMU_MESSAGE_TYPES_ ENUM for message type.
     * 
     * @exception std::exception / segment fault (FATAL) when handling byte reading OR message type not available.
     * 
     * @remark
     */
    inline _IMU_MESSAGE_TYPES_ GetMessageType(const unsigned char &byte) {
        switch (byte)
            {
            case 0x01:
                return _IMU_MESSAGE_TYPES_::ACCELERATION;
                
            case 0x02:
                return _IMU_MESSAGE_TYPES_::ROTATION_VECTOR;
            
            default:
                throw std::runtime_error("Unsupported type");
            }
    }

    /**
     * @brief Extracts message length byte and returns as an unsigned.
     * 
     * @param [in] byte is the current byte being read on the serial port.
     * 
     * @return message length in bytes.
     * 
     * @exception std::exception / segment fault (FATAL) when handling byte reading.
     * 
     * @remark
     */
    inline unsigned int GetMessageLength(const unsigned char &byte) {
        unsigned int len = byte;

        return len;
    }

    /**
     * @brief Validates the message payload against the checksum in the packet.
     * 
     * @param [in] messageChecksum is the checksum 2 byte pointer (final 2 bytes in packet).
     * @param [in] message is the raw byte payload.
     * @param [in] messageLen is the len in bytes of the message payload.
     * 
     * @return true if message CRC-16/CCITT-False checksum matches the packet checksum.
     * @return false if the message CRC-16/CCITT-False checksum does not match the packet checksum.
     * 
     * @exception std::exception / segment fault (FATAL) when handling byte reading.
     * 
     * @remark
     */
    inline bool ValidateMessage(const unsigned char* messageChecksum, const unsigned char* message, unsigned int messageLen) {
        unsigned long realChecksum = CalculateCRC16CCITTFalseChecksum(message, messageLen);

        unsigned int receivedChecksum = (messageChecksum[0] << 8) | messageChecksum[1];

        if (realChecksum != receivedChecksum) {
            return false;
        }

        return true;
    }

    /**
     * @brief Reads exactly len bytes from the serial port into the provided buffer.
     * 
     * @param [out] data is the destination byte pointer where serial data will be written.
     * @param [in] len is the exact number of bytes to read from the serial port.
     * 
     * @return
     * 
     * @exception boost::system::system_error if the serial port read fails, the port is closed,
     *            or the requested number of bytes cannot be read.
     * 
     * @remark This has zero memory safety. It is on you to ensure that data points to at least
     *         len bytes of valid writable memory.
     */
    void ReadExact(unsigned char* data, std::size_t len);

private:
    boost::asio::io_context m_io;
    boost::asio::serial_port m_serial;

    std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> m_callback;

    cm_t m_cm;

    FRIEND_TEST(_IMUSerialPortTest, ValidateCalculateCRC16CCITTFalseChecksum);
    FRIEND_TEST(_IMUSerialPortTest, ValidateIsStartEncoder);
    FRIEND_TEST(_IMUSerialPortTest, ValidateGetMessageType);
    FRIEND_TEST(_IMUSerialPortTest, ValidateGetMessageLength);
    FRIEND_TEST(_IMUSerialPortTest, ValidateValidateMessage);
    FRIEND_TEST(_IMUSerialPortTest, ValidateSetBaudRate);
    FRIEND_TEST(_IMUSerialPortTest, ValidateOpen);
    FRIEND_TEST(_IMUSerialPortTest, ValidateClose);
    FRIEND_TEST(_IMUSerialPortTest, ValidateConstructor);
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
    std::shared_ptr<_IMUSerialPort> m_serialPort;
    SerialComService m_serialComService;

    std::function<void(std::optional<Raw_RotationVectorWAcc>, std::optional<Raw_Accelerometer>)> m_callback;
    std::atomic<bool> m_hasCallback;
};

#endif // IMU_SERIAL_PORT_READER_HPP