#ifndef SERIAL_COM_SERVICE_HPP
#define SERIAL_COM_SERVICE_HPP

#include <string>
#include <atomic>
#include <functional>
#include <thread>

#include <boost/asio.hpp>
#include <gtest/gtest_prod.h> 

class SerialPortBase;

class SerialComService {
public:

    /**
     * @brief Constructor
     *
     * @param [in] path is the file descriptor to the serial com port (ie /dev/serial/by-id/..., etc)
     * @param [in] baudRate how fast data transfer takes place (bits per second) typically 9600 or 
     */
    SerialComService(std::string path,
                     unsigned int baudRate,
                     std::shared_ptr<SerialPortBase> serialPort);

    ~SerialComService();

    /**
     * @brief Starts com Service in a separate thread and assign it to m_runThread.
     *          This thread holds the business logic of reading com data and apply
     *          callback on it.
     *
     * @return
     */
    void Start();

    /**
     * @brief Stops m_runThread
     *
     * @return
     */
    void Stop();

private:

    /**
     * @brief Invoke Boost Serial Port API to prepare reading serial data from COM port at m_path using m_baudRate
     *
     * @return
     */
    void ConfigureSerialPort();

    /**
     * @brief Verify POSIX serial com path is valid
     *
     * @remark POSIX path regex must be of the form ^/dev/[^\s]+$
     *
     * @param [in] path that path to the serial com port
     *
     * @return true if path has the valid regex, else false
     */
    static bool VerifyPath(const std::string& path);

    std::jthread m_runThread;                       // Serial port processing thread
    std::atomic<bool> m_running;                    // Processing thread status flag

    boost::asio::io_context m_io;               // Asio context
    std::shared_ptr<SerialPortBase> m_serial;   // Pointer to a DI serial object. This serial contains the callback.

    std::string m_path;         // Serial port file descriptor
    unsigned int m_baudRate;    // Serial port baud rate

    FRIEND_TEST(SerialComServiceTest, InitialValues);
    FRIEND_TEST(SerialComServiceTest, ServiceLoopStatus);
    FRIEND_TEST(SerialComServiceTest, VerifyPathReturnsTrue);
    FRIEND_TEST(SerialComServiceTest, VerifyPathReturnsFalse);
};

#endif