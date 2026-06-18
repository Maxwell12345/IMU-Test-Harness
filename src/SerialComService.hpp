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
     * @brief Invoke Boost Serial Port API and read data from m_serial
     *
     * @return Com data as a string
     */
    std::string ReadFromPort();

    std::jthread m_runThread;                       // Serial port processing thread
    std::atomic<bool> m_running;                    // Processing thread status flag

    boost::asio::io_context m_io;               // Asio context
    std::shared_ptr<SerialPortBase> m_serial;   // Pointer to a DI serial object. This serial contains the callback.

    std::string m_path;         // Serial port file descriptor
    unsigned int m_baudRate;    // Serial port baud rate

    FRIEND_TEST(SerialComServiceTest, Initial_Values);
    FRIEND_TEST(SerialComServiceTest, Service_Loop_Status);
};

#endif