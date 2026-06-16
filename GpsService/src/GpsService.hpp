#ifndef SERIAL_GPS_SERVICE_HPP
#define SERIAL_GPS_SERVICE_HPP

#include <string>
#include <atomic>
#include <thread>

#include <boost/asio.hpp>
#include <gtest/gtest_prod.h> 

class SerialPortBase;

class GpsService {
public:

    /**
     * @brief Constructor
     *
     * @param [in] path is the file descriptor to the serial com port (ie /dev/serial/by-id/..., etc)
     * @param [in] baudRate how fast data transfer takes place (bits per second) typically 9600 or 
     */
    GpsService(std::string path,
               unsigned int baudRate,
               std::unique_ptr<SerialPortBase> serialPort);

    ~GpsService();

    /**
     * @brief Start Gps Service in a separate thread and assign it to m_runThread.
     *          This thread holds the business logic of reading and multicast NMEA sentences
     *
     * @return
     */
    void Start();

    /**
     * @brief Stop Gps Service in m_runThread
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
     * @brief Invoke Boost Serial Port API and read NMEA data from m_serial
     *
     * @return NMEA sentence
     */
    std::string ReadFromPort();

    std::jthread m_runThread;       // Serial port processing thread
    std::atomic<bool> m_running;    // Processing thread status flag

    boost::asio::io_context m_io;               // Asio context
    std::unique_ptr<SerialPortBase> m_serial;   // Pointer to a DI serial object

    std::string m_path;         // Serial port file descriptor
    unsigned int m_baudRate;    // Serial port baud rate

    FRIEND_TEST(GpsServiceTest, Initial_Values);
    FRIEND_TEST(GpsServiceTest, Service_Loop_Status);
};

#endif