#ifndef NMEA_SERVICE_HPP
#define NMEA_SERVICE_HPP

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <gtest/gtest_prod.h> 

class SerialPortBase;
class SerialComService;
class MulticastService;

class NmeaService {
public:
    NmeaService(std::unique_ptr<SerialComService> serialComService,
                std::unique_ptr<MulticastService> multicastService);

    ~NmeaService();
    
    /**
     * @brief Starts nmea service
     * 
     * @remark Because this class depends on serial com port reading, this method
     *      actually calls the underlying m_serialComService->Start().
     * 
     * @return
     */
    void Start();

    /**
     * @brief Stops nmea service
     * 
     * @remark Because this class depends on serial com port reading, this method
     *      actually calls the underlying m_serialComService->Stop().
     * 
     * @return
     */
    void Stop();

private:

    /**
     * @brief XOR characters between $ and * exclusively and compare to the checksum included
     *      in the payload
     * 
     * @param [in] nmea the nmea sentence without CRLF
     * 
     * @return true if paylaod checksum matches the calculated checksum, else false
     */
    static bool IsValidChecksum(const std::string& nmea);
    
    /**
     * @brief nmea string validity check
     * 
     * @param [in] nmea string ot be checked
     * 
     * @return true if valid, else false
     */
    static bool IsValidNmea(const std::string& nmea);
    
    /**
     * @brief On com read handler
     * 
     * @remark this function handles reading from port and processing read data.
     * 
     * @param [in] port is the underlying port instance. Can be used to read data from.
     * 
     * @return
     */
    void Callback(SerialPortBase& port);

    std::atomic<bool> m_running;    //Status flag, true when service is running, else false

    std::mutex m_nmeaMutex;     // Mutex to m_nmea
    std::string m_nmea;         // Latest read nmea sentence from com port

    std::unique_ptr<SerialComService> m_serialComService;       //used to read from serial com port
    std::unique_ptr<MulticastService> m_multicastService;       //used to multicast m_nmea sentence upon successful com read

    FRIEND_TEST(NmeaServiceTest, IsValidNmeaTestReturnsTrue);
    FRIEND_TEST(NmeaServiceTest, IsValidNmeaTestReturnsFalse);
    FRIEND_TEST(NmeaServiceTest, IsValidChecksumReturnsTrue);
    FRIEND_TEST(NmeaServiceTest, IsValidChecksumReturnsFalse);
};

#endif