#pragma once

#include <utility>
#include <boost/asio.hpp>

#include "NmeaMessage.hpp"
#include "BoostSerialPort.hpp"
#include "SerialComService.hpp"

class NmeaReader {
public:

    /**
     * @param [in] path com port path
     * @param [in] baud baud rate
     */
    explicit NmeaReader(const std::string& path, int baud);
    ~NmeaReader();

    /**
     * @brief Accessor to m_nmeaMessage
     * 
     * @return snapshot of m_nmeaMessages
     */
    NmeaMessage GetNmeaMessage() const;

    /**
     * @brief accessor of m_nmeaMessageReady. Can be used to check if new nmea came in from com port
     * 
     * @return true if new nmea is received, else false
     */
    bool GetNmeaMessageReady() const;

    /**
     * @brief starts com serial service and decode incoming nmea
     * 
     * @return
     */
    void Start();

    /**
     * @brief stops com serial service
     * 
     * @return
     */
    void Stop();

private:
    /**
     * @brief reads from serial com port and parses for nmea Message.
     * 
     * @remark If valid nmea is recieved, function will set m_nmeaMessage = new data
     *      and m_nmeaMessageReady = true
     * 
     * @param [in] serial boost serial port
     * 
     * @return
     */
    void Callback(SerialPortBase& serial);

    /**
     * @brief parses nmea sentence and return a NmeaMessage struct representation of the nmea sentence
     * 
     * @param [in] line nmea sentence
     * 
     * @return NmeaMessage object with parsed data
     */
    NmeaMessage Parse(const std::string& line);

    /**
     * @brief verifies checksum
     * 
     * @return true if checksum is correct from data, else false
     */
    bool ValidChecksum(const std::string& line);

    /**
     * @brief extracts nmea data to degree.
     * 
     * @remarks negative return value indicate South or West
     * 
     * @param [in] raw raw representation of lat / lon in deg, min
     * @param [in] hemi one of N, S, E, or W direction
     * 
     * @return lat/ lon in degrees
     */
    double ParseDeg(const std::string& raw, const std::string& hemi);

    mutable std::atomic<bool> m_nmeaMessageReady;       // Ready state if m_nmeaMessage is valid and unaccessed
    mutable std::mutex m_nmeaMessageMutex;              // mutex for m_nmeaMessage
    NmeaMessage m_nmeaMessage;                          // parsed nmea sentence

    std::unique_ptr<SerialComService> m_serialComService;
    
};