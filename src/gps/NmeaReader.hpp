/******************************************************************************
 * File:             NmeaReader.hpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Declares serial NMEA sentence reception and reusable
 *                   parsing utilities for live and recorded GPS inputs.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_NMEAREADER_HPP
#define INU_DISPLAY_NMEAREADER_HPP
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

    /**
     *
     * @brief   Parses a complete NMEA sentence into the project's NmeaMessage representation after validating its checksum.
     *          RMC sentences provide fix validity, coordinates, speed, and course; GGA sentences provide fix quality,
     *          coordinates, satellite count, dilution, and altitude.
     *
     * @param [in] line  Complete NMEA sentence beginning with '$' and containing a two-digit hexadecimal checksum after '*'.
     *
     * @return  Parsed NmeaMessage. The returned message has validChecksum set to false and retains default measurement fields
     *          when the input checksum or supported sentence layout is invalid.
     *
     * @throws  std::bad_alloc  If memory allocation for sentence fields fails.
     */
    static NmeaMessage Parse(const std::string& line);

    /**
     *
     * @brief   Computes the NMEA XOR checksum over the sentence body and compares it with the two hexadecimal checksum digits
     *          following '*'.
     *
     * @param [in] line  Complete NMEA sentence to validate.
     *
     * @return  True when the sentence framing and checksum are valid; otherwise false.
     */
    static bool ValidChecksum(const std::string& line);

    /**
     *
     * @brief   Converts an NMEA degrees-and-minutes coordinate and hemisphere into signed decimal degrees.
     *
     * @param [in] raw   Coordinate encoded as degrees followed by decimal minutes.
     * @param [in] hemi  Hemisphere designator: N, S, E, or W.
     *
     * @return  Signed decimal degrees, negative for S or W. Returns zero when raw is empty.
     */
    static double ParseDeg(const std::string& raw, const std::string& hemi);

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

    mutable std::atomic<bool> m_nmeaMessageReady;       // Ready state if m_nmeaMessage is valid and unaccessed
    mutable std::mutex m_nmeaMessageMutex;              // mutex for m_nmeaMessage
    NmeaMessage m_nmeaMessage;                          // parsed nmea sentence

    std::unique_ptr<SerialComService> m_serialComService;
    
};

#endif // INU_DISPLAY_NMEAREADER_HPP
