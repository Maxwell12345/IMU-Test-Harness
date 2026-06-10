/******************************************************************************
 * File:             NMEAParser.hpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 8:54 AM
 * Description:      This class will parse NMEA data, store its data and report that it has received data.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_NMEAPARSER_HPP
#define INU_DISPLAY_NMEAPARSER_HPP

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>

#include "NMEAParserData.hpp"
#include "NMEAParserPacket.hpp"
#include "NMEASentenceGGA.hpp"
#include "NMEASentenceRMC.hpp"

class NMEAParser : public NMEAParserPacket {
public:
    NMEAParser();

    virtual ~NMEAParser();

    void ResetData();

    [[nodiscard]] const NMEASentenceGGA &GetGGA() const;

    [[nodiscard]] const NMEASentenceRMC &GetRMC() const;

    NMEAParserData::ERROR_E ProcessNMEABuffer(const char *pData, size_t nBufferSize) override;

protected:
    NMEAParserData::ERROR_E ProcessRxCommand(char *pCmd, char *pData) override;

    virtual void DataAccessSemaphoreLock() {}

    virtual void DataAccessSemaphoreUnlock() {}

private:

    // GPS Sentence Types
    NMEASentenceGGA m_GGA;                                                ///< GPGGA Specific sentence data
    NMEASentenceRMC m_RMC;                                                ///< GPRMC Recommended minimum data for GPS
};

#endif //INU_DISPLAY_NMEAPARSER_HPP
