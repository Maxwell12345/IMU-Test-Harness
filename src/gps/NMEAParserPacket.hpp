/******************************************************************************
 * File:             NMEAParserPacket.hpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:06 AM
 * Description:      This class will parse NMEA data packet and call its virtual processor methods.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_NMEAPARSERPACKET_HPP
#define INU_DISPLAY_NMEAPARSERPACKET_HPP

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include "NMEAParserData.hpp"

class NMEAParserPacket {
public:
    NMEAParserPacket();

    ~NMEAParserPacket();

    void Reset();

    virtual void OnError(NMEAParserData::ERROR_E nError, char *pCmd) {
        UNUSED_PARAM(nError);
        UNUSED_PARAM(pCmd);
    }

    virtual NMEAParserData::ERROR_E ProcessNMEABuffer(const char *pData, size_t nBufferSize);

protected:
    virtual NMEAParserData::ERROR_E ProcessRxCommand(char *pCmd, char *pData) = 0;

    virtual void TimeTag() {}

private:
    enum PARSE_STATE {
        PARSE_STATE_SOM = 0,
        PARSE_STATE_CMD,
        PARSE_STATE_DATA,
        PARSE_STATE_CHECKSUM_1,
        PARSE_STATE_CHECKSUM_2,
    };

private:
    PARSE_STATE m_nState;
    uint8_t m_u8Checksum;
    uint8_t m_u8ReceivedChecksum;
    uint16_t m_nIndex;
    char m_pCommand[NMEAParserData::c_uMaxCmdLen]{};
    char m_pTalker[NMEAParserData::c_uMaxCmdLen]{};
    char m_pSentenceType[NMEAParserData::c_uMaxCmdLen]{};
    char m_pData[NMEAParserData::c_uMaxDataLen]{};
    const std::unordered_set<std::string> m_allowedTalkers = {"GP", "GA", "GN", "GL", "II"};   // strictly reject any satellite constellation source that isn't: US-governed GPS, galileo, multi-source/combined, GLONASS, or integrated instrumentation

};

#endif //INU_DISPLAY_NMEAPARSERPACKET_HPP
