/******************************************************************************
 * File:             NMEAParser.cpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:07 AM
 *
 ******************************************************************************/

#include <cstdio>
#include <cstring>
#include "NMEAParser.hpp"

NMEAParser::NMEAParser() {
    ResetData();
}

NMEAParser::~NMEAParser() = default;

void NMEAParser::ResetData() {
    DataAccessSemaphoreLock();

    m_GGA.ResetData();
    m_RMC.ResetData();

    DataAccessSemaphoreUnlock();
}

NMEAParserData::ERROR_E NMEAParser::ProcessRxCommand(char *pCmd, char *pData) {

    //-----------------------------------------------------------------------------
    if (strcmp(pCmd, "GGA") == 0) {
        DataAccessSemaphoreLock();
        m_GGA.ProcessSentence(pCmd, pData);
        m_GSA.FlagReceivedGGA();
        DataAccessSemaphoreUnlock();
    }
    else if (strcmp(pCmd, "RMC") == 0) {
        DataAccessSemaphoreLock();
        m_RMC.ProcessSentence(pCmd, pData);
        DataAccessSemaphoreUnlock();
    }

    return NMEAParserData::ERROR_OK;
}

NMEAParserData::ERROR_E NMEAParser::ProcessNMEABuffer(const char *pData, size_t nBufferSize) {
    return NMEAParserPacket::ProcessNMEABuffer(pData, nBufferSize);
}

const NMEASentenceGGA &NMEAParser::GetGGA() const {
    return m_GGA;
}

const NMEASentenceRMC &NMEAParser::GetRMC() const {
    return m_RMC;
}


