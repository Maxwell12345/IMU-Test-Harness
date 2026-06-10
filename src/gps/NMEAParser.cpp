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

#include "NMEASentenceZDA.hpp"

NMEAParser::NMEAParser() {
    ResetData();
}

NMEAParser::~NMEAParser() = default;

void NMEAParser::ResetData() {
    DataAccessSemaphoreLock();

    m_GGA.ResetData();
    m_GSV.ResetData();
    m_GSA.ResetData();
    m_RMC.ResetData();
    m_HDG.ResetData();
    m_ZDA.ResetData();
    m_HDT.ResetData();

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
    else if (strcmp(pCmd, "GSV") == 0) {
        DataAccessSemaphoreLock();
        m_GSV.ProcessSentence(pCmd, pData);
        DataAccessSemaphoreUnlock();
    }
    else if (strcmp(pCmd, "GSA") == 0) {
        DataAccessSemaphoreLock();
        m_GSA.ProcessSentence(pCmd, pData);
        DataAccessSemaphoreUnlock();
    }
    else if (strcmp(pCmd, "RMC") == 0) {
        DataAccessSemaphoreLock();
        m_RMC.ProcessSentence(pCmd, pData);
        DataAccessSemaphoreUnlock();
    }
    else if (strcmp(pCmd, "HDG") == 0) {
        DataAccessSemaphoreLock();
        m_HDG.ProcessSentence(pCmd, pData);
        DataAccessSemaphoreUnlock();
    }
    else if (strcmp(pCmd, "ZDA") == 0) {
        DataAccessSemaphoreLock();
        m_ZDA.ProcessSentence(pCmd, pData);
        DataAccessSemaphoreUnlock();
    }
    else if (strcmp(pCmd, "HDT") == 0) {
        DataAccessSemaphoreLock();
        bool status = m_HDT.ProcessSentence(pCmd, pData);
        if (status != NMEAParserData::ERROR_E::ERROR_OK) m_HDT.ResetData();
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

const NMEASentenceGSV &NMEAParser::GetGSV() const {
    return m_GSV;
}

const NMEASentenceGSA &NMEAParser::GetGSA() const {
    return m_GSA;
}

const NMEASentenceRMC &NMEAParser::GetRMC() const {
    return m_RMC;
}


const NMEASentenceHDG &NMEAParser::GetHDG() const {
    return m_HDG;
}

const NMEASentenceZDA &NMEAParser::GetZDA() const
{
    return m_ZDA;
}

const NMEASentenceHDT &NMEAParser::GetHDT() const
{
    return m_HDT;
}
