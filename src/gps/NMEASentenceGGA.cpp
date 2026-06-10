/******************************************************************************
 * File:             NMEASentenceGGA.cpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:14 AM
 *
 ******************************************************************************/

#include <cstdlib>
#include "NMEASentenceGGA.hpp"

NMEASentenceGGA::NMEASentenceGGA() :
        m_nOldVSpeedSeconds(0),
        m_dOldVSpeedAlt(0.0) {
    ResetData();
}

NMEASentenceGGA::~NMEASentenceGGA() = default;

NMEAParserData::ERROR_E NMEASentenceGGA::ProcessSentence(char *pCmd, char *pData) {
    UNUSED_PARAM(pCmd);
    char szField[c_nMaxField];
    char szTemp[c_nMaxField];

    if (GetField(pData, szField, 0, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_nHour = (szField[0] - '0') * 10 + (szField[1] - '0');
        m_sentenceData.m_nMinute = (szField[2] - '0') * 10 + (szField[3] - '0');
        m_sentenceData.m_nSecond = (szField[4] - '0') * 10 + (szField[5] - '0');
    }

    if (GetField(pData, szField, 1, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dLatitude = atof((char *) szField + 2) / 60.0;
        szField[2] = '\0';
        m_sentenceData.m_dLatitude += atof((char *) szField);

    }
    if (GetField(pData, szField, 2, c_nMaxField) == NMEAParserData::ERROR_OK) {
        if (szField[0] == 'S') {
            m_sentenceData.m_dLatitude = -m_sentenceData.m_dLatitude;
        }
    }

    if (GetField(pData, szField, 3, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dLongitude = atof((char *) szField + 3) / 60.0;
        szField[3] = '\0';
        m_sentenceData.m_dLongitude += atof((char *) szField);
    }
    if (GetField(pData, szField, 4, c_nMaxField) == NMEAParserData::ERROR_OK) {
        if (szField[0] == 'W') {
            m_sentenceData.m_dLongitude = -m_sentenceData.m_dLongitude;
        }
    }

    if (GetField(pData, szField, 5, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_nGPSQuality = (NMEAParserData::GPS_QUALITY_E) ((char) szField[0] - '0');
    }

    if (GetField(pData, szField, 6, c_nMaxField) == NMEAParserData::ERROR_OK) {
        szTemp[0] = szField[0];
        szTemp[1] = szField[1];
        szTemp[2] = '\0';
        m_sentenceData.m_nSatsInView = atoi(szTemp);
    }

    if (GetField(pData, szField, 7, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dHDOP = atof((char *) szField);
    }

    if (GetField(pData, szField, 8, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dAltitudeMSL = atof((char *) szField);
    }

    if (GetField(pData, szField, 9, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dGeoidalSep = atof((char *) szField);
    }

    if (GetField(pData, szField, 11, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dDifferentialAge = atof((char *) szField);
    }

    if (GetField(pData, szField, 13, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_nDifferentialID = atoi((char *) szField);
    }

    int nSeconds = (int) m_sentenceData.m_nMinute * 60 + (int) m_sentenceData.m_nSecond;
    if (nSeconds > m_nOldVSpeedSeconds) {
        double dDiff = (double) (m_nOldVSpeedSeconds - nSeconds);
        double dVal = dDiff / 60.0;
        if (dVal != 0.0) {
            m_sentenceData.m_dVertSpeed = (m_dOldVSpeedAlt - m_sentenceData.m_dAltitudeMSL) / dVal;
        }
    }
    m_dOldVSpeedAlt = m_sentenceData.m_dAltitudeMSL;
    m_nOldVSpeedSeconds = nSeconds;

    m_uRxCount++;

    return NMEAParserData::ERROR_OK;
}

void NMEASentenceGGA::ResetData(void) {
    m_uRxCount = 0;
    m_sentenceData.m_dAltitudeMSL = 0.0;
    m_sentenceData.m_dDifferentialAge = 0.0;
    m_sentenceData.m_dGeoidalSep = 0.0;
    m_sentenceData.m_dHDOP = 0.0;
    m_sentenceData.m_dLatitude = 0.0;
    m_sentenceData.m_dLongitude = 0.0;
    m_sentenceData.m_dVertSpeed = 0.0;
    m_sentenceData.m_nDifferentialID = 0;
    m_sentenceData.m_nGPSQuality = NMEAParserData::GQ_FIX_NOT_AVAILABLE;
    m_sentenceData.m_nHour = 0;
    m_sentenceData.m_nMinute = 0;
    m_sentenceData.m_nSatsInView = 0;
    m_sentenceData.m_nSecond = 0;
}
