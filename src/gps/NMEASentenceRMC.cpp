/******************************************************************************
 * File:             NMEASentenceRMC.cpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:17 AM
 *
 ******************************************************************************/

#include <cstdlib>
#include "NMEASentenceRMC.hpp"

NMEASentenceRMC::NMEASentenceRMC() = default;

NMEASentenceRMC::~NMEASentenceRMC() = default;

NMEAParserData::ERROR_E NMEASentenceRMC::ProcessSentence(char *pCmd, char *pData) {

    UNUSED_PARAM(pCmd);
    char szField[c_nMaxField];

    // Time
    if (GetField(pData, szField, 0, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_nHour = (szField[0] - '0') * 10 + (szField[1] - '0');
        m_sentenceData.m_nMinute = (szField[2] - '0') * 10 + (szField[3] - '0');
        m_sentenceData.m_nSecond = (szField[4] - '0') * 10 + (szField[5] - '0');
    }

    // Status
    if (GetField(pData, szField, 1, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_nStatus = (NMEAParserData::RMC_STATUS_E) ((char) szField[0]);
    } else {
        m_sentenceData.m_nStatus = (NMEAParserData::RMC_STATUS_VOID);
    }

    //
    // Latitude
    //
    if (GetField(pData, szField, 2, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dLatitude = atof((char *) szField + 2) / 60.0;
        szField[2] = '\0';
        m_sentenceData.m_dLatitude += atof((char *) szField);

    }
    if (GetField(pData, szField, 3, c_nMaxField) == NMEAParserData::ERROR_OK) {
        if (szField[0] == 'S') {
            m_sentenceData.m_dLatitude = -m_sentenceData.m_dLatitude;
        }
    }

    //
    // Longitude
    //
    if (GetField(pData, szField, 4, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dLongitude = atof((char *) szField + 3) / 60.0;
        szField[3] = '\0';
        m_sentenceData.m_dLongitude += atof((char *) szField);
    }
    if (GetField(pData, szField, 5, c_nMaxField) == NMEAParserData::ERROR_OK) {
        if (szField[0] == 'W') {
            m_sentenceData.m_dLongitude = -m_sentenceData.m_dLongitude;
        }
    }

    // Speed over ground knots
    if (GetField(pData, szField, 6, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dSpeedKnots = atol(szField);
    } else {
        m_sentenceData.m_dSpeedKnots = 0.0;
    }

    // Track Angle
    if (GetField(pData, szField, 7, c_nMaxField) == NMEAParserData::ERROR_OK) {
        m_sentenceData.m_dTrackAngle = atof(szField);
    } else {
        m_sentenceData.m_dTrackAngle = 0.0;
    }


    // Date
    if (GetField(pData, szField, 8, c_nMaxField) == NMEAParserData::ERROR_OK) {
        // 23 03 94       Date - 23rd of March 1994
        m_sentenceData.m_nDay = (szField[0] - '0') * 10 + (szField[1] - '0');
        m_sentenceData.m_nMonth = (szField[2] - '0') * 10 + (szField[3] - '0');
        m_sentenceData.m_nYear = (szField[4] - '0') * 10 + (szField[5] - '0');
        m_sentenceData.m_nYear += 2000;
    } else {
        m_sentenceData.m_nMonth = 0;
        m_sentenceData.m_nDay = 0;
        m_sentenceData.m_nYear = 0;
    }


    // Magnetic Variation
    if (GetField(pData, szField, 9, c_nMaxField) == NMEAParserData::ERROR_OK) {

        m_sentenceData.m_dMagneticVariation = atof(szField);

        if (GetField(pData, szField, 10, c_nMaxField) == NMEAParserData::ERROR_OK) {
            if (szField[0] == 'W') {
                m_sentenceData.m_dMagneticVariation *= -1.0;
            }
        }
    } else {
        m_sentenceData.m_dMagneticVariation = 0.0;
    }


    m_uRxCount++;

    return NMEAParserData::ERROR_OK;
}

void NMEASentenceRMC::ResetData(void) {
    m_uRxCount = 0;
    m_sentenceData.m_dAltitudeMSL = 0.0;
    m_sentenceData.m_dLatitude = 0.0;
    m_sentenceData.m_dLongitude = 0.0;
    m_sentenceData.m_dMagneticVariation = 0.0;
    m_sentenceData.m_dSecond = 0;
    m_sentenceData.m_dSpeedKnots = 0.0;
    m_sentenceData.m_dTrackAngle = 0.0;
    m_sentenceData.m_nDay = 0;
    m_sentenceData.m_nHour = 0;
    m_sentenceData.m_nMinute = 0;
    m_sentenceData.m_nMonth = 0;
    m_sentenceData.m_nSecond = 0;
    m_sentenceData.m_nStatus = NMEAParserData::RMC_STATUS_VOID;
    m_sentenceData.m_nYear = 0;
}
