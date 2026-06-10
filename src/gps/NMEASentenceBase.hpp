/******************************************************************************
 * File:             NMEASentenceBase.hpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:06 AM
 * Description:      This is the base class for every NMEA sentence.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_NMEASENTENCEBASE_HPP
#define INU_DISPLAY_NMEASENTENCEBASE_HPP

#pragma once

#include <string>
#include "NMEAParserData.hpp"

class NMEASentenceBase {
public:
    NMEASentenceBase();

    ~NMEASentenceBase();

    virtual NMEAParserData::ERROR_E ProcessSentence(char *pCmd, char *pData) = 0;

    virtual void ResetData() = 0;

    [[nodiscard]] unsigned int GetRxCount() const { return m_uRxCount; }

public:
    static const int c_nMaxField = 256;

protected:
    static NMEAParserData::ERROR_E GetField(const char *pData, char *pField, int nFieldNum, int nMaxFieldLen);

protected:
    unsigned int m_uRxCount;

private:
    std::string m_strSentenceID;
    NMEAParserData::TALKER_ID_E m_nTalkerID;

};

#endif //INU_DISPLAY_NMEASENTENCEBASE_HPP
