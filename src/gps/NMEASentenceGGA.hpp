/******************************************************************************
 * File:             NMEASentenceGGA.hpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:07 AM
 * Description:      GGA Data class.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_NMEASENTENCEGGA_HPP
#define INU_DISPLAY_NMEASENTENCEGGA_HPP

#pragma once

#include <string>
#include "NMEAParserData.hpp"
#include "NMEASentenceBase.hpp"

class NMEASentenceGGA : public NMEASentenceBase {
public:
    NMEASentenceGGA();

    virtual ~NMEASentenceGGA();

    NMEAParserData::ERROR_E ProcessSentence(char *pCmd, char *pData) override;

    void ResetData() override;

    NMEAParserData::GGA_DATA_T GetSentenceData() { return m_sentenceData; }

private:
    NMEAParserData::GGA_DATA_T m_sentenceData{};
    int m_nOldVSpeedSeconds;
    double m_dOldVSpeedAlt;

};

#endif //INU_DISPLAY_NMEASENTENCEGGA_HPP
