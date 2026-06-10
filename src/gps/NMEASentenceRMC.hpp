/******************************************************************************
 * File:             NMEASentenceRMC.hpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:07 AM
 * Description:      RMC Data class.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_NMEASENTENCERMC_HPP
#define INU_DISPLAY_NMEASENTENCERMC_HPP

#include "NMEASentenceBase.hpp"
#include "NMEAParserData.hpp"

class NMEASentenceRMC : public NMEASentenceBase {
public:
    NMEASentenceRMC();

    virtual ~NMEASentenceRMC();

    NMEAParserData::ERROR_E ProcessSentence(char *pCmd, char *pData) override;

    void ResetData() override;

    NMEAParserData::RMC_DATA_T GetSentenceData() { return m_sentenceData; }

private:
    NMEAParserData::RMC_DATA_T m_sentenceData{};

};

#endif //INU_DISPLAY_NMEASENTENCERMC_HPP
