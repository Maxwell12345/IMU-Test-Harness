/******************************************************************************
 * File:             NMEASentenceBase.cpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:13 AM
 *
 ******************************************************************************/

#include "NMEASentenceBase.hpp"

NMEASentenceBase::NMEASentenceBase() :
        m_uRxCount(0) {
}


NMEASentenceBase::~NMEASentenceBase() {
}

NMEAParserData::ERROR_E NMEASentenceBase::GetField(const char *pData, char *pField, int nFieldNum, int nMaxFieldLen) {
    if (pData == nullptr || pField == nullptr || nMaxFieldLen <= 0) {
        return NMEAParserData::ERROR_FAIL;
    }

    int i = 0;
    int nField = 0;
    while (nField != nFieldNum && pData[i]) {
        if (pData[i] == ',') {
            nField++;
        }

        i++;

        if (pData[i] == 0) {
            pField[0] = '\0';
            return NMEAParserData::ERROR_FAIL;
        }
    }

    if (pData[i] == ',' || pData[i] == '*') {
        pField[0] = '\0';
        return NMEAParserData::ERROR_FAIL;
    }

    int i2 = 0;
    while (pData[i] != ',' && pData[i] != '*' && pData[i]) {
        pField[i2] = pData[i];
        i2++;
        i++;

        if (i2 >= nMaxFieldLen) {
            i2 = nMaxFieldLen - 1;
            break;
        }
    }
    pField[i2] = '\0';

    return NMEAParserData::ERROR_OK;
}
