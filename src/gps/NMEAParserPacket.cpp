/******************************************************************************
 * File:             NMEAParserPacket.cpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:12 AM
 *
 ******************************************************************************/

#include "NMEAParser.hpp"

NMEAParserPacket::NMEAParserPacket() :
        m_nState(PARSE_STATE_SOM),
        m_u8Checksum(0),
        m_u8ReceivedChecksum(0),
        m_nIndex(0) {
    Reset();
}

NMEAParserPacket::~NMEAParserPacket() = default;

NMEAParserData::ERROR_E NMEAParserPacket::ProcessNMEABuffer(const char *pData, size_t nBufferSize) {
    for (size_t i = 0; i < nBufferSize; i++) {
        char cData = pData[i];
        switch (m_nState) {
            case PARSE_STATE_SOM:
                if (cData == '$') {
                    TimeTag();

                    m_u8Checksum = 0;
                    m_nIndex = 0;
                    m_nState = PARSE_STATE_CMD;
                }
                break;

            case PARSE_STATE_CMD:
                if (cData != ',' && cData != '*') {
                    m_pCommand[m_nIndex++] = cData;
                    m_u8Checksum ^= cData;

                    if (m_nIndex >= NMEAParserData::c_uMaxCmdLen) {
                        OnError(NMEAParserData::ERROR_CMD_BUFFER_OVERFLOW, m_pCommand);
                        m_nState = PARSE_STATE_SOM;
                    }
                } else {
                    m_pCommand[m_nIndex] = '\0';
                    if (m_nIndex != 5) {
                        OnError(NMEAParserData::ERROR_FAIL, m_pCommand);
                        m_nState = PARSE_STATE_SOM;
                        break;
                    }
                    m_pTalker[0] = m_pCommand[0];
                    m_pTalker[1] = m_pCommand[1];
                    m_pTalker[2] = '\0';
                    m_pSentenceType[0] = m_pCommand[2];
                    m_pSentenceType[1] = m_pCommand[3];
                    m_pSentenceType[2] = m_pCommand[4];
                    m_pSentenceType[3] = '\0';
                    m_u8Checksum ^= cData;
                    m_nIndex = 0;
                    m_nState = PARSE_STATE_DATA;
                }
                break;

            case PARSE_STATE_DATA:
                if (cData == '*') {
                    m_pData[m_nIndex] = '\0';
                    m_nState = PARSE_STATE_CHECKSUM_1;
                } else {
                    if (cData == '\r') {
                        if (!m_allowedTalkers.contains(std::string(m_pTalker))) {
                            Reset();
                            return NMEAParserData::ERROR_UNTRUSTED_TALKER;
                        }
                        m_pData[m_nIndex] = '\0';
                        ProcessRxCommand(m_pSentenceType, m_pData);
                        m_nState = PARSE_STATE_SOM;
                        return NMEAParserData::ERROR_OK;
                    }

                    m_u8Checksum ^= cData;
                    m_pData[m_nIndex] = cData;
                    if (++m_nIndex >= NMEAParserData::c_uMaxDataLen) {
                        OnError(NMEAParserData::ERROR_RX_BUFFER_OVERFLOW, m_pCommand);
                        m_nState = PARSE_STATE_SOM;
                    }
                }
                break;

            case PARSE_STATE_CHECKSUM_1:
                if ((cData - '0') <= 9) {
                    m_u8ReceivedChecksum = (cData - '0') << 4;
                } else {
                    m_u8ReceivedChecksum = (cData - 'A' + 10) << 4;
                }

                m_nState = PARSE_STATE_CHECKSUM_2;

                break;

            case PARSE_STATE_CHECKSUM_2:
                if ((cData - '0') <= 9) {
                    m_u8ReceivedChecksum |= (cData - '0');
                } else {
                    m_u8ReceivedChecksum |= (cData - 'A' + 10);
                }

                if (m_u8Checksum == m_u8ReceivedChecksum) {
                    if (m_allowedTalkers.contains(std::string(m_pTalker))) {
                        ProcessRxCommand(m_pSentenceType, m_pData);
                    } else {
                        Reset();
                        return NMEAParserData::ERROR_UNTRUSTED_TALKER;
                    }
                } else {
                    OnError(NMEAParserData::ERROR_CHECKSUM, m_pCommand);
                }

                m_nState = PARSE_STATE_SOM;
                break;

            default:
                m_nState = PARSE_STATE_SOM;
        }
    }

    return NMEAParserData::ERROR_OK;
}

void NMEAParserPacket::Reset() {
    m_nState = PARSE_STATE_SOM;
    m_u8Checksum = m_u8ReceivedChecksum = 0;
    m_nIndex = 0;
}
