/******************************************************************************
 * File:             NMEAParserData.hpp
 * 
 * Author:           Idler Sgt Maxwell J.
 * Organization:     Marine Corps Software Factory
 * Created On:       1/1/2025 9:06 AM
 * Description:      This class will parse NMEA data, store its data and report that it has received data.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_NMEAPARSERDATA_HPP
#define INU_DISPLAY_NMEAPARSERDATA_HPP

#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>

#define UNUSED_PARAM(x) (void)(x)

namespace NMEAParserData {

    enum ERROR_E {
        ERROR_OK = 0,
        ERROR_FAIL = 1,
        ERROR_TOO_MANY_SATELLITES = 2,
        ERROR_CHECKSUM = 3,
        ERROR_RX_BUFFER_OVERFLOW,
        ERROR_CMD_BUFFER_OVERFLOW,
        ERROR_UNTRUSTED_TALKER
    };

    static const uint32_t c_uMaxCmdLen = 32;
    static const uint32_t c_uMaxDataLen = 256;
    static const int c_nMaxConstellation = 64;
    static const int c_nMaxGSASats = 12;
    static const int c_nInvlidPRN = 0;

    enum TALKER_ID_E {
        TID_AB = (uint16_t) 'A' << 8 | (uint16_t) 'B',
        TID_AD = (uint16_t) 'A' << 8 | (uint16_t) 'D',
        TID_AG = (uint16_t) 'A' << 8 | (uint16_t) 'G',
        TID_AP = (uint16_t) 'A' << 8 | (uint16_t) 'P',
        TID_BD = (uint16_t) 'B' << 8 | (uint16_t) 'D',
        TID_BN =
        (uint16_t) 'B' << 8 | (uint16_t) 'N',
        TID_CC = (uint16_t) 'C' << 8 |
                 (uint16_t) 'C',
        TID_CD = (uint16_t) 'C' << 8 |
                 (uint16_t) 'D',
        TID_CM =
        (uint16_t) 'C' << 8 | (uint16_t) 'M',
        TID_CS = (uint16_t) 'C' << 8 | (uint16_t) 'S',
        TID_CT = (uint16_t) 'C' << 8 |
                 (uint16_t) 'T',
        TID_CV =
        (uint16_t) 'C' << 8 | (uint16_t) 'V',
        TID_CX =
        (uint16_t) 'C' << 8 | (uint16_t) 'X',
        TID_DE = (uint16_t) 'D' << 8 | (uint16_t) 'E',
        TID_DF = (uint16_t) 'D' << 8 | (uint16_t) 'F',
        TID_DU = (uint16_t) 'D' << 8 | (uint16_t) 'U',
        TID_EC = (uint16_t) 'E' << 8 |
                 (uint16_t) 'C',
        TID_EP = (uint16_t) 'E' << 8 |
                 (uint16_t) 'P',
        TID_ER = (uint16_t) 'E' << 8 | (uint16_t) 'R',
        TID_GA = (uint16_t) 'G' << 8 | (uint16_t) 'A',
        TID_GB = (uint16_t) 'G' << 8 | (uint16_t) 'B',
        TID_GL =
        (uint16_t) 'G' << 8 | (uint16_t) 'L',
        TID_GN = (uint16_t) 'G' << 8 |
                 (uint16_t) 'N',
        TID_GP = (uint16_t) 'G' << 8 | (uint16_t) 'P',
        TID_HC = (uint16_t) 'H' << 8 | (uint16_t) 'C',
        TID_HE = (uint16_t) 'H' << 8 | (uint16_t) 'E',
        TID_HN =
        (uint16_t) 'H' << 8 | (uint16_t) 'N',
        TID_II = (uint16_t) 'I' << 8 | (uint16_t) 'I',
        TID_IN = (uint16_t) 'I' << 8 | (uint16_t) 'N',
        TID_LA = (uint16_t) 'L' << 8 | (uint16_t) 'A',
        TID_LC = (uint16_t) 'L' << 8 | (uint16_t) 'C',
        TID_MP =
        (uint16_t) 'M' << 8 | (uint16_t) 'P',
        TID_NL = (uint16_t) 'N' << 8 | (uint16_t) 'L',
        TID_OM =
        (uint16_t) 'O' << 8 | (uint16_t) 'M',
        TID_OS =
        (uint16_t) 'O' << 8 | (uint16_t) 'S',
        TID_QZ = (uint16_t) 'Q' << 8 |
                 (uint16_t) 'Z',
        TID_RA = (uint16_t) 'R' << 8 | (uint16_t) 'A',
        TID_SD = (uint16_t) 'S' << 8 | (uint16_t) 'D',
        TID_SN = (uint16_t) 'S' << 8 |
                 (uint16_t) 'N',
        TID_SS = (uint16_t) 'S' << 8 | (uint16_t) 'S',
        TID_TI = (uint16_t) 'T' << 8 | (uint16_t) 'I',
        TID_TR = (uint16_t) 'T' << 8 | (uint16_t) 'R',
        TID_U0 =
        (uint16_t) 'U' << 8 | (uint16_t) '0',
        TID_U1 =
        (uint16_t) 'U' << 8 | (uint16_t) '1',
        TID_U2 =
        (uint16_t) 'U' << 8 | (uint16_t) '2',
        TID_U3 =
        (uint16_t) 'U' << 8 | (uint16_t) '3',
        TID_U4 =
        (uint16_t) 'U' << 8 | (uint16_t) '4',
        TID_U5 =
        (uint16_t) 'U' << 8 | (uint16_t) '5',
        TID_U6 =
        (uint16_t) 'U' << 8 | (uint16_t) '6',
        TID_U7 =
        (uint16_t) 'U' << 8 | (uint16_t) '7',
        TID_U8 =
        (uint16_t) 'U' << 8 | (uint16_t) '8',
        TID_U9 =
        (uint16_t) 'U' << 8 | (uint16_t) '9',
        TID_UP = (uint16_t) 'U' << 8 | (uint16_t) 'P',
        TID_VD = (uint16_t) 'V' << 8 |
                 (uint16_t) 'D',
        TID_DM = (uint16_t) 'D' << 8 |
                 (uint16_t) 'M',
        TID_VW = (uint16_t) 'V' << 8 |
                 (uint16_t) 'W',
        TID_WI = (uint16_t) 'W' << 8 | (uint16_t) 'I',
        TID_YC =
        (uint16_t) 'Y' << 8 | (uint16_t) 'C',
        TID_YD = (uint16_t) 'Y' << 8 |
                 (uint16_t) 'D',
        TID_YF =
        (uint16_t) 'Y' << 8 | (uint16_t) 'F',
        TID_YL = (uint16_t) 'Y' << 8 | (uint16_t) 'L',
        TID_YP =
        (uint16_t) 'Y' << 8 | (uint16_t) 'P',
        TID_YR =
        (uint16_t) 'Y' << 8 | (uint16_t) 'R',
        TID_YT =
        (uint16_t) 'Y' << 8 | (uint16_t) 'T',
        TID_YV = (uint16_t) 'Y' << 8 | (uint16_t) 'V',
        TID_YX = (uint16_t) 'Y' << 8 | (uint16_t) 'X',
        TID_ZA = (uint16_t) 'Z' << 8 | (uint16_t) 'A',
        TID_ZC = (uint16_t) 'Z' << 8 | (uint16_t) 'C',
        TID_ZQ = (uint16_t) 'Z' << 8 | (uint16_t) 'Q',
        TID_ZV =
        (uint16_t) 'Z' << 8 | (uint16_t) 'V',
    };

    enum GPS_QUALITY_E {
        GQ_FIX_NOT_AVAILABLE = 0,
        GQ_GPS_SPS_MODE = 1,
        GQ_GPS_DIFFERENTIAL_SPS_MODE = 2,
        GQ_GPS_PPS_MODE = 3,
        GQ_REAL_TIME_KINEMATIC = 4,
        GQ_FLOAT_RTK = 5,
        GQ_ESTIMATED_DEAD_RECONING = 6,
        GQ_MANUAL_INPUT_MODE = 7,
        GQ_SIMULATOR_MODE = 8,
    };

    typedef struct SAT_INFO_T {
        double dAzimuth;
        double dElevation;
        int nPRN;
        int nSNR;
    } SAT_INFO_T;

    enum ACTIVE_SAT_AUTO_MODE_E {
        ASAM_MANUAL = 'M',
        ASAM_AUTO = 'A',
    };

    enum ACTIVE_SAT_MODE_E {
        ASM_FIX_NOT_AVAILABLE = 1,
        ASM_2D = 2,
        ASM_3D = 3,
    };

    typedef struct GGA_DATA_T {
        int m_nHour;
        int m_nMinute;
        int m_nSecond;
        double m_dLatitude;
        double m_dLongitude;
        double m_dAltitudeMSL;
        GPS_QUALITY_E m_nGPSQuality;
        int m_nSatsInView;
        double m_dHDOP;
        double m_dGeoidalSep;
        double m_dDifferentialAge;
        int m_nDifferentialID;
        double m_dVertSpeed;
    } GGA_DATA_T;

    typedef struct ZDA_DATA_T
    {
        int m_nHour;
        int m_nMinute;
        int m_nSecond;
        int m_nDay;
        int m_nMonth;
        int m_nYear;
        int m_nLocalTimeZoneHourOffsetFromUTC;
        int m_nLocalTimeZoneMinuteOffsetFromUTC;
    } ZDA_DATA_T;

    typedef struct GSV_DATA_T {
        int nTotalNumberOfSentences;
        int nSentenceNumber;
        int nSatsInView;
        NMEAParserData::SAT_INFO_T SatInfo[c_nMaxConstellation];
    } GSV_DATA_T;

    typedef struct GSA_DATA_T {
        NMEAParserData::ACTIVE_SAT_AUTO_MODE_E nAutoMode;
        NMEAParserData::ACTIVE_SAT_MODE_E nMode;
        int pnPRN[NMEAParserData::c_nMaxConstellation];
        double dPDOP;
        double dHDOP;
        double dVDOP;
        unsigned int uGGACount;
    } GSA_DATA_T;

    enum RMC_STATUS_E {
        RMC_STATUS_ACTIVE = 'A',
        RMC_STATUS_VOID = 'V',
    };

    typedef struct RMC_DATA_T {
        time_t m_timeGGA;
        int m_nHour;
        int m_nMinute;
        int m_nSecond;
        double m_dSecond;
        double m_dLatitude;
        double m_dLongitude;
        double m_dAltitudeMSL;
        RMC_STATUS_E m_nStatus;
        double m_dSpeedKnots;
        double m_dTrackAngle;
        int m_nMonth;
        int m_nDay;
        int m_nYear;
        double m_dMagneticVariation;

    } RMC_DATA_T;

    enum HDG_DIRECTION_T {
        HDG_LONGITUDE_DIRECTION_EAST = 'E',
        HDG_LONGITUDE_DIRECTION_WEST = 'W',
    };

    typedef struct HDG_DATA_T {
        double m_heading;
        int m_magneticDeviation;
        double m_magneticDeviationDegrees;
        HDG_DIRECTION_T m_magneticDeviationDirection;
        double m_magneticVariationDegrees;
        HDG_DIRECTION_T m_magneticVariationDirection;
    } HDG_DATA_T;

    enum HDT_DIRECTION_T {
        TRUE_NORTH = 'T',
    };

    typedef struct HDT_DATA_T {
        double m_heading;
        HDT_DIRECTION_T m_direction;
    } HDT_DATA_T;


};

#endif //INU_DISPLAY_NMEAPARSERDATA_HPP
