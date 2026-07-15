/******************************************************************************
 * Filename:     utils.hpp
 *
 * Author:       Tran Sgt Brandon
 * Organization: Marine Corps Software Factory
 * Created On:   5/21/2026 1:38 PM
 * Description:  This header defines the namespace for IMU Coordinate Transformation.
 *
 ******************************************************************************/

#ifndef IMU_UTILS_HPP
#define IMU_UTILS_HPP

#pragma once

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <numbers>
#include <array>

namespace IMUUtils
{
    inline double RAD_PER_DEGREE = std::numbers::pi / 180.0;
    inline double DEGREE_PER_RAD = 180.0 / std::numbers::pi;

    struct GpsUpdate {
        std::chrono::steady_clock::time_point rxTimestamp;
        std::chrono::system_clock::time_point gpsTimestamp;
        double latitude;
        double longitude;

        GpsUpdate():
            rxTimestamp(std::chrono::steady_clock::now()) {}

        GpsUpdate(double latitude, double longitude, std::chrono::system_clock::time_point gpsTimestamp):
            rxTimestamp(std::chrono::steady_clock::now()),
            gpsTimestamp(gpsTimestamp),
            latitude(latitude),
            longitude(longitude) {}
    };

    struct KineticState {
        std::chrono::steady_clock::time_point timestamp;
        double speedEastWest{};
        double speedNorthSouth{};
        double accelerationEastWest{};
        double accelerationNorthSouth{};

        KineticState():
            timestamp(std::chrono::steady_clock::now()) {}

        KineticState(double v_x, double v_y, double a_x, double a_y):
            timestamp(std::chrono::steady_clock::now()),
            speedEastWest(v_x),
            speedNorthSouth(v_y),
            accelerationEastWest(a_x),
            accelerationNorthSouth(a_y) {}

        KineticState(std::chrono::steady_clock::time_point tp, double v_x, double v_y, double a_x, double a_y):
            timestamp(tp),
            speedEastWest(v_x),
            speedNorthSouth(v_y),
            accelerationEastWest(a_x),
            accelerationNorthSouth(a_y) {}

        KineticState& operator=(const KineticState& other) {
            timestamp = other.timestamp;
            speedEastWest = other.speedEastWest;
            speedNorthSouth = other.speedNorthSouth;
            accelerationEastWest = other.accelerationEastWest;
            accelerationNorthSouth = other.accelerationNorthSouth;

            return *this;
        }
    };

    /**
     * @brief Converts Degrees to Radians from IMU readings.
     *
     * @remarks
     * Expected behavior:
     *  - Ensures the heading is normalized to [0.0 -> 360.0] degrees
     *  - Returns heading in radians
     *
     * Assumptions:
     *  -Input is a valid heading in degrees (not NaN)
     *
     *  @param  degrees Heading angle in degrees
     * 
     *  @return         Normalized heading in radians from [0.0 -> 2 * pi]
     */
    inline double DegreesToRadians(double degrees) {
        if(degrees < 0 || degrees >= 360) {
        degrees = fmod(degrees, 360);
        degrees = (degrees < 0)? degrees + 360 : degrees;
        }

        return degrees * std::numbers::pi / 180.0;
    };

    /**
     *
     * @brief Converts to Global X Acceleration from IMU (x,y) readings.
     *
     * @remarks
     * Expected behavior:
     * - Returns linear acceleration in global X (East-positive) in meters / second / second.
     * - Uses same rotation formula as defined in global_rotation_tests.cpp
     * - boat_x and boat_y are accelerations in boat frame (right/forward)
     *
     * Assumptions:
     * - Assumes theta_t is measured with respect to True North.
     *
     * @param   theta_t The heading of the boat (which direction the boat's
     *                  y-axis is pointing) in radians measured from True North.
     * @param   boat_x  The boat's linear acceleration in the x-axis in meters per second (m/s).
     * @param   boat_y  The boat's linear acceleration in the y-axis in meters per second (m/s).
     * @return          Linear acceleration in global x axis (East/West with East being positive) measured in meters per sec
     * per second.
     */
    inline double InertialToGlobal_X(double theta_t, double boat_x, double boat_y) {
        double global_X = std::cos(theta_t) * boat_x + std::sin(theta_t) * boat_y;
        return global_X;
    };

    /**
     *  @brief Converts to Global Y Acceleration from IMU (x,y) readings.
     *
     * @remark
     * Expected behavior:
     * - Returns linear acceleration in global Y (North-positive) in meters / second / second.
     * - Uses the same rotation formula as defined in global_rotation_tests.cpp
     *
     * @remark
     * Assumptions:
     * - theta_t is heading in radians measured from True North
     * - boat_x and boat_y are accelerations in boat frame (right/forward)
     *
     * @param   theta_t Boat heading in radians from True North.
     * @param   boat_x  The boat's linear acceleration in the x-axis in meters per second (m/s).
     * @param   boat_y  The boat's linear acceleration in the y-axis in meters per second(m/s).
     * @return          Linear acceleration in global y axis (North/South with North being positive) measured in meters per
     * sec per second.
     *
     */
    inline double InertialToGlobal_Y(double theta_t, double boat_x, double boat_y) {
        double global_Y = std::cos(theta_t) * boat_y - std::sin(theta_t) * boat_x;
        return global_Y;
    };

    /**
     *
     * @brief   Calculates the current state of the vessel on a global coordinate frame using degrees latitude/longitude as
     * the metric for movenemt. Utilizes basic Newtonia motion mechanics and assumes a constant acceleration over the elapsed
     * period between measurements to project where an object is given a rate of acceleration, previous velocity, and
     * time elapsed.
     *
     * @param [in] previous     A const reference to the previously calculated kinetic state of the system in global reference frame
     * @param [in] accelerationEastWest     The east/west acceleration of the system as measured by the IMU, converted to global
     * frame (measured in degrees longitude per second per second) with east being positive
     * @param [in] accelerationNorthSouth     The north/south acceleration of the system as measured by the IMU, converted to global
     * frame (measured in degrees latitude per second per second) with north being positive
     * @param [in] currentTimestamp     The steady clock timestamp to assign to the calculated state. Allows tests and
     * production callers to control the exact elapsed time used in the velocity update.
     *
     * @return  The calculated position of the vessel
     */
    inline KineticState CalculateKineticUpdate(const IMUUtils::KineticState& previous, double accelerationEastWest,
                                            double accelerationNorthSouth, std::chrono::steady_clock::time_point currentTimestamp) {
        KineticState current = KineticState(currentTimestamp, 0.0, 0.0, 0.0, 0.0);
        const double deltaT = std::chrono::duration<double>(current.timestamp - previous.timestamp).count();

        current.accelerationEastWest = accelerationEastWest;
        current.speedEastWest = previous.speedEastWest + accelerationEastWest * deltaT;

        current.accelerationNorthSouth = accelerationNorthSouth;
        current.speedNorthSouth= previous.speedNorthSouth + accelerationNorthSouth * deltaT;

        return current;
    }

    /**
     *
     * @brief   Calculates the current kinetic state using the steady clock timestamp captured at the time of the call.
     *
     * @param [in] previous     A const reference to the previously calculated kinetic state of the system in global reference frame
     * @param [in] accelerationEastWest     The east/west acceleration of the system as measured by the IMU, converted to global
     * frame (measured in degrees longitude per second per second) with east being positive
     * @param [in] accelerationNorthSouth     The north/south acceleration of the system as measured by the IMU, converted to global
     * frame (measured in degrees latitude per second per second) with north being positive
     *
     * @return  The calculated position of the vessel
     */
    inline KineticState CalculateKineticUpdate(const IMUUtils::KineticState& previous, double accelerationEastWest,
                                            double accelerationNorthSouth) {
        return CalculateKineticUpdate(previous, accelerationEastWest, accelerationNorthSouth,
                                        std::chrono::steady_clock::now());
    }

    /**
     * @brief Converts magnetic heading and declination angle values into a true north angle.
     *        True north angle is a degree value from 0 to 360.
     * 
     * @param [in] magneticHeading Magnetic heading in geodidic WGS84 from [0, 360).
     * @param [in] declinationAngle NOAA calculated declination angle in geodedic WGS84 [-180, 180).
     * 
     * @return True north heading in geodedic WGS84 from [0, 360).
     * 
     * @remarks
     * 
     * @throws std::runtime_error if inputs are out of bounds.
     * 
     */
    inline double MagneticToTrueHeading(double magneticHeading, double declinationAngle) {
        if (magneticHeading < 0.0 || magneticHeading >= 360.0 || declinationAngle < -180.0 || declinationAngle >= 180.0) {
            throw std::runtime_error("Magnetic heading or declination angle is invalid. Magnetic heading bounds are [0.0, 360.0) and declination angle bounds are [-180.0, 180.0). Got, magneticHeading: " + std::to_string(magneticHeading) + " and declinationAngle: " + std::to_string(declinationAngle));
        }

        double trueNorthHeading = magneticHeading + declinationAngle;

        while (trueNorthHeading < 0.0) {
            trueNorthHeading += 360.0;
        }

        while (trueNorthHeading >= 360.0) {
            trueNorthHeading -= 360.0;
        }

        return trueNorthHeading;
    }

    /**
     * @brief Converts quaternions w + (i,j,k) to calculate magnetic heading.
     *
     * @remark
     * Expected Behavior:
     * The returned heading will always be between [0.0, 360).
     *
     * @remark
     * Assumptions:
     * - w, i, j and k values are normalized.
     *
     * @param w Scalar
     * @param i Quaternion rotation with respect to x axis.
     * @param j Quaternion rotation with respect to y axis.
     * @param k Quaternion rotation with respect to z axis.
     * @return  A double containing the magnetic heading of the IMU in degrees (not radians).
     *
     */
    inline double Calculate_Magnetic_Heading(double w, double i, double j, double k) {
        const double n = std::sqrt(w*w + i*i + j*j + k*k);

        if (!std::isfinite(n) || n < 1e-12) {
            throw std::runtime_error("Invalid quaternion norm.");
        }

        w /= n;
        i /= n;
        j /= n;
        k /= n;

        // Android/BNO08x rotation-vector convention:
        // world X = east, world Y = magnetic north, world Z = up.
        // Heading/azimuth is angle from magnetic north to device +Y, clockwise positive.
        const double headingRad = std::atan2(
            2.0 * (i*j - k*w),
            1.0 - 2.0 * (i*i + k*k)
        );

        double degrees = headingRad * 180.0 / std::numbers::pi;

        degrees = std::fmod(degrees, 360.0);
        if (degrees < 0.0) {
            degrees += 360.0;
        }
        return degrees;
    };

    struct ENUAccel {
        double east;
        double north;
        double up;
    };

    inline IMUUtils::ENUAccel RotateLinearAccelToTrueENU(double w, double i, double j, double k, double ax, double ay, double az, double declinationDeg) {
        const double n = std::sqrt(w*w + i*i + j*j + k*k);

        if (!std::isfinite(n) || n < 1e-12) {
            throw std::runtime_error("Invalid quaternion norm.");
        }

        w /= n;
        i /= n;
        j /= n;
        k /= n;

        // BNO08x / Android-style rotation matrix:
        // device/body vector -> magnetic-world ENU vector.
        //
        // world X = east
        // world Y = magnetic north
        // world Z = up
        const double R00 = 1.0 - 2.0 * (j*j + k*k);
        const double R01 = 2.0 * (i*j - k*w);
        const double R02 = 2.0 * (i*k + j*w);

        const double R10 = 2.0 * (i*j + k*w);
        const double R11 = 1.0 - 2.0 * (i*i + k*k);
        const double R12 = 2.0 * (j*k - i*w);

        const double R20 = 2.0 * (i*k - j*w);
        const double R21 = 2.0 * (j*k + i*w);
        const double R22 = 1.0 - 2.0 * (i*i + j*j);

        const double magEast  = R00*ax + R01*ay + R02*az;
        const double magNorth = R10*ax + R11*ay + R12*az;
        const double up       = R20*ax + R21*ay + R22*az;

        // NOAA convention: declination is positive east of true north.
        // true heading = magnetic heading + declination.
        const double d = declinationDeg * std::numbers::pi / 180.0;

        const double trueEast = std::cos(d) * magEast + std::sin(d) * magNorth;
        const double trueNorth = -std::sin(d) * magEast + std::cos(d) * magNorth;

        return {trueEast, trueNorth, up};
    }

    /**
     * @brief Converts lat/lon into mercator projected units in meters.
     *
     * @param lat WGS84 latitude in degrees.
     * @param lon WGS84 longitude in degrees.
     *
     * @return {E, N, zone, N0} mercator project in meters. The E component is the longitudinal 
     *                      axis, the N component is the latitudinal axis, the zone is the
     *                      UTM zone, and N0 is the northern hemosphere coeff required for inverse 
     *                      calculations.
     */
    inline std::array<double, 4> WGS84_to_UTM(double lat, double lon) {
        // paramters
        constexpr double piOver180 = std::numbers::pi_v<double> / 180.0;
        constexpr double a = 6378137.0;
        constexpr double k0 = 0.9996;
        constexpr double eSq = 0.0066943799901413169961;
        constexpr double ePrimeSq = 0.006739496742276;
        constexpr double r = 6367449.1458234153093;
        constexpr double U0 = -32144.479935001393;
        constexpr double U2 = 135.366898504447;
        constexpr double U4 = -0.709321656818;
        constexpr double U6 = 0.003986100961;
        constexpr double E0 = 500000.0;

        // Zone calculation
        int zone = std::clamp(static_cast<int>(std::floor((lon + 180.0) / 6.0)) + 1, 1, 60);

        if (lat >= 56.0 && lat < 64.0 && lon >= 3.0 && lon < 12.0) {
            zone = 32;
        }

        if (lat >= 72.0 && lat < 84.0) {
            if (lon >= 0.0 && lon < 9.0) zone = 31;
            else if (lon >= 9.0 && lon < 21.0) zone = 33;
            else if (lon >= 21.0 && lon < 33.0) zone = 35;
            else if (lon >= 33.0 && lon < 42.0) zone = 37;
        }

        // calculated coefficients
        const double phi0 = 0.0;
        const double lambda0 = (6.0 * zone - 183.0) * piOver180;
        const double N0 = lat < 0.0 ? 10000000.0 : 0.0;

        const double phi = lat * piOver180;
        const double lambda = lon * piOver180;

        const double L = (lambda - lambda0) * std::cos(phi);
        const double LSq = L * L;
        const double nSq = ePrimeSq * std::pow(std::cos(phi), 2);
        const double t = std::tan(phi);
        const double tSq = t * t;
        const double tSqSq = tSq * tSq;
        const double sinPhi = std::sin(phi);
        const double cosPhi = std::cos(phi);
        const double cosPhiSq = cosPhi * cosPhi;
        const double A1 = k0 * a / std::sqrt(1.0 - eSq * std::pow(std::sin(phi),2));
        const double A2 = 0.5 * A1 * t;
        const double A3 = 0.166666666666666667 * ( 1 - tSq + nSq );
        const double A4 = 0.083333333333333333 * ( 5 - tSq + nSq * ( 9 + 4 * nSq ) );
        const double A5 = 0.008333333333333333 * ( 5 - 18 * tSq + tSqSq + nSq * ( 14 - 58 * tSq ) );
        const double A6 = 0.002777777777777778 * ( 61 - 58 * tSq + tSqSq + nSq * ( 270 - 330 * tSq ) );
        const double A7 = 0.000198412698413000 * ( 61 - 479 * tSq + 179 * tSqSq - ( tSqSq * tSq ) );
        const double omega = phi * r + sinPhi * cosPhi * ( U0 + U2 * cosPhiSq + U4 * ( cosPhiSq * cosPhiSq ) + U6 * ( cosPhiSq * cosPhiSq * cosPhiSq ) );
        const double S = k0 * omega;
        const double S0 = 0.0;

        const double E = E0 + A1 * L * ( 1.0 + LSq * (A3 + LSq * ( A5 + A7 * LSq ) ) );
        const double N = N0 + S - S0 + A2 * LSq * ( 1.0 + LSq * ( A4 + A6 * LSq ) );

        return {E, N, static_cast<double>(zone), N0};
    }

    /**
     * @brief Converts UTM x, y, and zone into WGS84 lat/lon.
     *
     * @param x UTM x.
     * @param y UTM y.
     * @param zone UTM zone.
     * @param N0 Northern Hemisphere coeff.
     *
     * @return {lat, lon} in WGS84 standard.
     */
    inline std::array<double, 2> UTM_to_WGS84(double E, double N, double zone, double N0) {
        // parameters
        constexpr double piOver180 = std::numbers::pi_v<double> / 180.0;
        constexpr double one80OverPi = 180.0 / std::numbers::pi_v<double>;
        constexpr double a = 6378137.0;
        constexpr double k0 = 0.9996;
        constexpr double r = 6367449.1458234153093;
        constexpr double eSq = 0.0066943799901413169961;
        constexpr double ePrimeSq = 0.006739496742276;
        constexpr double S0 = 0.0;

        const double V0 = ( ePrimeSq / 4 ) * ( ( ( ( 16384.0 * ePrimeSq - 11025.0 ) * ( ePrimeSq / 64.0 ) + 175.0 ) * ( ePrimeSq / 4.0 ) - 45 ) * ( ePrimeSq / 16 ) + 3 ); 
        const double V2 = ( ( ePrimeSq * ePrimeSq ) / 32 ) * ( ( ( ( -20464721.0 / 120.0 ) * ePrimeSq + 19413.0 ) * ( ePrimeSq / 8.0 ) - 1477.0 ) * ( ePrimeSq / 32.0 ) + 21.0 );
        const double V4 = ( ( ePrimeSq * ePrimeSq * ePrimeSq ) / 192.0 ) * ( ( ( 4737141.0 / 28.0 ) * ePrimeSq - 17121.0 ) * ( ePrimeSq / 32.0 ) + 151.0 );
        const double V6 = ( ( ePrimeSq * ePrimeSq * ePrimeSq * ePrimeSq ) / 1024.0 ) * ( ( -427277.0 / 35.0 ) * ePrimeSq + 1097.0 );

        const double omega = ( N - N0 + S0 ) / ( k0 * r );
        const double cosOmega = std::cos( omega );
        const double cosOmegaSq = cosOmega * cosOmega;
        const double cosOmegaSqSq = cosOmegaSq * cosOmegaSq;

        const double sinOmega = std::sin( omega );
        const double phiF = omega + ( sinOmega * cosOmega ) * ( V0 + V2 * cosOmegaSq + V4 * cosOmegaSqSq + V6 * cosOmegaSq * cosOmegaSqSq );
        const double sinPhiF = std::sin( phiF );
        const double cosPhiF = std::cos( phiF );
        
        const double Rf = ( k0 * a ) / std::sqrt( 1 - eSq * sinPhiF * sinPhiF );
        const double Q = ( E - 500000.0 ) / Rf;
        const double QSq = Q * Q;

        const double tf = std::tan( phiF );
        const double NfSq = ePrimeSq * cosPhiF * cosPhiF;

        const double tfSq = tf * tf;
        const double tfSqSq = tfSq * tfSq;

        const double B2 = -0.5 * tf * ( 1 + NfSq );
        const double B3 = -( 1.0 / 6.0 ) * ( 1 + 2 * tfSq + NfSq );
        const double B4 = -( 1.0 / 12.0 ) * ( 5.0 + 3.0 * tfSq + NfSq * ( 1.0 - 9.0 * tfSq ) - 4.0 * NfSq * NfSq );
        const double B5 = ( 1.0 / 120.0 ) * ( 5.0 + 28.0 * tfSq + 24.0 * tfSqSq + NfSq * ( 6.0 + 8.0 * tfSq ) );
        const double B6 = ( 1.0 / 360.0 ) * ( 61.0 + 90.0 * tfSq + 45.0 * tfSqSq + NfSq * ( 46.0 - 252.0 * tfSq - 90.0 * tfSqSq ) );
        const double B7 = -( 1.0 / 5040.0 ) * ( 61.0 + 662.0 * tfSq + 1320.0 * tfSqSq + 720.0 * tfSq * tfSqSq );

        const double phi = phiF + B2 * QSq * ( 1 + QSq * ( B4 + B6 * QSq ) );
        const double L = Q * ( 1 + QSq * ( B3 + QSq * ( B5 + B7 * QSq * Q ) ) );
        const double lambda = (6.0 * zone - 183.0) * piOver180 + L / cosPhiF;

        return {phi * one80OverPi, lambda * one80OverPi};
    }
}; // namespace IMUUtils

#endif // IMU_UTILS_HPP