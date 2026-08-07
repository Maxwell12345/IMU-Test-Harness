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
    constexpr double SEMI_MAJOR_AXIS_M = 6'378'137.0;
    constexpr double SEMI_MINOR_AXIS_M = 6'356'752.3;
    constexpr double ECCENTRICITY_SQUARED = 1.0 - ((SEMI_MINOR_AXIS_M * SEMI_MINOR_AXIS_M) / (SEMI_MAJOR_AXIS_M * SEMI_MAJOR_AXIS_M));
    constexpr double FLATTENING = 1 - (SEMI_MINOR_AXIS_M / SEMI_MAJOR_AXIS_M);
    constexpr double piOver180 = 0.017453292519943;
    constexpr double one80OverPi = 57.29577951308232;

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

    /*
     * @brief Calculates the vertical radius of the elipse
     *
     * @param [in] sinLat is the sin(lat) evaluated outside of the function
     * 
     * @return the vertical radius of the elipse in meters
    */
    inline double PrimeVerticalRadiusOfCurvature(double sinLat) {
        double denominator = std::sqrt(1 - ECCENTRICITY_SQUARED * sinLat * sinLat);
        return SEMI_MAJOR_AXIS_M / denominator;
    }

    /*
     * @brief Converts lat, lon, height in WGS84 to Earth Centered, Earth Fixed coordinate in meters
     *
     * @param [in] lambda   longitude in degrees
     * @param [in] phi      latitude in degrees
     * @param [in] height   height in meters
     * @param [out] X       X coordinate in ECEF in meters
     * @param [out] Y       Y coordinate in ECEF in meters
     * @param [out] Z       Z coordinate in ECEF in meters
    */
    inline void WGS84_To_ECEF(const double lambda, const double phi, const double height, double& X, double& Y, double& Z) {
        const double lonRad = lambda * piOver180;
        const double latRad = phi * piOver180;

        const double sinLat = std::sin(latRad);
        const double cosLat = std::cos(latRad);
        const double sinLon = std::sin(lonRad);
        const double cosLon = std::cos(lonRad);

        const double radius = PrimeVerticalRadiusOfCurvature(sinLat);

        X = ( radius + height ) * cosLat * cosLon;
        Y = ( radius + height ) * cosLat * sinLon;
        Z = ( (1.0 - ECCENTRICITY_SQUARED) * radius + height ) * sinLat;
    }

    /*
     * @brief Converts Geodetic WGS84 coordinates to ENU coordinates in meters
     *
     * @param [in] lambdaR  longitude of reference point in degrees
     * @param [in] phiR     latitude of reference point in degrees
     * @param [in] heightR  height of reference point in meters
     * @param [in] lambda   longitude of destination point in degrees
     * @param [in] phi      latitude of destination point in degrees
     * @param [in] height   height of destination point in meters
     * @param [out] E       E coordinate in ENU frame in meters
     * @param [out] N       N coordinate in ENU frame in meters
     * @param [out] U       U coordinate in ENU frame in meters
    */
    inline void WGS84_To_ENU(const double lambdaR, const double phiR, const double heightR, const double lambda, const double phi, const double height, double& E, double& N, double& U) {
        const double lonRefRad = lambdaR * piOver180;
        const double latRefRad = phiR * piOver180;
        const double lonDestRad = lambda * piOver180;
        const double latDestRad = phi * piOver180;
    
        const double cosLatRef = std::cos(latRefRad);
        const double cosLonRef = std::cos(lonRefRad);
        const double sinLatRef = std::sin(latRefRad);
        const double sinLonRef = std::sin(lonRefRad);
        const double cosLatDest = std::cos(latDestRad);
        const double cosLonDest = std::cos(lonDestRad);
        const double sinLatDest = std::sin(latDestRad);
        const double sinLonDest = std::sin(lonDestRad);

        const double NRef = PrimeVerticalRadiusOfCurvature(sinLatRef);
        const double NDest = PrimeVerticalRadiusOfCurvature(sinLatDest);

        const double dX = (NDest + height) * cosLatDest * cosLonDest - (NRef + heightR) * cosLatRef * cosLonRef;
        const double dY = (NDest + height) * cosLatDest * sinLonDest - (NRef + heightR) * cosLatRef * sinLonRef;
        const double dZ = ((1 - ECCENTRICITY_SQUARED) * NDest + height) * sinLatDest - ((1 - ECCENTRICITY_SQUARED) * NRef + heightR) * sinLatRef;
    
        E = -dX * sinLonRef + dY * cosLonRef;
        N = -dX * sinLatRef * cosLonRef - dY * sinLatRef * sinLonRef + dZ * cosLatRef;
        U = dX * cosLatRef * cosLonRef + dY * cosLatRef * sinLonRef + dZ * sinLatRef;
    }

    /*
     * @brief Converts ENU coordinates in meters to Geodetic WGS84 coordinates
     *
     * @param [in] E            E coordinate in ENU frame in meters
     * @param [in] N            N coordinate in ENU frame in meters
     * @param [in] U            U coordinate in ENU frame in meters
     * @param [in] lambdaR      longitude of reference point in degrees
     * @param [in] phiR         latitude of reference point in degrees
     * @param [in] heightR      height of reference point in meters
     * @param [out] WGS_phi     WGS84 latitude in degrees
     * @param [out] WGS_lambda  WGS84 longitude in degrees
     * @param [out] height      WGS84 altitude in degrees
    */
    inline void ENU_To_WGS84(const double E, const double N, const double U, const double lambdaR, const double phiR, const double heightR, double& WGS_phi, double& WGS_lambda, double& height) {
        constexpr double oneMinusESq = 1.0 - ECCENTRICITY_SQUARED;
        constexpr double aSqMinusBSq = SEMI_MAJOR_AXIS_M * SEMI_MAJOR_AXIS_M - SEMI_MINOR_AXIS_M * SEMI_MINOR_AXIS_M;
        constexpr double aSq = SEMI_MAJOR_AXIS_M * SEMI_MAJOR_AXIS_M;
        constexpr double bSq = SEMI_MINOR_AXIS_M * SEMI_MINOR_AXIS_M;
        constexpr double ePrimSq = ( aSq - bSq ) / bSq;
        constexpr double eSqSq = ECCENTRICITY_SQUARED * ECCENTRICITY_SQUARED;

        const double lambdaRad = lambdaR * piOver180;
        const double phiRad = phiR * piOver180;
        const double sinLambda = std::sin(lambdaRad);
        const double cosLambda = std::cos(lambdaRad);
        const double sinPhi = std::sin(phiRad);
        const double cosPhi = std::cos(phiRad);
        
        // Convert to ECEF
        const double NLocal = PrimeVerticalRadiusOfCurvature(sinPhi);
        const double Xr = (NLocal + heightR) * cosPhi * cosLambda;
        const double Yr = (NLocal + heightR) * cosPhi * sinLambda;
        const double Zr = ((1 - ECCENTRICITY_SQUARED) * NLocal + heightR) * sinPhi;
        const double Xp = -E * sinLambda - N * sinPhi * cosLambda + U * cosPhi * cosLambda + Xr;
        const double Yp = E * cosLambda - N * sinPhi * sinLambda + U * cosPhi * sinLambda + Yr;
        const double Zp = N * cosPhi + U * sinPhi + Zr;

        // Convert to WGS84
        const double pSq = Xp * Xp + Yp * Yp;
        const double p = std::sqrt(pSq);
        const double ZSq = Zp * Zp;

        const double F = 54.0 * bSq * ZSq;
        const double G = pSq + oneMinusESq * ZSq - ECCENTRICITY_SQUARED * aSqMinusBSq;

        const double c = eSqSq * F * pSq / ( G * G * G );
        const double s = std::cbrt( 1.0 + c + std::sqrt( c * c + 2.0 * c ) );
        const double kappa = s + 1.0 + ( 1.0 / s );

        const double P = F / ( 3.0 * kappa * kappa * G * G );
        const double Q = std::sqrt( 1.0 + 2.0 * eSqSq * P );

        const double r0 = ( -P * ECCENTRICITY_SQUARED * p ) / ( 1.0 + Q ) + 
                          std::sqrt( 0.5 * aSq * ( 1.0 + ( 1.0 / Q ) ) - ( P * oneMinusESq * ZSq ) / ( Q * ( 1.0 + Q ) ) - 0.5 * P * pSq );

        const double innerRootUTerm = p - ECCENTRICITY_SQUARED * r0;
        const double innerRootUSqTerm = innerRootUTerm * innerRootUTerm;
        const double V = std::sqrt( innerRootUSqTerm + oneMinusESq * ZSq );

        const double bSqOverAV = bSq / ( SEMI_MAJOR_AXIS_M * V );

        const double z0 = bSqOverAV * Zp;

        height = std::sqrt( innerRootUSqTerm + ZSq ) * ( 1.0 - bSqOverAV );
        WGS_phi = std::atan2( Zp + ePrimSq * z0 , p ) * one80OverPi;
        WGS_lambda = std::atan2( Yp, Xp ) * one80OverPi;
    }
    
    inline void LocalENUVectorToOriginENU(double& E0, double& N0, double& U0, const double E, const double N, const double U, const double currentLon, const double currentLat, const double originLon, const double originLat) {
        const double currentLonRad = currentLon * piOver180;
        const double currentLatRad = currentLat * piOver180;
        const double originLonRad = originLon * piOver180;
        const double originLatRad = originLat * piOver180;

        const double sinCurrentLon = std::sin(currentLonRad);
        const double cosCurrentLon = std::cos(currentLonRad);
        const double sinCurrentLat = std::sin(currentLatRad);
        const double cosCurrentLat = std::cos(currentLatRad);
        const double sinOriginLon = std::sin(originLonRad);
        const double cosOriginLon = std::cos(originLonRad);
        const double sinOriginLat = std::sin(originLatRad);
        const double cosOriginLat = std::cos(originLatRad);

        const double X = -E * sinCurrentLon - N * sinCurrentLat * cosCurrentLon + U * cosCurrentLat * cosCurrentLon;
        const double Y = E * cosCurrentLon - N * sinCurrentLat * sinCurrentLon + U * cosCurrentLat * sinCurrentLon;
        const double Z = N * cosCurrentLat + U * sinCurrentLat;

        E0 = -X * sinOriginLon + Y * cosOriginLon;
        N0 = -X * sinOriginLat * cosOriginLon - Y * sinOriginLat * sinOriginLon + Z * cosOriginLat;
        U0 = X * cosOriginLat * cosOriginLon + Y * cosOriginLat * sinOriginLon + Z * sinOriginLat;
    }

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

    inline double ComputeENUHeading(double i, double j, double k, double real, double dRoll, double dPitch, double dYaw, double dt, double declinationRadians) {
        double qx = i;
        double qy = j;
        double qz = k;
        double qw = real;

        const double qNorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

        if (qNorm <= 1e-12) {
            throw std::runtime_error("Invalid rotation-vector quaternion");
        }

        qx /= qNorm;
        qy /= qNorm;
        qz /= qNorm;
        qw /= qNorm;

        const double omega = std::sqrt(dRoll * dRoll + dPitch * dPitch + dYaw * dYaw);

        double dqx;
        double dqy;
        double dqz;
        double dqw;

        if (omega > 1e-12 && dt > 0.0) {
            const double halfAngle = 0.5 * omega * dt;
            const double scale = std::sin(halfAngle) / omega;

            dqx = dRoll  * scale;
            dqy = dPitch * scale;
            dqz = dYaw   * scale;
            dqw = std::cos(halfAngle);
        } else {
            dqx = 0.5 * dRoll  * dt;
            dqy = 0.5 * dPitch * dt;
            dqz = 0.5 * dYaw   * dt;
            dqw = 1.0;
        }

        const double predictedX = qw * dqx + qx * dqw + qy * dqz - qz * dqy;
        const double predictedY = qw * dqy - qx * dqz + qy * dqw + qz * dqx;
        const double predictedZ = qw * dqz + qx * dqy - qy * dqx + qz * dqw;
        const double predictedW = qw * dqw - qx * dqx - qy * dqy - qz * dqz;

        const double predictedNorm = std::sqrt(predictedX * predictedX + predictedY * predictedY + predictedZ * predictedZ + predictedW * predictedW);

        qx = predictedX / predictedNorm;
        qy = predictedY / predictedNorm;
        qz = predictedZ / predictedNorm;
        qw = predictedW / predictedNorm;

        const double magneticENUHeading = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        const double trueENUHeading = magneticENUHeading - declinationRadians;

        return std::remainder(trueENUHeading, 2.0 * 3.141592653589793);
    }

}; // namespace IMUUtils

#endif // IMU_UTILS_HPP