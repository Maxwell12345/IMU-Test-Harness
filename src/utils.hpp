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

#include <cmath>
#include <string>
#include <stdexcept>

namespace IMUUtils
{

    /**
     * Converts Degrees to Radians from IMU readings.
     *
     * Expected behavior:
     *  - Ensures the heading is normalized to [0.0 -> 360.0] degrees
     *  - Returns heading in radians
     *
     * Assumptions:
     *  -Input is a valid heading in degrees (not NaN)
     *
     *  @param  degrees Heading angle in degrees
     *  @return         Normalized heading in radians from [0.0 -> 2 * pi]
     */
    double DegreesToRadians(double degrees)
    {
        while (degrees >= 360.0)
        {
            degrees -= 360.0;
        }
        while (degrees < 0.0)
        {
            degrees += 360.0;
        }

        return degrees * M_PI / 180.0;
    };

    /**
     * Converts to Global X Acceleration from IMU (x,y) readings.
     *
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
     * @return          Linear acceleration in global x axis (East/West with East being positive) measured in meters per sec per second.
     */
    double InertialToGlobal_X(double theta_t, double boat_x, double boat_y)
    {
        double global_X = std::cos(theta_t) * boat_x + std::sin(theta_t) * boat_y;
        return global_X;
    };
    /**
     * Converts to Global Y Acceleration from IMU (x,y) readings.
     *
     * Expected behavior:
     * - Returns linear acceleration in global Y (North-positive) in meters / second / second.
     * - Uses the same rotation formula as defined in global_rotation_tests.cpp
     *
     * Assumptions:
     * - theta_t is heading in radians measured from True North
     * - boat_x and boat_y are accelerations in boat frame (right/forward)
     *
     * @param   theta_t Boat heading in radians from True North.
     * @param   boat_x  The boat's linear acceleration in the x-axis in meters per second (m/s).
     * @param   boat_y  The boat's linear acceleration in the y-axis in meters per second(m/s).
     * @return          Linear acceleration in global y axis (North/South with North being positive) measured in meters per sec per second.
     *
     */
    double InertialToGlobal_Y(double theta_t, double boat_x, double boat_y)
    {
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
    double MagneticToTrueHeading(double magneticHeading, double declinationAngle) {
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

} // namespace IMUUtils

#endif // IMU_UTILS_HPP