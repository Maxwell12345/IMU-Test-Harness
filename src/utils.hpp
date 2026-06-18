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

namespace IMUUtils
{
  inline double METERS_PER_DEGREE = 111111.1;
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

    return degrees * M_PI / 180.0;
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
   * @brief   Converts a linear acceleration along the surface of the Earth oriented East/West from meters/(second^2)
   * into degrees_longitude / (second^2)
   *
   * @param [in] boat_latitude    The latitude of the boat at the moment in time the acceleration was measured, in
   * units of decimal degrees (DD.dddddd). Latitudes south of the equator are entered as negative values
   * @param [in] global_x     The linear acceleration of the boat along the East/West axis with East being positive
   *
   * @return  A double containing the longitudinal acceleration in degrees per second per second, with East being positive
   *
   * @throws out_of_range    If the inputted latitude is outside the inclusive bounds of  [-90.0 , 90.0] decimal degrees
   */
  inline double Convert_Global_X_to_DegPerS2(double boat_latitude, double global_x) {
      if (boat_latitude < -90.0 || boat_latitude > 90.0) {
          throw std::out_of_range("Latitude values must be between [-90.0 , 90.0] degrees in degreee decimal form (e.g. 12.3456).");
      }
      return global_x / (METERS_PER_DEGREE * std::cos(boat_latitude * IMUUtils::RAD_PER_DEGREE));
  }

  /**
   *
   * @brief   Converts a linear acceleration along the surface of the Earth oriented North/South from meters/(second^2)
   * into degrees_latitude / (second^2)
   *
   * @param [in] global_y     The linear acceleration of the boat along the North/South axis with North being positive
   *
   * @return  A double containing the latitudinal acceleration in degrees per second per second, with North being positive
   */
  inline double Convert_Global_Y_to_DegPerS2(double global_y) {
      return global_y / METERS_PER_DEGREE;
  }

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
      // double deltaX = previous.speedEastWest * deltaT + .5 * accelerationEastWest * deltaT;

      current.accelerationNorthSouth = accelerationNorthSouth;
      current.speedNorthSouth= previous.speedNorthSouth + accelerationNorthSouth * deltaT;
      // double deltaX = previous.speedNorthSouth * deltaT + .5 * accelerationNorthSouth * deltaT;

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
    double heading = std::atan2(2 * ((w * k) + (i * j)), 1 - 2 * ((j * j) + (k * k)));

      if(w < 0.0 || w > 1.0) {
        throw std::out_of_range("Invalid scalar. Must been between 0 and 1.");
      }
    
    heading = heading * 180.0 / M_PI;

      while(heading > 360) {
        heading -= 360;
      };

      while(heading < 0) {
        heading += 360;
      };

      return heading;
  };
}; // namespace IMUUtils

#endif // IMU_UTILS_HPP