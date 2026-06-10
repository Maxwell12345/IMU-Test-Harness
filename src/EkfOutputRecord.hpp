#ifndef EKF_OUTPUT_RECORD_HPP
#define EKF_OUTPUT_RECORD_HPP

#include <chrono>
#include <cstdio>
#include <string>
#include <stdint.h>
#include <variant>
#include <stdexcept>

#include <Eigen/Dense> 

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct EkfOutputRecord {
    double timestamp;
    double x;
    double y;
    double vx;
    double vy;
    double ax;
    double ay;
    bool hasGps;
    
    EkfOutputRecord(): timestamp(0),
                       x(0),
                       y(0),
                       vx(0),
                       vy(0),
                       ax(0),
                       ay(0)
                       {};

    EkfOutputRecord(const Vector6d& v) {
        timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        x = v[0];
        y = v[1];
        vx = v[2];
        vy = v[3];
        ax = v[4];
        ay = v[5];
    }

    std::string toString() const {
        char cs[512];
        snprintf(cs, sizeof(cs), "Timestamp: %ld, x: %f, y: %f, vx: %f, vy: %f, ax: %f, ay: %f",
                 timestamp,
                 x,
                 y,
                 vx,
                 vy,
                 ax,
                 ay);

        return std::string(cs);
    }
};

#endif