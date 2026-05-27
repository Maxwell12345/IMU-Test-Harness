#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Dense> 
#include <mutex>
#include "IMUGPSFusionKF.hpp"
#include "MagneticDeclination.hpp"

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

struct GpsSample {
    long long t_ms{};
    int hour{};
    int minute{};
    int second{};
    int millisecond{};
    double latitude{};
    double longitude{};
};

struct ImuSample {
    long long t_ms{};
    int hour{};
    int minute{};
    int second{};
    int millisecond{};
    Vec3 accel{};
    Vec3 mag{};
    Vec3 gyro{};
    double heading{};
    double roll{};
    double pitch{};
    double declination_angle{};
    std::array<double, 4> quat{};
    Vec3 linear_accel{};
    Vec3 gravity{};
};

static constexpr double kEarthRadiusMeters = 6378137.0;
static constexpr double kPi = 3.141592653589793238462643383279502884;

static double deg_to_rad(double deg) {
    return deg * kPi / 180.0;
}

static std::vector<std::string> split_semicolon(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    std::stringstream ss(line);

    while (std::getline(ss, field, ';')) {
        if (!field.empty() && field.back() == '\r') {
            field.pop_back();
        }
        if (!field.empty()) {
            fields.push_back(field);
        }
    }

    return fields;
}

static long long day_ms(int hour, int minute, int second, int millisecond) {
    return static_cast<long long>(hour) * 3600000LL +
           static_cast<long long>(minute) * 60000LL +
           static_cast<long long>(second) * 1000LL +
           static_cast<long long>(millisecond);
}

static int16_t int16_le(int lo, int hi) {
    const uint16_t u = static_cast<uint16_t>((lo & 0xff) | ((hi & 0xff) << 8));
    if (u >= 32768) {
        return static_cast<int16_t>(static_cast<int>(u) - 65536);
    }
    return static_cast<int16_t>(u);
}

static double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3 cross(Vec3 a, Vec3 b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

static Vec3 sub(Vec3 a, Vec3 b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 mul(Vec3 a, double s) {
    return Vec3{a.x * s, a.y * s, a.z * s};
}

static double norm(Vec3 a) {
    return std::sqrt(dot(a, a));
}

static bool normalize(Vec3& a) {
    const double n = norm(a);
    if (!std::isfinite(n) || n < 1e-12) {
        return false;
    }
    a = mul(a, 1.0 / n);
    return true;
}

static ImuSample parse_imu_fields(const std::vector<std::string>& f, long long t_ms) {
    if (f.size() < 48) {
        throw std::runtime_error("IMU row needs 48 semicolon fields");
    }

    ImuSample out;
    out.hour = std::stoi(f[0]);
    out.minute = std::stoi(f[1]);
    out.second = std::stoi(f[2]);
    out.millisecond = std::stoi(f[3]);
    out.t_ms = t_ms;

    std::array<int, 44> b{};
    for (int i = 0; i < 44; ++i) {
        b[static_cast<size_t>(i)] = std::stoi(f[static_cast<size_t>(4 + i)]);
    }

    std::array<int16_t, 22> raw{};
    for (int i = 0; i < 22; ++i) {
        raw[static_cast<size_t>(i)] = int16_le(b[static_cast<size_t>(2 * i)], b[static_cast<size_t>(2 * i + 1)]);
    }

    out.accel = Vec3{raw[0] / 100.0, raw[1] / 100.0, raw[2] / 100.0};
    out.mag = Vec3{raw[3] / 16.0, raw[4] / 16.0, raw[5] / 16.0};
    out.gyro = Vec3{raw[6] / 16.0, raw[7] / 16.0, raw[8] / 16.0};
    out.heading = raw[9] / 16.0;
    out.roll = raw[10] / 16.0;
    out.pitch = raw[11] / 16.0;
    out.quat = {raw[12] / 16384.0, raw[13] / 16384.0, raw[14] / 16384.0, raw[15] / 16384.0};
    out.linear_accel = Vec3{raw[16] / 100.0, raw[17] / 100.0, raw[18] / 100.0};
    out.gravity = Vec3{raw[19] / 100.0, raw[20] / 100.0, raw[21] / 100.0};

    return out;
}

static GpsSample parse_gps_fields(const std::vector<std::string>& f, long long t_ms) {
    if (f.size() < 6) {
        throw std::runtime_error("GPS row needs 6 semicolon fields");
    }

    GpsSample out;
    out.hour = std::stoi(f[0]);
    out.minute = std::stoi(f[1]);
    out.second = std::stoi(f[2]);
    out.millisecond = std::stoi(f[3]);
    out.latitude = std::stod(f[4]);
    out.longitude = std::stod(f[5]);
    out.t_ms = t_ms;
    return out;
}

static std::vector<ImuSample> load_imu(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Could not open IMU file: " + path);
    }

    std::vector<ImuSample> samples;
    std::string line;
    long long day_offset = 0;
    long long last_raw = -1;
    const long long one_day = 86400000LL;

    while (std::getline(file, line)) {
        const auto fields = split_semicolon(line);
        if (fields.empty()) {
            continue;
        }
        if (fields.size() < 48) {
            throw std::runtime_error("Bad IMU row in file: " + path);
        }

        const int hour = std::stoi(fields[0]);
        const int minute = std::stoi(fields[1]);
        const int second = std::stoi(fields[2]);
        const int millisecond = std::stoi(fields[3]);
        const long long raw = day_ms(hour, minute, second, millisecond);

        if (last_raw >= 0 && raw < last_raw && last_raw - raw > one_day / 2) {
            day_offset += one_day;
        }

        samples.push_back(parse_imu_fields(fields, day_offset + raw));
        last_raw = raw;
    }

    std::sort(samples.begin(), samples.end(), [](const ImuSample& a, const ImuSample& b) {
        return a.t_ms < b.t_ms;
    });

    return samples;
}

static std::vector<GpsSample> load_gps(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Could not open GPS file: " + path);
    }

    std::vector<GpsSample> samples;
    std::string line;
    long long day_offset = 0;
    long long last_raw = -1;
    const long long one_day = 86400000LL;

    while (std::getline(file, line)) {
        const auto fields = split_semicolon(line);
        if (fields.empty()) {
            continue;
        }
        if (fields.size() < 6) {
            throw std::runtime_error("Bad GPS row in file: " + path);
        }

        const int hour = std::stoi(fields[0]);
        const int minute = std::stoi(fields[1]);
        const int second = std::stoi(fields[2]);
        const int millisecond = std::stoi(fields[3]);
        const long long raw = day_ms(hour, minute, second, millisecond);

        if (last_raw >= 0 && raw < last_raw && last_raw - raw > one_day / 2) {
            day_offset += one_day;
        }

        samples.push_back(parse_gps_fields(fields, day_offset + raw));
        last_raw = raw;
    }

    std::sort(samples.begin(), samples.end(), [](const GpsSample& a, const GpsSample& b) {
        return a.t_ms < b.t_ms;
    });

    return samples;
}

static bool magnetic_global_accel_ne(const ImuSample& imu, double& north_mps2, double& east_mps2) {
    Vec3 down = imu.gravity;
    Vec3 mag = imu.mag;

    if (!std::isfinite(imu.declination_angle)) {
        return false;
    }

    if (!normalize(down) || !normalize(mag)) {
        return false;
    }

    Vec3 magnetic_north = sub(mag, mul(down, dot(mag, down)));
    if (!normalize(magnetic_north)) {
        return false;
    }

    Vec3 magnetic_east = cross(down, magnetic_north);
    if (!normalize(magnetic_east)) {
        return false;
    }

    const double magnetic_n = dot(imu.linear_accel, magnetic_north);
    const double magnetic_e = dot(imu.linear_accel, magnetic_east);

    const double d = deg_to_rad(imu.declination_angle);
    const double c = std::cos(d);
    const double s = std::sin(d);

    north_mps2 = c * magnetic_n - s * magnetic_e;
    east_mps2 = s * magnetic_n + c * magnetic_e;

    return true;
}

static double meters_per_lat_degree() {
    return kEarthRadiusMeters * kPi / 180.0;
}

static double meters_per_lon_degree(double lat_deg) {
    const double c = std::cos(lat_deg * kPi / 180.0);
    const double safe_c = std::abs(c) < 1e-9 ? (c < 0.0 ? -1e-9 : 1e-9) : c;
    return meters_per_lat_degree() * safe_c;
}

static constexpr const char* kFilterCsvPath = "kf_state.csv";
static std::mutex g_filter_csv_mtx;

static bool filter_csv_needs_header(const char* path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    return !in.good() || in.tellg() == std::streampos(0);
}


static void write_filter_csv_header(std::ostream& csv) {
    csv << "step,dt_s,has_gps,"
        << "imu_z_lon,imu_z_lat,imu_z_vel_lon,imu_z_vel_lat,imu_z_acc_lon,imu_z_acc_lat,"
        << "gps_z_lon,gps_z_lat,gps_z_vel_lon,gps_z_vel_lat,gps_z_acc_lon,gps_z_acc_lat,"
        << "x_lon,x_lat,x_vel_lon,x_vel_lat,x_acc_lon,x_acc_lat";

    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            csv << ",P_" << r << "_" << c;
        }
    }

    csv << '\n';
}

static void write_vec6_csv(std::ostream& csv, const Vector6d& v) {
    for (int i = 0; i < 6; ++i) {
        csv << ',' << v(i);
    }
}

static void write_nan_vec6_csv(std::ostream& csv) {
    for (int i = 0; i < 6; ++i) {
        csv << ",nan";
    }
}

static void log_filter_state_csv(
    double dt,
    const Vector6d& x,
    const Matrix6d& P,
    const Vector6d& imu,
    const Vector6d& gps,
    bool has_gps
) {
    std::lock_guard<std::mutex> lock(g_filter_csv_mtx);

    static std::ofstream csv;
    static bool initialized = false;
    static unsigned long long step = 0;

    if (!initialized) {
        const bool need_header = filter_csv_needs_header(kFilterCsvPath);

        csv.open(kFilterCsvPath, std::ios::out | std::ios::app);
        if (!csv) {
            std::cerr << "Error: failed to open filter CSV file: "
                      << kFilterCsvPath << '\n';
            return;
        }

        if (need_header) {
            write_filter_csv_header(csv);
            csv.flush();
        }

        initialized = true;
    }

    csv << std::setprecision(17)
        << step++ << ','
        << dt << ','
        << (has_gps ? 1 : 0);

    // IMU measurement vector:
    // imu = [0, 0, vel_lon, vel_lat, acc_lon, acc_lat]
    write_vec6_csv(csv, imu);

    // GPS measurement vector:
    // gps = [lon, lat, 0, 0, 0, 0]
    // If no GPS was associated with this IMU step, write nan fields.
    if (has_gps) {
        write_vec6_csv(csv, gps);
    } else {
        write_nan_vec6_csv(csv);
    }

    // Filter state:
    // x = [lon, lat, vel_lon, vel_lat, acc_lon, acc_lat]
    write_vec6_csv(csv, x);

    // Filter covariance.
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            csv << ',' << P(r, c);
        }
    }

    csv << '\n';
    csv.flush();
}

static void run_filter(double acc_lat, double acc_lon, double vel_lat, double vel_lon, const GpsSample* gps, double dt, IMUGPSFusionKF_2D_ConstantAcceleration kf) {
    std::pair<Vector6d, Matrix6d> x_P_pair;

    Vector6d gps_;
    Vector6d imu;

    // if (gps) {
    //     double lat = gps->latitude;
    //     double lon = gps->longitude;

    //     gps_ << lon, lat, 0.0, 0.0, 0.0, 0.0;
    //     imu << 0.0, 0.0, vel_lon, vel_lat, acc_lon, acc_lat;

    //     x_P_pair = kf.Step(dt, gps_, imu);
    // }
    // else {
        imu << 0.0, 0.0, vel_lon, vel_lat, acc_lon, acc_lat;

        x_P_pair = kf.Step(dt, imu);
    // }

    const bool has_gps = gps != nullptr;

    log_filter_state_csv(dt, x_P_pair.first, x_P_pair.second, imu, gps_, false);
}

int main(int argc, char** argv) {
    std::vector<ImuSample> imu_samples = load_imu(argv[1]);
    const std::vector<GpsSample> gps_samples = load_gps(argv[2]);

    if (imu_samples.empty()) {
        throw std::runtime_error("IMU file had no samples");
    }

    MagneticDeclination magneticDeclination = MagneticDeclination();
    magneticDeclination.LoadCOF("/home/idler/workspace/inu-display/test/WMM.COF");

    size_t next_gps = 0;
    double current_lat = gps_samples.empty() ? 0.0 : gps_samples.front().latitude;
    double vel_n_mps = 0.0;
    double vel_e_mps = 0.0;
    long long last_imu_t = imu_samples.front().t_ms;

    double declination_angle = 0.0;

    bool is_kf_init = false;

    if (!gps_samples.empty()) {
        declination_angle = magneticDeclination.CalculateDeclination(gps_samples.front().longitude, gps_samples.front().latitude, 244.0, 2024.0);
    }

    Vector6d x0;
    x0 << gps_samples.front().longitude, current_lat, 2/111111.1, 1/111111.1, 2/111111.1, 1/111111.1;
    Matrix6d P0;
    P0 << 5.0/ 111111.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 5.3/ 111111.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.12/ 111111.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.031/ 111111.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.1/ 111111.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.9/ 111111.0;

    Matrix6d R_GPS = Matrix6d::Identity() * 1e-13;
    R_GPS(0,0) = 2/111111.1;
    R_GPS(1,1) = 1/111111.1;
    Matrix6d R_IMU = Matrix6d::Identity() * 1e-13;
    R_GPS(2,2) = 3/111111.1;
    R_GPS(3,3) = 4/111111.1;
    R_GPS(4,4) = 1.3/111111.1;
    R_GPS(5,5) = 1.9/111111.1;

    std::cout << x0 << std::endl;

    Matrix6d Q0 = Matrix6d::Identity() * 0.00001;

    IMUGPSFusionKF_2D_ConstantAcceleration kf = IMUGPSFusionKF_2D_ConstantAcceleration(x0, P0, R_GPS, R_IMU, Q0, 4.605, 7.779, 9.210, 13.277, 10, 2, 200, 10, 200, 10);
    
    for (auto& imu : imu_samples) {
        const GpsSample* associated_gps = nullptr;

        while (next_gps < gps_samples.size() && gps_samples[next_gps].t_ms <= imu.t_ms) {
            associated_gps = &gps_samples[next_gps];
            current_lat = associated_gps->latitude;
            declination_angle = magneticDeclination.CalculateDeclination(associated_gps->longitude, associated_gps->latitude, 244.0, 2024.0);
            ++next_gps;
        }

        imu.declination_angle = declination_angle;

        double acc_n_mps2 = 0.0;
        double acc_e_mps2 = 0.0;
        const bool valid_accel = magnetic_global_accel_ne(imu, acc_n_mps2, acc_e_mps2);
        const double dt = std::max(0.0, static_cast<double>(imu.t_ms - last_imu_t) / 1000.0);

        if (valid_accel) {
            vel_n_mps += acc_n_mps2 * dt;
            vel_e_mps += acc_e_mps2 * dt;

            const double m_per_lat = meters_per_lat_degree();
            const double m_per_lon = meters_per_lon_degree(current_lat);
            const double acc_lat_deg_s2 = acc_n_mps2 / m_per_lat;
            const double acc_lon_deg_s2 = acc_e_mps2 / m_per_lon;
            const double vel_lat_deg_s = vel_n_mps / m_per_lat;
            const double vel_lon_deg_s = vel_e_mps / m_per_lon;

            run_filter(acc_lat_deg_s2, acc_lon_deg_s2, vel_lat_deg_s, vel_lon_deg_s, associated_gps, dt, kf);
            last_imu_t = imu.t_ms;
        } else {
            acc_n_mps2 = std::numeric_limits<double>::quiet_NaN();
            acc_e_mps2 = std::numeric_limits<double>::quiet_NaN();

            std::cout << "BAD!" << std::endl;
        }
    }
}
