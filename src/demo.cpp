#include "demo.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <signal.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>

#include "bno085_hal.hpp"  // brings in sh2.h and sh2_err.h under extern "C"

extern "C" {
#include "sh2_SensorValue.h"
}

// ---------------------------------------------------------------------------
// Field-safe configuration
// ---------------------------------------------------------------------------

// The program writes into this directory unless IMU_GPS_LOG_DIR is set.
// Example override:
//     IMU_GPS_LOG_DIR=/mnt/usb/logs ./your_program
static constexpr const char* kDefaultLogDir = "/home/pi/logs";

static constexpr const char* kImuCsvFileName  = "imu_measurements.csv";
static constexpr const char* kNmeaCsvFileName = "nmea_sentences.csv";

// Change this if your GPS is on a different Pi serial port.
static constexpr const char* kNmeaPort = "/dev/ttyACM0";
static constexpr int         kNmeaBaud = 9600;

// Durability policy.
// Data is written immediately with write(2), then fdatasync(2) is called
// periodically. Header rows are always synced immediately.
//
// At the default IMU rates below, this usually syncs the IMU file roughly every
// 0.1 to 0.25 seconds, depending on row rate. GPS rows usually sync every row.
static constexpr int      kSyncIntervalMs    = 250;
static constexpr uint64_t kMaxRowsBeforeSync = 256;

// For maximum power-loss protection, set this to true. It is much slower and
// may not keep up with high-rate IMU logging on an SD card.
static constexpr bool kSyncEveryRow = false;

// For even stricter write behavior, set this to true. This opens files with
// O_DSYNC when available. It can be very slow on SD cards.
static constexpr bool kOpenWithODsync = false;

static std::atomic<bool> g_running{true};
static volatile sig_atomic_t g_stop_requested = 0;

static std::mutex g_imu_csv_mtx;
static std::unique_ptr<class DurableAppendFile> g_imu_csv;
static std::string g_log_dir;

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

static void request_stop(int /*signum*/) {
    g_stop_requested = 1;
}

static void install_signal_handlers() {
    struct sigaction sa{};
    sa.sa_handler = request_stop;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

static bool keep_running() {
    return g_running.load(std::memory_order_relaxed) && !g_stop_requested;
}

static void sleep_interruptible_ms(int total_ms) {
    int slept = 0;
    while (slept < total_ms && keep_running()) {
        const int chunk = (total_ms - slept > 10) ? 10 : (total_ms - slept);
        std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
        slept += chunk;
    }
}

static bool directory_exists(const std::string& path) {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

static std::string parent_dir_of(const std::string& path) {
    const size_t slash = path.find_last_of('/');
    if (slash == std::string::npos) return ".";
    if (slash == 0) return "/";
    return path.substr(0, slash);
}

static bool mkdir_p(const std::string& path) {
    if (path.empty()) return false;
    if (directory_exists(path)) return true;

    std::string current;
    size_t i = 0;

    if (path[0] == '/') {
        current = "/";
        i = 1;
    }

    while (i < path.size()) {
        const size_t slash = path.find('/', i);
        const std::string part = path.substr(
            i,
            slash == std::string::npos ? std::string::npos : slash - i
        );

        if (!part.empty()) {
            if (!current.empty() && current.back() != '/') {
                current.push_back('/');
            }
            current += part;

            if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                std::cerr << "[ERROR] mkdir failed for " << current
                          << ": " << std::strerror(errno) << "\n";
                return false;
            }

            if (!directory_exists(current)) {
                std::cerr << "[ERROR] Path exists but is not a directory: "
                          << current << "\n";
                return false;
            }
        }

        if (slash == std::string::npos) break;
        i = slash + 1;
    }

    return directory_exists(path);
}

static std::string path_join(const std::string& dir, const std::string& file) {
    if (dir.empty()) return file;
    if (dir.back() == '/') return dir + file;
    return dir + "/" + file;
}

static std::string choose_log_dir() {
    const char* env_dir = std::getenv("IMU_GPS_LOG_DIR");
    if (env_dir && env_dir[0] != '\0') {
        return std::string(env_dir);
    }

    if (mkdir_p(kDefaultLogDir)) {
        return std::string(kDefaultLogDir);
    }

    const std::string fallback = "./logs";
    std::cerr << "[WARN] Falling back to " << fallback << " for log output\n";
    return fallback;
}

static bool file_exists_and_size(const std::string& path, off_t& size_out) {
    struct stat st{};
    if (::stat(path.c_str(), &st) != 0) return false;
    size_out = st.st_size;
    return true;
}

static bool fsync_parent_directory(const std::string& path) {
    const std::string parent = parent_dir_of(path);

    int dfd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (dfd < 0) {
        std::cerr << "[WARN] Could not open parent directory for fsync: "
                  << parent << ": " << std::strerror(errno) << "\n";
        return false;
    }

    const bool ok = (::fsync(dfd) == 0);
    if (!ok) {
        std::cerr << "[WARN] fsync failed for parent directory " << parent
                  << ": " << std::strerror(errno) << "\n";
    }

    ::close(dfd);
    return ok;
}

static double epoch_seconds_ms() {
    using namespace std::chrono;

    const auto now = system_clock::now();
    const auto ms_since_epoch =
        duration_cast<milliseconds>(now.time_since_epoch());

    return static_cast<double>(ms_since_epoch.count()) / 1000.0;
}

static std::string csv_quote(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 2);

    out.push_back('"');

    for (char c : value) {
        if (c == '"') {
            out += "\"\"";
        } else if (c != '\r' && c != '\n') {
            out.push_back(c);
        }
    }

    out.push_back('"');
    return out;
}

// ---------------------------------------------------------------------------
// Durable append-only CSV writer
// ---------------------------------------------------------------------------

class DurableAppendFile {
public:
    DurableAppendFile(std::string path, std::string header)
        : path_(std::move(path)), header_(std::move(header))
    {
        open_file();
    }

    ~DurableAppendFile() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (fd_ >= 0) {
            sync_locked();
            ::close(fd_);
            fd_ = -1;
        }
    }

    DurableAppendFile(const DurableAppendFile&) = delete;
    DurableAppendFile& operator=(const DurableAppendFile&) = delete;

    bool good() const {
        return fd_ >= 0;
    }

    const std::string& path() const {
        return path_;
    }

    bool write_line(const std::string& line) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (fd_ < 0) return false;

        if (!write_all_locked(line)) {
            return false;
        }

        ++rows_since_sync_;

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_sync_
        ).count();

        if (kSyncEveryRow ||
            rows_since_sync_ >= kMaxRowsBeforeSync ||
            elapsed_ms >= kSyncIntervalMs) {
            return sync_locked();
        }

        return true;
    }

    bool sync_now() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (fd_ < 0) return false;
        return sync_locked();
    }

private:
    void open_file() {
        const std::string parent = parent_dir_of(path_);
        if (!mkdir_p(parent)) {
            std::cerr << "[ERROR] Could not create log directory: "
                      << parent << "\n";
            return;
        }

        off_t existing_size = 0;
        const bool existed = file_exists_and_size(path_, existing_size);
        const bool need_header = !existed || existing_size == 0;

        int flags = O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC;
#ifdef O_DSYNC
        if (kOpenWithODsync) {
            flags |= O_DSYNC;
        }
#endif

        fd_ = ::open(path_.c_str(), flags, 0644);
        if (fd_ < 0) {
            std::cerr << "[ERROR] Failed to open CSV file " << path_
                      << ": " << std::strerror(errno) << "\n";
            return;
        }

        // If this file has just been created, make the directory entry durable.
        if (!existed) {
            fsync_parent_directory(path_);
        }

        last_sync_ = std::chrono::steady_clock::now();

        if (need_header) {
            if (!write_all_locked(header_)) {
                ::close(fd_);
                fd_ = -1;
                return;
            }

            // The header is always made durable immediately. This prevents
            // a newly-created file from coming back as zero bytes after a
            // sudden power loss, assuming the storage device honors syncs.
            sync_locked();
        }
    }

    bool write_all_locked(const std::string& text) {
        const char* p = text.data();
        size_t left = text.size();

        while (left > 0) {
            const ssize_t n = ::write(fd_, p, left);

            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }

                std::cerr << "[ERROR] write failed for " << path_
                          << ": " << std::strerror(errno) << "\n";
                return false;
            }

            if (n == 0) {
                std::cerr << "[ERROR] write returned zero bytes for "
                          << path_ << "\n";
                return false;
            }

            p += n;
            left -= static_cast<size_t>(n);
        }

        return true;
    }

    bool sync_locked() {
        if (fd_ < 0) return false;

        if (::fdatasync(fd_) != 0) {
            std::cerr << "[ERROR] fdatasync failed for " << path_
                      << ": " << std::strerror(errno) << "\n";
            return false;
        }

        rows_since_sync_ = 0;
        last_sync_ = std::chrono::steady_clock::now();
        return true;
    }

    std::string path_;
    std::string header_;
    int fd_ = -1;
    uint64_t rows_since_sync_ = 0;
    std::chrono::steady_clock::time_point last_sync_{};
    mutable std::mutex mtx_;
};

// ---------------------------------------------------------------------------
// IMU per-measurement logger
// ---------------------------------------------------------------------------

static std::string imu_csv_header() {
    return "epoch_s,"
           "sensor_timestamp_us,"
           "sensor_id,"
           "sensor_name,"
           "status,"
           "accuracy,"
           "x,"
           "y,"
           "z,"
           "i,"
           "j,"
           "k,"
           "real,"
           "rotation_accuracy\n";
}

static void write_imu_prefix(
    std::ostringstream& row,
    const sh2_SensorValue_t& val,
    const char* sensor_name
) {
    const double epoch_s = epoch_seconds_ms();

    const int sensor_id = static_cast<int>(val.sensorId);
    const int status    = static_cast<int>(val.status);
    const int accuracy  = static_cast<int>(val.status & 0x03);

    row << std::fixed << std::setprecision(3)
        << epoch_s << ',';

    row << std::setprecision(9)
        << val.timestamp << ','
        << sensor_id << ','
        << csv_quote(sensor_name) << ','
        << status << ','
        << accuracy << ',';
}

static void log_imu_measurement_csv(const sh2_SensorValue_t& val) {
    std::lock_guard<std::mutex> lock(g_imu_csv_mtx);

    if (!g_imu_csv || !g_imu_csv->good()) {
        return;
    }

    std::ostringstream row;

    switch (val.sensorId) {
        case SH2_ACCELEROMETER:
            write_imu_prefix(row, val, "ACCELEROMETER");
            row << val.un.accelerometer.x << ','
                << val.un.accelerometer.y << ','
                << val.un.accelerometer.z
                << ",,,,,"
                << '\n';
            break;

        case SH2_LINEAR_ACCELERATION:
            write_imu_prefix(row, val, "LINEAR_ACCELERATION");
            row << val.un.linearAcceleration.x << ','
                << val.un.linearAcceleration.y << ','
                << val.un.linearAcceleration.z
                << ",,,,,"
                << '\n';
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            write_imu_prefix(row, val, "GYROSCOPE_CALIBRATED");
            row << val.un.gyroscope.x << ','
                << val.un.gyroscope.y << ','
                << val.un.gyroscope.z
                << ",,,,,"
                << '\n';
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            write_imu_prefix(row, val, "MAGNETIC_FIELD_CALIBRATED");
            row << val.un.magneticField.x << ','
                << val.un.magneticField.y << ','
                << val.un.magneticField.z
                << ",,,,,"
                << '\n';
            break;

        case SH2_ROTATION_VECTOR:
            write_imu_prefix(row, val, "ROTATION_VECTOR");
            row << ",,,"
                << val.un.rotationVector.i << ','
                << val.un.rotationVector.j << ','
                << val.un.rotationVector.k << ','
                << val.un.rotationVector.real << ','
                << val.un.rotationVector.accuracy
                << '\n';
            break;

        case SH2_GAME_ROTATION_VECTOR:
            write_imu_prefix(row, val, "GAME_ROTATION_VECTOR");
            row << ",,,"
                << val.un.gameRotationVector.i << ','
                << val.un.gameRotationVector.j << ','
                << val.un.gameRotationVector.k << ','
                << val.un.gameRotationVector.real << ','
                << '\n';
            break;

        default:
            return;
    }

    g_imu_csv->write_line(row.str());
}

static void sensor_callback(void* /*cookie*/, sh2_SensorEvent_t* event) {
    sh2_SensorValue_t val{};

    if (sh2_decodeSensorEvent(&val, event) != SH2_OK) {
        return;
    }

    log_imu_measurement_csv(val);
}

// ---------------------------------------------------------------------------
// NMEA / GPS per-sentence logger
// ---------------------------------------------------------------------------

static std::string nmea_csv_header() {
    return "epoch_s,"
           "port,"
           "baud,"
           "sentence_id,"
           "checksum_ok,"
           "nmea_sentence\n";
}

static std::string trim_line_endings(std::string s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) {
        s.pop_back();
    }
    return s;
}

static std::string nmea_sentence_id(const std::string& sentence) {
    if (sentence.empty()) return "";

    size_t start = 0;
    if (sentence[0] == '$' || sentence[0] == '!') {
        start = 1;
    }

    size_t end = sentence.find(',', start);
    size_t star = sentence.find('*', start);

    if (end == std::string::npos || (star != std::string::npos && star < end)) {
        end = star;
    }

    if (end == std::string::npos) {
        end = sentence.size();
    }

    if (end <= start) return "";

    return sentence.substr(start, end - start);
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    return -1;
}

// Returns:
//   1  = checksum valid
//   0  = checksum invalid
//  -1  = no usable checksum found
static int nmea_checksum_ok(const std::string& sentence) {
    if (sentence.empty()) return -1;
    if (sentence[0] != '$' && sentence[0] != '!') return -1;

    const size_t star = sentence.find('*');
    if (star == std::string::npos || star + 2 >= sentence.size()) {
        return -1;
    }

    const int hi = hex_value(sentence[star + 1]);
    const int lo = hex_value(sentence[star + 2]);

    if (hi < 0 || lo < 0) {
        return -1;
    }

    const unsigned char expected =
        static_cast<unsigned char>((hi << 4) | lo);

    unsigned char actual = 0;

    for (size_t i = 1; i < star; ++i) {
        actual ^= static_cast<unsigned char>(sentence[i]);
    }

    return actual == expected ? 1 : 0;
}

static bool baud_to_speed(int baud, speed_t& speed) {
    switch (baud) {
        case 4800:   speed = B4800;   return true;
        case 9600:   speed = B9600;   return true;
        case 19200:  speed = B19200;  return true;
        case 38400:  speed = B38400;  return true;
        case 57600:  speed = B57600;  return true;
        case 115200: speed = B115200; return true;

#ifdef B230400
        case 230400: speed = B230400; return true;
#endif

#ifdef B460800
        case 460800: speed = B460800; return true;
#endif

#ifdef B921600
        case 921600: speed = B921600; return true;
#endif

        default:
            return false;
    }
}

static bool configure_serial_port(int fd, int baud) {
    speed_t speed{};

    if (!baud_to_speed(baud, speed)) {
        std::cerr << "[ERROR] Unsupported NMEA baud rate: " << baud << "\n";
        return false;
    }

    struct termios tty{};

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "[ERROR] tcgetattr failed for NMEA serial port: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
        std::cerr << "[ERROR] Failed to set NMEA baud rate: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    // 8N1, raw serial.
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CLOCAL;
    tty.c_cflag |= CREAD;

#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] tcsetattr failed for NMEA serial port: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

static void log_nmea_sentence_csv(
    DurableAppendFile& csv,
    const std::string& port,
    int baud,
    const std::string& sentence
) {
    const double epoch_s = epoch_seconds_ms();
    const std::string sentence_id = nmea_sentence_id(sentence);
    const int checksum_state = nmea_checksum_ok(sentence);

    std::ostringstream row;
    row << std::fixed << std::setprecision(3)
        << epoch_s << ','
        << csv_quote(port) << ','
        << baud << ','
        << csv_quote(sentence_id) << ','
        << checksum_state << ','
        << csv_quote(sentence)
        << '\n';

    csv.write_line(row.str());
}

static int open_and_configure_nmea_port(const std::string& port, int baud) {
    const int fd = ::open(
        port.c_str(),
        O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC
    );

    if (fd < 0) {
        std::cerr << "[ERROR] Failed to open NMEA serial port "
                  << port << ": " << std::strerror(errno) << "\n";
        return -1;
    }

    if (!configure_serial_port(fd, baud)) {
        ::close(fd);
        return -1;
    }

    return fd;
}

static void record_nmea_from_fd(
    int fd,
    DurableAppendFile& csv,
    const std::string& port,
    int baud
) {
    std::string line;
    char buffer[256];

    while (keep_running()) {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        const int poll_result = ::poll(&pfd, 1, 100);

        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "[ERROR] poll failed on NMEA serial port: "
                      << std::strerror(errno) << "\n";
            break;
        }

        if (poll_result == 0) {
            continue;
        }

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            std::cerr << "[ERROR] NMEA serial port disconnected or invalid\n";
            break;
        }

        const ssize_t n = ::read(fd, buffer, sizeof(buffer));

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }

            std::cerr << "[ERROR] read failed on NMEA serial port: "
                      << std::strerror(errno) << "\n";
            break;
        }

        if (n == 0) {
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            const char ch = buffer[i];

            if (ch == '\n') {
                std::string sentence = trim_line_endings(line);
                line.clear();

                if (sentence.empty()) {
                    continue;
                }

                const size_t start = sentence.find_first_of("$!");
                if (start == std::string::npos) {
                    continue;
                }

                sentence.erase(0, start);

                if (sentence.empty()) {
                    continue;
                }

                log_nmea_sentence_csv(csv, port, baud, sentence);
            } else if (ch != '\0') {
                if (line.size() < 1024) {
                    line.push_back(ch);
                } else {
                    line.clear();
                }
            }
        }
    }

    csv.sync_now();
}

static void nmea_recorder_thread(
    std::string port,
    int baud,
    std::string csv_path
) {
    DurableAppendFile csv(csv_path, nmea_csv_header());
    if (!csv.good()) {
        std::cerr << "[ERROR] NMEA CSV logger failed to initialize\n";
        return;
    }

    std::cerr << "[INFO] NMEA recorder will use " << port
              << " @ " << baud << " baud -> " << csv.path() << "\n";

    while (keep_running()) {
        const int fd = open_and_configure_nmea_port(port, baud);

        if (fd < 0) {
            sleep_interruptible_ms(1000);
            continue;
        }

        std::cerr << "[INFO] NMEA recorder connected to " << port << "\n";
        record_nmea_from_fd(fd, csv, port, baud);
        ::close(fd);

        if (keep_running()) {
            std::cerr << "[WARN] NMEA recorder will try to reconnect to "
                      << port << "\n";
            sleep_interruptible_ms(1000);
        }
    }

    csv.sync_now();
    std::cerr << "[INFO] NMEA recorder stopped\n";
}

// ---------------------------------------------------------------------------
// BNO085 helper
// ---------------------------------------------------------------------------

static void enable_sensor(sh2_SensorId_t sensor_id, uint32_t interval_us) {
    sh2_SensorConfig_t cfg{};
    cfg.reportInterval_us = interval_us;

    if (sh2_setSensorConfig(sensor_id, &cfg) != SH2_OK) {
        std::cerr << "[WARN] Failed to enable sensor id="
                  << static_cast<int>(sensor_id)
                  << "\n";
    }
}

// RAII wrapper: puts stdin into non-blocking raw mode when possible and
// restores it on destruction.
struct RawTerminal {
    struct termios saved_tty{};
    bool have_tty = false;
    int saved_flags = -1;

    RawTerminal() {
        saved_flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
        if (saved_flags >= 0) {
            ::fcntl(STDIN_FILENO, F_SETFL, saved_flags | O_NONBLOCK);
        }

        if (::isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &saved_tty) == 0) {
            have_tty = true;

            struct termios t = saved_tty;
            t.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
            t.c_cc[VMIN]  = 0;
            t.c_cc[VTIME] = 0;

            tcsetattr(STDIN_FILENO, TCSANOW, &t);
        }
    }

    ~RawTerminal() {
        if (have_tty) {
            tcsetattr(STDIN_FILENO, TCSANOW, &saved_tty);
        }

        if (saved_flags >= 0) {
            ::fcntl(STDIN_FILENO, F_SETFL, saved_flags);
        }
    }
};

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

void run_demo() {
    g_running.store(true, std::memory_order_relaxed);
    g_stop_requested = 0;

    install_signal_handlers();
    RawTerminal term;

    g_log_dir = choose_log_dir();
    if (!mkdir_p(g_log_dir)) {
        std::cerr << "[ERROR] Could not create final log directory: "
                  << g_log_dir << "\n";
        return;
    }

    const std::string imu_csv_path  = path_join(g_log_dir, kImuCsvFileName);
    const std::string nmea_csv_path = path_join(g_log_dir, kNmeaCsvFileName);

    {
        std::lock_guard<std::mutex> lock(g_imu_csv_mtx);
        g_imu_csv = std::make_unique<DurableAppendFile>(
            imu_csv_path,
            imu_csv_header()
        );
    }

    if (!g_imu_csv || !g_imu_csv->good()) {
        std::cerr << "[ERROR] IMU CSV logger failed to initialize\n";
        g_imu_csv.reset();
        return;
    }

    sh2_Hal_t hal = bno085_hal_create();
    if (sh2_open(&hal, nullptr, nullptr) != SH2_OK) {
        std::cerr << "[ERROR] sh2_open failed — check wiring and I2C address\n";
        g_imu_csv->sync_now();
        g_imu_csv.reset();
        return;
    }

    sh2_setSensorCallback(sensor_callback, nullptr);

    // IMPORTANT:
    // Keep this order. This matches the version that was working.
    // Do NOT call sh2_service() before these enable calls.
    enable_sensor(SH2_ACCELEROMETER,             2'500);
    enable_sensor(SH2_LINEAR_ACCELERATION,       2'500);
    enable_sensor(SH2_GYROSCOPE_CALIBRATED,      2'500);
    enable_sensor(SH2_MAGNETIC_FIELD_CALIBRATED, 10'000);
    enable_sensor(SH2_ROTATION_VECTOR,           2'500);
    enable_sensor(SH2_GAME_ROTATION_VECTOR,      2'500);

    std::thread service_thread([]() {
        while (keep_running()) {
            sh2_service();
        }
    });

    std::thread nmea_thread(
        nmea_recorder_thread,
        std::string(kNmeaPort),
        kNmeaBaud,
        nmea_csv_path
    );

    std::cout << "BNO085 durable per-measurement logging + NMEA recording\n";
    std::cout << "Press Esc to stop. Ctrl+C and SIGTERM are also handled.\n";
    std::cout << "IMU CSV:  " << imu_csv_path << "\n";
    std::cout << "NMEA CSV: " << nmea_csv_path << "\n";
    std::cout << "Sync policy: fdatasync every " << kSyncIntervalMs
              << " ms or " << kMaxRowsBeforeSync << " rows";
    if (kSyncEveryRow) {
        std::cout << " plus every row";
    }
    std::cout << "\n\n";

    while (keep_running()) {
        char ch = 0;
        const ssize_t n = ::read(STDIN_FILENO, &ch, 1);

        if (n == 1 && ch == '\x1b') {
            g_running.store(false, std::memory_order_relaxed);
            break;
        }

        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            // Do not exit just because stdin is unavailable; this is common
            // under systemd or cron.
        }

        sleep_interruptible_ms(10);
    }

    g_running.store(false, std::memory_order_relaxed);

    if (service_thread.joinable()) {
        service_thread.join();
    }

    if (nmea_thread.joinable()) {
        nmea_thread.join();
    }

    sh2_close();

    {
        std::lock_guard<std::mutex> lock(g_imu_csv_mtx);
        if (g_imu_csv) {
            g_imu_csv->sync_now();
            g_imu_csv.reset();
        }
    }

    std::cout << "\nShutdown complete. Logs are in: " << g_log_dir << "\n";
}
