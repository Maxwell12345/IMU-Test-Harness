#include "bno085_hal.hpp"

#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr uint8_t     BNO085_I2C_ADDR  = 0x4A;
static constexpr const char* I2C_BUS_PATH     = "/dev/i2c-1";

/// Startup delay after opening the bus to let the sensor finish its POR sequence.
static constexpr unsigned int STARTUP_DELAY_US = 100'000; // 100 ms

// ---------------------------------------------------------------------------
// Module-level state
//
// The SH2 driver is a singleton, so one global fd is safe here.
// ---------------------------------------------------------------------------

static int g_fd = -1;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * @brief Returns the current CLOCK_MONOTONIC time in microseconds.
 * @return uint32_t Microseconds since an arbitrary epoch (wraps after ~71 min).
 */
static uint32_t now_us() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint32_t>(
        static_cast<uint64_t>(ts.tv_sec) * 1'000'000ULL +
        static_cast<uint64_t>(ts.tv_nsec) / 1'000ULL
    );
}

// ---------------------------------------------------------------------------
// HAL callbacks
// ---------------------------------------------------------------------------

/**
 * @brief Opens /dev/i2c-1 and configures the BNO085 slave address.
 * @param[in] self HAL context pointer (unused; SH2 driver passes it automatically).
 * @return SH2_OK on success, SH2_ERR on failure.
 */
static int hal_open(sh2_Hal_t* self) {
    g_fd = open(I2C_BUS_PATH, O_RDWR);
    if (g_fd < 0) return SH2_ERR;

    if (ioctl(g_fd, I2C_SLAVE, BNO085_I2C_ADDR) < 0) {
        close(g_fd);
        g_fd = -1;
        return SH2_ERR;
    }

    usleep(STARTUP_DELAY_US);
    return SH2_OK;
}

/**
 * @brief Closes the I2C bus file descriptor.
 * @param[in] self HAL context pointer (unused).
 */
static void hal_close(sh2_Hal_t* self) {
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}

/**
 * @brief Reads one SHTP packet from the BNO085 over I2C.
 *
 * The BNO085 presents the current SHTP packet starting from the 4-byte header
 * on every I2C read transaction.  This function performs two transactions:
 *  1. A 4-byte read to obtain the packet length from the SHTP header.
 *  2. A full-length read (resetting the device's internal read pointer) to
 *     retrieve the complete packet into @p pBuffer.
 *
 * @param[in]  self    HAL context pointer (unused).
 * @param[out] pBuffer Destination buffer; must be at least @p len bytes.
 * @param[in]  len     Maximum bytes the caller can accept.
 * @param[out] t_us    Timestamp of the read in microseconds (CLOCK_MONOTONIC).
 * @return Number of bytes placed in @p pBuffer, or 0 if no valid packet was available.
 */
static int hal_read(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us) {
    // --- Transaction 1: probe the 4-byte SHTP header ---
    uint8_t header[4] = {};
    if (::read(g_fd, header, sizeof(header)) != static_cast<ssize_t>(sizeof(header))) {
        return 0;
    }

    *t_us = now_us();

    // Lower 15 bits of bytes [1:0] encode the total packet length including header.
    const uint16_t packet_len =
        (static_cast<uint16_t>(header[1] & 0x7F) << 8) | header[0];

    if (packet_len == 0 || packet_len > static_cast<uint16_t>(len)) {
        return 0;
    }

    // --- Transaction 2: read full packet (device resets read pointer on new START) ---
    const ssize_t received = ::read(g_fd, pBuffer, packet_len);
    if (received <= 0) {
        return 0;
    }

    return static_cast<int>(received);
}

/**
 * @brief Writes an SHTP packet to the BNO085 over I2C.
 * @param[in] self    HAL context pointer (unused).
 * @param[in] pBuffer Data to transmit.
 * @param[in] len     Number of bytes to transmit.
 * @return Number of bytes written, or SH2_ERR on failure.
 */
static int hal_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len) {
    const ssize_t result = ::write(g_fd, pBuffer, len);
    return (result < 0) ? SH2_ERR : static_cast<int>(result);
}

/**
 * @brief Returns the current time in microseconds.
 * @param[in] self HAL context pointer (unused).
 * @return uint32_t Microseconds since an arbitrary epoch (CLOCK_MONOTONIC).
 */
static uint32_t hal_getTimeUs(sh2_Hal_t* self) {
    return now_us();
}

// ---------------------------------------------------------------------------
// Public factory
// ---------------------------------------------------------------------------

sh2_Hal_t bno085_hal_create() {
    sh2_Hal_t hal{};
    hal.open      = hal_open;
    hal.close     = hal_close;
    hal.read      = hal_read;
    hal.write     = hal_write;
    hal.getTimeUs = hal_getTimeUs;
    return hal;
}
