# Inu-Display — Agent Root Instruction File

> This file is the authoritative context document for AI agents operating on the inu-display codebase.
> Read this file in full before making any code changes, generating files, or answering questions about the project.
> When in doubt about a pattern or convention, defer to what is described here over what you infer from partial file reads.

---

## Project Background

This project uses a GPS and inertial measurement unit (IMU) to determine the location of a moving vehicle, and the software needs to be able to use inputs from both the GPS and the IMU to predict where the vehicle is. 

**Operational context:** The system is intended for deployment in potentially **offline environments**. This has direct implications for every dependency, installer, and network assumption in the codebase. Do not suggest solutions that require internet access at runtime. Do not suggest cloud services, telemetry, or update mechanisms that phone home.

**Deployment target:** Linux (x86_64 primary, ARM64 secondary) and Windows (verison 10 or higher).  

**Version control:** GitHub, including air-gapped military instances at `https://github.com/B-Atkinson/inu-display.git`.

---

## Software Architecture Overview

The software component of this will be done in C++ (version 20). An extended Kalman filter will be used to predict the state of the vehicle using inputs from the IMU to correct GPS error. The purpose of the main software is to simply access the measurements provided by the IMU. This software will read the IMU-produced data through a Raspberry Pi. The Kalman filter needs linear accelearation (without gravity), rotational acceleration, and magnometer readings.

## Hardware Architecture Overview

This project will test multiple IMU models, but the first is an `Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085 (BNO080) - STEMMA QT / Qwiic`. The initial plan is for the IMU (regardless of model) to connect to a Raspberry Pi 5 through the Pi's GPIO pins. From initial reading of documentation, IMU can communicate with the Pi through I2C, UART-SHTP, or UART-RVT. 

### Key Components

| Component                   | Language | Role                                                                   |
|-----------------------------|---|------------------------------------------------------------------------|
| Control Software            | C++ | Core processing engine, receives IMU measurements through Raspberry Pi |

### Hardware Documentation

All pertinent documentation for IMUs being used in this project can be found inside the 'references' directory inside the project root. Documentation provided by manufacturer will always include the part/model number in the document name as it is found in the host file system (i.e. docs pertatining to the 'BNO085' IMU will have either 'BNO085', 'bno085') or will use some form of wildcard notation for the numbers (so 'BNO08X' could apply to 'BNO085' as well as 'BNO080').

---

## Technology Stack

### C++ Backend

- **Standard:** C++20 (`CMAKE_CXX_STANDARD 20`, extensions OFF)
- **Build system:** CMake 3.27+ with Ninja generator, vcpkg manifest mode for all dependencies
- **Concurrency:** Intel TBB, Boost.Asio
- **Math/CV:** Eigen3
- **Serialization/Config:** yaml-cpp, Boost.Log, SQLiteCpp, unofficial-sqlite3
- **Testing:** Google Test + Google Mock (see `common.hpp`)

### Frontend

None planned yet, initial start will be printing sensor values to stdout.


### Build Targets

| Target | Toolchain | CMake Flag |
|---|---|---|
| Windows x64 | MSVC | `CRUSADER_APPLICATION_BUILD_TYPE_STANDALONE` |
| Linux x86_64 | GCC/Clang | `CRUSADER_APPLICATION_BUILD_TYPE_LINUX` |
| Linux ARM64 | aarch64-linux-gnu cross-compiler | Custom `arm64-linux-cross` vcpkg triplet |

**Cross-compilation note (ARM64):** vcpkg autotools packages require both `--host` (target architecture) and `--build` (build machine) flags. Supply both via `VCPKG_MAKE_BUILD_TRIPLET`. Note: autotools `--host`/`--build` semantics are **inverted** relative to CMake's usage of "host" and "build" — do not conflate them.

### Deployment

Because the target software is being developed on a Ubuntu x86-64 host but will be run on a Raspberry Pi with an ARM-based processor, this software will need to be packaged into a Docker container to save headaches with cross-platform. To support this, a well-defined VCPKG manifest will be needed to facilitate building of the container images.

## Documentation Template & Rules

All source files in this project follow a **standard file header**. Every new `.cpp`, `.hpp`, `.h`, `.ts`, `.tsx`, `.py`, `.sh`, and `.ps1` file must include this header, adapted to the file's language comment syntax.

### C / C++ Header Template

```cpp
/******************************************************************************
 * File:             <filename.ext>
 *
 * Author:           <Rank LastName FirstName M.>
 * Organization:     Marine Corps Software Factory
 * Created On:       <MM/DD/YY H:MMam/pm>
 * Description:      Briefly describe the purpose of this file and its role
 *                   within the project. Mention key functions or classes it
 *                   contains.
 *
 ******************************************************************************/
```

### Shell Script Header Template

```bash
#!/bin/bash
################################################################################
# File:         <filename.sh>
#
# Author:       <Rank LastName FirstName M.>
# Organization: Marine Corps Software Factory
# Created On:   <MM/DD/YY>
# Description:  Briefly describe what this script does.
#
################################################################################
```

### TypeScript / React Header Template

```ts
/******************************************************************************
 * File:             <filename.ts|tsx>
 *
 * Author:           <Rank LastName FirstName M.>
 * Organization:     Marine Corps Software Factory
 * Created On:       <MM/DD/YY>
 * Description:      Briefly describe the purpose of this file.
 *
 ******************************************************************************/
```

### C/C++ Method Documentation

All non-trivial C++ methods — public, protected, and private — must have a Doxygen-format doc block in the header file (`.hpp` / `.h`). Trivial getters/setters consisting of a single return statement are exempt but encouraged. The format follows the style established in `FurunoConnectManager.hpp` and is described in full below.

#### Template

```cpp
/**
 *
 * @brief   <One or more sentences giving a clear, standalone description of what the method
 *           does. Include behavioral detail sufficient for a caller to use it correctly without
 *           reading the implementation. Do not say "this method" — describe the action directly.>
 *
 * @param [in]     <paramName>   <Description of the input. Explain valid values, units, and any
 *                                constraints. If the param drives branching behavior, describe each
 *                                branch outcome.>
 * @param [out]    <paramName>   <Description of what will be written into this parameter on return.
 *                                Describe the state of the object after a successful call.>
 * @param [in,out] <paramName>   <Description of what the method reads from this parameter on entry
 *                                and what it writes back on return.>
 *
 * @return  <Description of the return value, its type, and the range or meaning of possible values.
 *           If the method returns void, this tag must still be present and the description must be
 *           left empty — do not omit the tag.>
 *
 * @remarks <Optional. Include side effects, preconditions the caller is responsible for,
 *           postconditions guaranteed by the method, or behavioral notes that do not fit
 *           elsewhere.>
 *
 * @throws  <ExceptionType>   <Exact condition that causes this exception to propagate. Include
 *                             which internal call originates it if that is not obvious from the
 *                             method's own body.>
 * @throws  <ExceptionType>   <Additional exception. One @throws tag per exception type per
 *                             distinct throw condition.>
 */
ReturnType MethodName(ParamType param);
```

#### Rules

1. **`@brief` is mandatory on every documented method.** It must be a complete, informative description — not a restatement of the method name. `@brief Connects to radar.` is not acceptable. Describe *how* and *what happens as a result*.

2. **`@param` direction tags are mandatory.** Every parameter must have one of `[in]`, `[out]`, or `[in,out]`. The tag determines how the agent and the reader reason about ownership and mutation.
    - `[in]` — method reads from this parameter; caller retains ownership of the value.
    - `[out]` — method writes to this parameter; its value on entry is irrelevant.
    - `[in,out]` — method reads the entry value and modifies the object in place.

3. **`@return` is mandatory on every documented method, including `void` methods.** For `void` methods, the tag must appear with nothing after it:
   ```cpp
   * @return
   ```
   Do not omit the tag from a `void` method. This is an explicit project convention so that a reader scanning the doc block can confirm the absence of a return value is intentional, not an oversight.

4. **`@throws` is mandatory for every exception that is not fully caught and handled internally.** "Fully handled" means caught, dealt with, and not re-thrown. If an exception can propagate to the caller under any condition, it must be documented. Specific rules:
    - List each distinct throw condition as a separate `@throws` tag, even if the same exception type is thrown under multiple conditions.
    - If an exception originates in a called sub-method rather than the body of this method directly, the `@throws` description must name that sub-method as the source.
    - If a method explicitly makes no exception guarantee (i.e., is `noexcept`), document that in `@remarks` and omit `@throws`.
    - Do not document exceptions that are caught and swallowed internally unless swallowing them has a meaningful behavioral side effect worth noting in `@remarks`.

5. **`@remarks` is optional but expected** whenever a method has a precondition the caller must satisfy (e.g., a required call sequence, a required environment variable, a mutex that must not already be held), a notable side effect, or a behavioral subtlety not captured by the other tags.

6. **No undocumented public methods.** If adding a method to a public interface, the doc block is required before the PR is mergeable. Private methods should be documented unless they are trivially obvious from their name and signature alone.

7. **Doc blocks live in the header, not the `.cpp` file.** The header is the contract. Implementation comments in the `.cpp` may supplement but do not replace the header doc block.

#### Examples

**Non-void return, mixed param directions, multiple exceptions:**
```cpp
/**
 *
 * @brief   Sets the host name within the Furuno SDK to the IPv4 provided in `ipAddress`, then
 *          uses the SDK to attempt to connect to Furuno radars producing ARPA broadcasts on the
 *          network. Returns a vector of radar information objects for each logical radar the SDK
 *          successfully connects to.
 *
 * @param [in] ipAddress  IPv4 address of the host network interface the Furuno SDK should use
 *                        to reach the radar. Must be a valid dotted-decimal IPv4 string within
 *                        the 172.31.0.0/16 subnet.
 * @param [in] dualOn     When true, the physical radar is split into two logical radars, each
 *                        returned as a separate element in the result vector. When false, one
 *                        element is returned per physical radar.
 *
 * @return  Vector of RadarInformation objects for each logical radar successfully connected to.
 *          Contains two elements per physical radar when dualOn is true, one element otherwise.
 *          Empty if no radars are found.
 *
 * @remarks Requires layer-2 network routability to the Furuno radar. The host interface
 *          identified by ipAddress must be on the same broadcast domain as the radar.
 *
 * @throws  FurunoLicensingConnectException    If the vendor.txt license file cannot be found
 *                                             at the path returned by GetLicenseFilePath(), or
 *                                             if the file cannot be opened and read, within
 *                                             LoadLicenseIntoRadar().
 * @throws  FurunoUnknownErrorPresentException If the Furuno SDK fails to construct a License
 *                                             object from the contents of vendor.txt within
 *                                             LoadLicenseIntoRadar().
 * @throws  InvalidIPException                 If ipAddress is not a valid IPv4 address.
 * @throws  FurunoNAVNETIPException            If the SDK rejects the host interface address
 *                                             when setting the network target within
 *                                             GiveHostInterfaceIpToRadar().
 */
static std::vector<RadarInformation> AttemptConnectFurunoLinux(const std::string &ipAddress, bool dualOn);
```

**Void return, no exceptions:**
```cpp
/**
 *
 * @brief   Registers a new frontend WebSocket connection in the active connection set and
 *          initiates a backend WebSocket relay session for that client.
 *
 * @param [in] req   HTTP upgrade request that initiated the WebSocket handshake. Used to
 *                   extract client metadata for logging.
 * @param [in] conn  Shared pointer to the newly established frontend WebSocket connection.
 *                   Added to m_connections under m_connectionMutex.
 *
 * @return
 *
 * @remarks Acquires m_connectionMutex. Must not be called while m_connectionMutex is
 *          already held on the same thread.
 *
 */
void handleNewConnection(const HttpRequestPtr &req, const WebSocketConnectionPtr &conn) override;
```

### General Documentation Rules

1. **Every file gets a header.** No exceptions. If generating a new file, always include the full header block.
2. **Author format follows this convention:** <First> <MI> <Last>. Example: `Brian R. Atkinson`
3. **Organization is always:** `Marine Corps Software Factory`
4. **Descriptions must be substantive.** Do not leave the description as a placeholder. Describe what the file actually does.
5. **Include guards in C++ headers:** Use `#ifndef INU_DISPLAY_<FILENAME>_HPP` / `#define` / `#endif` format. Do not rely solely on `#pragma once`, though `#pragma once` may be included as a secondary guard after the `#ifndef`.
6. **Namespaces:** Drogon plugin classes live in `namespace my_plugin`. Do not introduce new top-level namespaces without discussion.
7. **Logging:** Use the compile-time logging macros controlled by `ALLOW_DEBUG_LOGGING`, `ALLOW_INFO_LOGGING`, `ALLOW_WARNING_LOGGING`, `ALLOW_ERROR_LOGGING`, and `ALLOW_WINDOW`. Do not use raw `std::cout` or `printf` for application logging.
8. **Cross-platform guards:** Use `#ifdef` / `#if defined(_WIN32)` / `#elif defined(__linux__)` for platform-specific code blocks. Never write Linux-only or Windows-only code in a shared file without a platform guard.
9. **No internet-dependent code:** Do not introduce dependencies, update checks, or telemetry that require network access at runtime. The deployment environment may be fully air-gapped.
10. **CMake dependency additions:** All new C++ dependencies must be added to the vcpkg manifest (`vcpkg.json`) and found via `find_package()` in `CMakeLists.txt`. Do not manually set include/lib paths except for the Furuno SDK which has no vcpkg port.
11. **Constructors:** Constructors do not need Doxygen style documentation.
---

## C++ Naming and File Conventions

### File Extensions

- C++ implementation files use `.cpp`.
- C++ header files always use `.hpp`.
- Do not introduce new project headers using `.h`. Legacy `.h` files may exist, but new headers should use `.hpp` unless explicitly directed otherwise.

### Type Naming

- Class, struct, enum, and type-alias names use PascalCase.
- Prefer precise responsibility-focused names over broad manager names when a class has a narrow role.
  - Use `GpsReceiver` for the UDP multicast GPS receive component.
  - Do not use `GPS_Manager` for the multicast-only receiver because it does not own the physical GPS hardware or serial interface.
- Use `Gps` rather than `GPS` inside PascalCase type names unless an existing type or external API already requires a different spelling.
  - Preferred: `GpsReceiver`, `GpsUpdate`, `GpsReceiverStats`.
  - Avoid: `GPS_Manager`, `GPS_Update`, `GPSReceiverStats`.

### Class Member Naming

- Public non-static class members begin with `m`.
  - Example: `mMemberName`.
- Public static class members begin with `ms`.
  - Example: `msMemberName`.
- Private non-static class members begin with `m_`.
  - Example: `m_memberName`.
- Private static class members begin with `m_s`.
  - Example: `m_sMemberName`.
- Apply these prefixes consistently in generated code, story text, class details, and documentation.

### Method Naming and API Intent

- Prefer method names that describe runtime behavior accurately.
- Use `enqueue...` for asynchronous persistence methods that place work on a queue.
  - Preferred: `enqueueGpsUpdate`, `enqueueImuMeasurement`, `enqueueKalmanOutput`.
  - Avoid naming asynchronous queueing methods `write...` if the method does not synchronously write to SQLite on the caller's thread.
- Use explicit lifecycle methods such as `start()` and `stop()` for classes that own worker threads or sockets.
- Worker threads must not be detached. They must be joinable and cleanly joined during shutdown.

---

## GitHub Story / Issue Template

Engineering stories are posted as GitHub issues and should be generated in Markdown using the structure below. Preserve the section names and ordering unless the user explicitly requests a different format.

```markdown
## Story Details ##
- <bulletized list of story or feature details>

## Due Outs ##
- <bulletized list of due outs and acceptance criteria>
- Implement unit tests with 80% code coverage for all methods
- Implement documentation for the class and its methods using the custom doxygen styled template

## Class Details ##
### Public Class Members ###
Name: <member name>
Type: <member type>
Description: <briefly describes what purpose this member serves>

---

### Public Class Method Signatures ###

```cpp
<method signature>
```

Parameters:
`<parameter name>` `<parameter type>` <parameter description>

Returns:
`<returned object name>` `<returned type>` <description of returned object>

Exceptions:
`<exception type>` <description of what causes the exception to be thrown>

Behaviors:
- <bulletized list of behavioral details>

---

### Private Class Members ###
Name: <member name>
Type: <member type>
Description: <briefly describes what purpose this member serves>
---

### Private Class Method Signatures ###

```cpp
<method signature>
```

Parameters:
`<parameter name>` `<parameter type>` <parameter description>

Returns:
`<returned object name>` `<returned type>` <description of returned object>

Exceptions:
`<exception type>` <description of what causes the exception to be thrown>

Behaviors:
- <bulletized list of behavioral details>
```

### Story Formatting Rules

- Use horizontal rules (`---`) between each member entry and between each method section. This makes long GitHub issues easier to scan.
- Use fenced `cpp` code blocks for method signatures.
- Use backticks around parameter names, return object names, return types, member names, and exception types.
- Public and private member names in stories must follow the project class-member naming convention.
- For classes with no public members, write:
  - `Name: None`
  - `Type: N/A`
- Story due outs must always include unit testing with 80% code coverage and documentation using the custom Doxygen style template.
- When a story is still being reviewed, provide a preview in chat. Once approved, generate a `.md` file for the user.
- Generated story files should use the class name and `_story.md` suffix when practical.
  - Example: `GpsReceiver_story.md`.

---

## GPS / IMU / Database Data Flow Context

The project is being organized around focused components instead of one monolithic manager. The expected near-term components are `GpsReceiver`, `ImuManager`, and `DatabaseManager`, with a Kalman filter component invoked from the IMU processing path.

### GpsReceiver

- `GpsReceiver` receives GPS update messages over UDP multicast on the private LAN.
- The multicast group IP is `239.1.2.3`.
- The multicast group port is `3636`.
- `GpsReceiver` owns the UDP multicast socket used to receive GPS update messages.
- `GpsReceiver` owns a receive thread containing the bound UDP multicast socket.
- The receive thread must not be detached and must be cleanly joined during shutdown.
- `GpsReceiver` is not responsible for interacting with the physical GPS device, serial interface, or raw GPS hardware protocol.
- Another object in the application will connect to the physical GPS interface, receive serialized GPS data, and publish GPS update payloads to the multicast group.
- `GpsReceiver` consumes the multicast payloads, deserializes/maps each valid payload into a `GpsUpdate` object, validates it, and then invokes a configured callback with the parsed `GpsUpdate`.
- The callback passed to the `GpsReceiver` constructor receives the parsed `GpsUpdate`; the callback is not responsible for parsing serialized payloads.
- The intended callback provider is `ImuManager`.
- `GpsReceiver` should timestamp each received message with:
  - a monotonic receive timestamp for local ordering, stale-data checks, and latency calculations;
  - a wall-clock receive timestamp for logging, database recording, and correlation with other system data.
- The GPS payload timestamp should be extracted from the serialized payload when available.
- `GpsReceiver` should reject malformed, incomplete, stale, or otherwise invalid GPS messages according to validation logic.
- `GpsReceiver` should expose diagnostic counters for received, accepted, rejected, malformed, stale, and callback-failed messages.
- `GpsReceiver` must not directly own `ImuManager`, `DatabaseManager`, or the Kalman filter class.

### ImuManager

- `ImuManager` receives IMU measurements from the Raspberry Pi path through an IMU callback.
- `ImuManager` receives GPS updates from `GpsReceiver`, most likely through a public update method or callback-compatible function.
- `ImuManager` maintains the latest accepted GPS update internally and must protect it with thread-safe access.
- `ImuManager` processes IMU measurements into a form readily usable by the Kalman filter.
- `ImuManager` passes processed IMU data and the latest appropriate GPS data to the Kalman filter callback.
- Avoid heavy, blocking, or long-running work directly inside the IMU measurement callback when practical. Prefer queueing the measurement and processing it from an internal worker thread when timing jitter matters.
- `ImuManager` should use stale-data checks before combining GPS data with IMU measurements.
- `ImuManager` may be constructed or owned via `boost::shared_ptr` where required by surrounding application patterns.

### DatabaseManager

- `DatabaseManager` owns the SQLite database connection.
- `DatabaseManager` owns a writer thread responsible for SQLite persistence.
- The writer thread must not be detached and must be cleanly joined during shutdown.
- Public persistence methods should enqueue records for asynchronous database writes rather than performing SQLite operations on caller threads.
- All SQLite writes should occur on the `DatabaseManager` writer thread.
- `DatabaseManager` should provide enqueue methods for GPS update records, IMU measurement records, and Kalman filter output records.
- `DatabaseManager` should batch queued records into SQLite transactions where practical.
- `DatabaseManager` should initialize or validate required database tables if they do not already exist.
- `DatabaseManager` should flush queued records during shutdown unless an unrecoverable database error occurs.
- `DatabaseManager` should expose diagnostic counters for queued records, written records, failed writes, dropped records, and current queue depth.
- `DatabaseManager` must not directly own `GpsReceiver`, `ImuManager`, or the Kalman filter class.

### Timestamp Guidance

- Use `std::chrono::steady_clock::time_point` for local monotonic receive timing, elapsed-time calculations, stale-data checks, ordering, and latency calculations.
- Use `std::chrono::system_clock::time_point` for wall-clock UTC timestamps, database correlation, logs, and timestamps that represent real-world calendar time.
- Avoid using `std::chrono::high_resolution_clock` for receive timing because it is not guaranteed to be steady on all platforms.
- A `GpsUpdate` should distinguish between local receive time and the GPS payload measurement time.
- For database storage, prefer explicit integer timestamp fields such as microseconds or nanoseconds since Unix epoch for wall-clock time. Monotonic timestamps should be stored as offsets from a recorded session start if replay across process runs is required.

### Threading and Shutdown Guidance

- Classes that own sockets, queues, or worker threads should expose explicit `start()` and `stop()` methods.
- Constructors should configure and validate dependencies but should avoid starting worker threads unless there is a strong reason to do so.
- Destructors should call `stop()` when the object is still running, and destructors must not throw.
- Shared state must be protected with mutexes, atomics, and condition variables where practical.
- Avoid holding mutexes while invoking externally supplied callbacks.
- Callback exceptions should be caught at thread boundaries and should not escape worker-thread entry points.
