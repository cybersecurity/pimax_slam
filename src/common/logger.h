// pimax_slam.pi.dll -- src/common/logger.h
// Leveled printf-style logger -- pimax_slam.pi.dll variant.
// Global instance g_logger at 0x18046A000 (first object in .data), tag "S alg:".
//
// Same class as ../LedObjectPoseEstimator/src/common/logger.h (identical layout: +0 m_enabled,
// +4 m_level, +8 m_tag, +40 m_file, +304 m_console; sizeof == 0x138), with these differences
// (verified against the binary):
//   * All members used by the LOGx macros are defined INLINE in this header: in this DLL
//     Debug/Info/Warn/Error and TimeString() are COMDAT functions that the linker placed in the
//     first object that used them (ceres_backend_interface.obj), not in logger.obj:
//        Debug 0x18000C120, Error 0x18000C2C0, Info 0x18000F500, Warn 0x18000F6A0,
//        TimeString 0x18000F310.  logger.cpp only defines kLogTag and g_logger.
//   * The Write() helper is inlined into each level function (each one is a separate 406-byte
//     function containing the whole body).
//   * After writing a line to the file the stream is FLUSHED (std::ostream::flush, called after
//     the TimeString() temporary has been destroyed => a separate statement).  The sibling DLL
//     reconstruction has no flush.
//   * TimeString() is a non-static member (called with `this` in rcx, return slot in rdx).
//   * Tag is "S alg:" (sibling: "C alg").
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

namespace pimax {

class Logger {
public:
    enum Level { kDebug = 1, kInfo = 2, kWarn = 3, kError = 4 };

    // Inlined into the dynamic initializer 0x180001810 (m_tag copy via std::string copy-ctor
    // 0x180008BF0, std::ofstream default ctor 0x180097B60, m_console = false).
    // m_enabled/m_level are constant-initialised in .data (0x18046A000 = {1, 3}).
    explicit Logger(const std::string& tag) : m_tag(tag) {}

    // 0x1801692E0 (defined in logger.cpp).  Opens <dir>/6DOF_<%Y_%m_%d_%H_%M_%S>.txt and keeps
    // only the 5 newest 6DOF_* files.  Called from the Headset init path (0x1801663F0).
    // NOTE: the binary emits it as a COMDAT inside the headset-system object, i.e. it was most
    // likely an inline member of this header; it is kept out of line here (integration
    // decision, see notes/integration_A1.md) so that not every TU pulls in <filesystem>.
    void Init(const std::string& dir);

    // 0x18000C120
    void Debug(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        Write(kDebug, "[debug]", fmt, args);
        va_end(args);
    }

    // 0x18000F500
    void Info(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        Write(kInfo, "[info]", fmt, args);
        va_end(args);
    }

    // 0x18000F6A0
    void Warn(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        Write(kWarn, "[warn]", fmt, args);
        va_end(args);
    }

    // 0x18000C2C0
    void Error(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        Write(kError, "[error]", fmt, args);
        va_end(args);
    }

    // Inlined into the static tracker-system destructor 0x180167320 (interface chunk):
    //   if (_Myfile && !filebuf::close()) setstate(failbit)   == std::ofstream::close()
    void Close() {
        if (m_file.is_open())
            m_file.close();
    }

private:
    // Inlined into each of the four level functions.
    void Write(int level, const char* levelName, const char* fmt, va_list args) {
        if (!m_enabled || m_level > level)
            return;
        char message[256];  // rsp+0x150
        char line[256];     // rsp+0x50
        // vsnprintf (UCRT inline: __stdio_common_vsprintf(opts|_CRT_INTERNAL_PRINTF_STANDARD_
        // SNPRINTF_BEHAVIOR, ...), result clamped to -1 by the CRT inline itself).
        int n = vsnprintf(message, sizeof(message), fmt, args);
        int m = snprintf(line, 0xFF, "%s", message);  // out-of-line inline snprintf 0x180016BF0
        if (n < 0 || m < 0)
            return;
        if (m_console)
            printf("%s %s %s", m_tag.c_str(), levelName, line);  // printf COMDAT 0x180016B90
        if (m_file.is_open()) {  // filebuf::_Myfile at this+0xB0
            m_file << TimeString().c_str() << levelName << line;
            m_file.flush();
        }
    }

    // 0x18000F310  "Y_M_D_h_m_s" (no zero padding), e.g. "2024_3_7_9_5_1".
    std::string TimeString() {
        std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm* tm = std::localtime(&t);
        std::stringstream ss;
        ss << tm->tm_year + 1900 << "_" << tm->tm_mon + 1 << "_" << tm->tm_mday << "_" << tm->tm_hour << "_"
           << tm->tm_min << "_" << tm->tm_sec;
        return ss.str();
    }

    bool m_enabled = true;    // +0
#ifdef PIMAX_SLAM_TEST_LOG_LEVEL   // test builds only (CMake PIMAX_SLAM_TEST_LOG_LEVEL), see test/
    int m_level = PIMAX_SLAM_TEST_LOG_LEVEL;
#else
    int m_level = kWarn;      // +4   (=3: only Warn/Error are written by default)
#endif
    std::string m_tag;        // +8   "S alg:"
    std::ofstream m_file;     // +40  (filebuf at +48, _Myfile at +176)
    bool m_console = false;   // +304 never enabled in the shipped build

    friend struct LoggerLayout;
};

// sizeof == 0x138; offsets from the inlined level functions / ctor (c00).
struct LoggerLayout {
    static_assert(offsetof(Logger, m_enabled) == 0, "Logger::m_enabled");
    static_assert(offsetof(Logger, m_level) == 4, "Logger::m_level");
    static_assert(offsetof(Logger, m_tag) == 8, "Logger::m_tag");
    static_assert(offsetof(Logger, m_file) == 40, "Logger::m_file");
    static_assert(offsetof(Logger, m_console) == 304, "Logger::m_console");
};
static_assert(sizeof(Logger) == 0x138, "sizeof(Logger)");

extern Logger g_logger;  // 0x18046A000

}  // namespace pimax

#define LOGD(...) ::pimax::g_logger.Debug(__VA_ARGS__)
#define LOGI(...) ::pimax::g_logger.Info(__VA_ARGS__)
#define LOGW(...) ::pimax::g_logger.Warn(__VA_ARGS__)
#define LOGE(...) ::pimax::g_logger.Error(__VA_ARGS__)
