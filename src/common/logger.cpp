// pimax_slam.pi.dll -- src/common/logger.cpp
//
// Global logger objects (all Logger members used by LOGx are inline, see logger.h).
// Dynamic initializers (.CRT$XCU order #23, #24):
//   0x180001850  kLogTag  = "S alg:"   (SSO bytes copied from 0x1803B16F4, atexit 0x1803A4920)
//   0x180001810  g_logger(kLogTag)      (atexit 0x1803A48A0 = ~ofstream 0x180097CD0 + ~string)
// The CRT table entry for kLogTag precedes the one for g_logger => definition order below.
//
// Logger::Init (0x1801692E0, chunk c14 draft common/logger_init.cpp) is defined here as well.
// In the binary it is a COMDAT emitted inside the headset-system object (i.e. the original was
// header-inline); see logger.h.
//
// Differences of Init to the LedObjectPoseEstimator version (0x180006CB0 there):
//   * file prefix "6DOF_" instead of "controller_" (path = dir + "/6DOF_" + time + ".txt");
//   * uses std::filesystem (MSVC: __std_fs_* imports, narrow->wide via _Convert_narrow_to_wide),
//     not boost::filesystem;
//   * old files are removed WITHOUT an exists() check (fs::remove(path, ec) directly).
// upstream status: new (Pimax common code).
#include "common/logger.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace pimax {

static const std::string kLogTag = "S alg:";  // 0x18046A138
Logger g_logger(kLogTag);                      // 0x18046A000

// 0x1801692E0
void Logger::Init(const std::string& dir)
{
    std::error_code ec;
    if (!fs::exists(fs::path(dir), ec) || !fs::is_directory(fs::path(dir), ec))   // status(...) type
        return;                                                                       // <= not_found / == directory

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm = *std::localtime(&now);   // _localtime64
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");   // 0x180163320
    std::string path = dir + "/6DOF_" + ss.str() + ".txt";
    m_file.open(path, std::ios::out | std::ios::app);   // filebuf::open(name, 10, _SH_DENYNO)

    std::vector<std::string> logs;
    for (fs::directory_iterator it{fs::path(dir)}, end; it != end; ++it) {
        if (fs::status(it->path(), ec).type() != fs::file_type::regular)
            continue;
        std::string name = it->path().filename().string();   // 0x18016F0F0
        if (name.find("6DOF_") != std::string::npos)
            logs.push_back(it->path().string());              // 0x180163850 on growth
    }
    std::sort(logs.begin(), logs.end(), [](const std::string& a, const std::string& b) {   // 0x180165620
        return fs::last_write_time(fs::path(a)) < fs::last_write_time(fs::path(b));        // 0x180167950
    });
    while (logs.size() > 5) {
        fs::remove(fs::path(logs.front()), ec);   // 0x1803650A0
        logs.erase(logs.begin());
    }
}

}  // namespace pimax
