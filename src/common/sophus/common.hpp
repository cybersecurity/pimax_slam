// pimax_slam.pi.dll -- src/common/sophus/common.hpp
//
// Pimax fork of Sophus/common.hpp (only the parts present in the binary).
// Reconciled from c03 (0x18002E970 FormatStream<1 arg>, 0x18002EDE0 FormatString, 0x180037490
// ensure handler) and c07 (0x1800B7BA0 / 0x1800B8020 / 0x1800B8490 FormatStream<3/1/2 args>,
// 0x1800B8900 FormatString, 0x1800CBC40 defaultEnsure<3 args>).  c03's single-argument
// `FormatStream` and `ensureFailed` are the 1-argument instantiations of the templates below.
//
// Pimax changes vs upstream Sophus:
//   * placeholders are "{}" (with "{{" escape and "{.N}" fixed/precision; digits are read from
//     spec[1] on even without a '.') instead of '%';
//   * the ensure handler prints "mlog ensure failed in function ..." (upstream "Sophus ensure
//     failed ...").
#pragma once

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include <Eigen/Core>

namespace Sophus {

/// Upstream Sophus constants (epsilon 1e-10 for double: 0x1803AF870; log uses eps^2 = 1e-20,
/// 0x1803AF860).
template <class Scalar>
struct Constants {
  static Scalar epsilon() { return Scalar(1e-10); }
  static Scalar pi() { return Scalar(M_PI); }
};

template <>
struct Constants<float> {
  static float epsilon() { return static_cast<float>(1e-5); }
  static float pi() { return static_cast<float>(M_PI); }
};

}  // namespace Sophus

namespace Sophus {
namespace details {

// Following: http://stackoverflow.com/a/22759544
template <class T>
class IsStreamable {
 private:
  template <class TT>
  static auto test(int)
      -> decltype(std::declval<std::stringstream&>() << std::declval<TT>(),
                  std::true_type());

  template <class>
  static auto test(...) -> std::false_type;

 public:
  static bool const value = decltype(test<T>(0))::value;
};

template <class T>
class ArgToStream {
 public:
  static void impl(std::stringstream& stream, T&& arg) {
    stream << std::forward<T>(arg);
  }
};

inline void FormatStream(std::stringstream& stream, char const* text) {
  stream << text;
  return;
}

// Instantiated in this object (via SO3::exp's SOPHUS_ENSURE in
// common/sophus/so3ex_base.h:303, "SO3::exp failed! omega: {}, real: {}, img: {}"):
//   0x1800b7ba0  FormatStream<Eigen::Transpose<const Eigen::Vector3d>, double&, double&>
//   0x1800b8490  FormatStream<double&, double&>
//   0x1800b8020  FormatStream<double&>
// (the 1-argument instantiation used by other call sites lives at 0x18002e970.)
//
// Pimax change vs. upstream Sophus: placeholder is "{...}" instead of '%':
//   "{{"     -> prints a single '{' (the second brace is consumed)
//   "{}"     -> prints the argument
//   "{.N}"   -> stream << std::fixed << std::setprecision(N), then the argument
//   "{xN}"   -> (x != '.') only setprecision(N) -- digits are always read from
//               fmt[1] on, so "{3}" sets NO precision (quirk, preserved)
//   "{" without a closing '}' -> the '{' is printed literally.
// std::fixed / precision are never reset (sticky for following args).
// TODO(verify): exact source shape of the brace parsing; the control flow below
// reproduces the binary (empty std::string declared before the "text[1]=='}'"
// shortcut, fmt.assign(text+1), find('}'), fmt = fmt.substr(0, pos),
// text += fmt.size(), recursion on text + 2).
template <class T, typename... Args>
void FormatStream(std::stringstream& stream, char const* text, T&& arg,
                  Args&&... args) {
  static_assert(IsStreamable<T>::value,
                "One of the args has no ostream overload!");
  for (; *text != '\0'; ++text) {
    if (*text == '{') {
      if (*(text + 1) == '{') {
        ++text;  // escaped brace: emit one '{' below
      } else {
        std::string fmt;
        bool closed = true;
        if (*(text + 1) != '}') {
          fmt = text + 1;
          const size_t pos = fmt.find('}');
          if (pos == std::string::npos) {
            closed = false;
          } else {
            fmt = fmt.substr(0, pos);
            text += fmt.size();
          }
        }
        if (closed) {
          if (!fmt.empty()) {
            if (fmt[0] == '.') {
              stream << std::fixed;
            }
            std::string precision;
            for (int i = 1; i < static_cast<int>(fmt.size()); ++i) {
              if (!isdigit(fmt[i])) {
                break;
              }
              precision += fmt[i];
            }
            if (!precision.empty()) {
              stream << std::setprecision(std::stoi(precision));
            }
          }
          ArgToStream<T&&>::impl(stream, std::forward<T>(arg));
          FormatStream(stream, text + 2, std::forward<Args>(args)...);
          return;
        }
      }
    }
    stream << *text;
  }
  stream << "\nFormat-Warning: There are " << sizeof...(Args) + 1
         << " args unused.";
  return;
}

// 0x1800b8900  FormatString<Eigen::Transpose<const Eigen::Vector3d>, double&, double&>
template <class... Args>
std::string FormatString(char const* text, Args&&... args) {
  std::stringstream stream;
  FormatStream(stream, text, std::forward<Args>(args)...);
  return stream.str();
}

inline std::string FormatString() { return std::string(); }

}  // namespace details

// 0x1800CBC40  defaultEnsure<Eigen::Transpose<const Eigen::Vector3d>, double&, double&>
// (called from SO3::exp 0x18010A400, so3ex_base.h:303 "SO3::exp failed! omega: {}, real: {}, img: {}").
// upstream-modified (Sophus common.hpp): message prefix "mlog ensure failed" instead of
// "Sophus ensure failed".  printf is 0x180016B90; the identity calls to the ICF-folded
// "singleton<extended_type_info_typeid<PlatMap>>" thunk are the std::forward<Args>(args).
template <class... Args>
void defaultEnsure(char const* function, char const* file, int line,
                   char const* description, Args&&... args) {
  std::printf("mlog ensure failed in function '%s', file '%s', line %d.\n",
              function, file, line);
  std::cout << details::FormatString(description, std::forward<Args>(args)...)
            << std::endl;
  std::abort();
}
}  // namespace Sophus


// Upstream Sophus SOPHUS_ENSURE without SOPHUS_ENABLE_ENSURE_HANDLER: calls defaultEnsure with
// __FUNCTION__ ("Sophus::SO3exBase<class Sophus::SO3<double,0> >::log", ...), __FILE__ and
// __LINE__ (the so3ex_base.h line numbers are forced there with #line).
#define SOPHUS_ENSURE(expr, ...)                                          \
  ((expr) ? ((void)0)                                                     \
          : ::Sophus::defaultEnsure(__FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__))
