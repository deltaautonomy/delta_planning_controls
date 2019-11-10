/*
 * Author  : Heethesh Vhavle
 * Version : 1.0.0
 * Date    : Nov 10, 2019
 */

#include <string>
#include <chrono>
#include <ctime>

#include <diagnostic_msgs/DiagnosticStatus.h>

#ifndef UTILS_H_
#define UTILS_H_

namespace delta {
namespace utils {

class FPSLogger {
 private:
  std::string name_;
  double fps_;
  double total_time_;
  long int total_frames_;
  std::chrono::system_clock::time_point last_;

 public:
  FPSLogger(std::string name) : name_(name) {
    Reset();
  }

  void Reset() {
    fps_ = 0;
    total_time_ = 0;
    total_frames_ = 0;
    last_ = std::chrono::system_clock::now();
  }

  void Lap() {
    last_ = std::chrono::system_clock::now();
  }

  void Tick(long int count=1) {
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = now - last_;
    total_time_ += elapsed_seconds.count();
    total_frames_ += count;
    fps_ = total_frames_ / total_time_;
  }

  double GetFPS() const {
    return fps_;
  }
};

inline diagnostic_msgs::DiagnosticStatus MakeDiagnosticsStatus(
    std::string name, std::string pipeline, double fps) {
  diagnostic_msgs::DiagnosticStatus msg;
  msg.level = diagnostic_msgs::DiagnosticStatus::OK;
  msg.name = name;
  msg.message = std::to_string(fps);
  msg.hardware_id = pipeline;
  return msg;
}

}  // utils
}  // delta

#endif /* UTILS_H_ */
