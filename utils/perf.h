#ifndef UTILS_PERF_H_
#define UTILS_PERF_H_

#include <sys/time.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
namespace perf {

struct mem_status {
  size_t vm_peak;  // KBytes
  size_t vm_size;  // KBytes
  size_t vm_hwm;   // KBytes
  size_t vm_rss;   // KBytes
};

inline void get_mem_usage(mem_status* status) {
  char buffer[1024] = "";

  FILE* file = fopen("/proc/self/status", "r");
  while (fscanf(file, " %1023s", buffer) == 1) {
    if (strcmp(buffer, "VmRSS:") == 0) {
      fscanf(file, " %lu", &status->vm_rss);
    }
    if (strcmp(buffer, "VmHWM:") == 0) {
      fscanf(file, " %lu", &status->vm_hwm);
    }
    if (strcmp(buffer, "VmSize:") == 0) {
      fscanf(file, " %lu", &status->vm_size);
    }
    if (strcmp(buffer, "VmPeak:") == 0) {
      fscanf(file, " %lu", &status->vm_peak);
    }
  }
  fclose(file);
}

inline double current_milliseconds(void) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec * 1000UL + tv.tv_usec / 1000UL;
}

class watch {
 public:
  // start a new mark
  void mark(const std::string& smark) { mark_[smark] = current_milliseconds(); }

  // return a mark's cost in milliseconds
  double show(const std::string& smark) {
    return current_milliseconds() - mark_[smark];
  }

  // return a mark's cost in milliseconds
  std::string showlit_mills(const std::string& smark) {
    double cost = current_milliseconds() - mark_[smark];
    return std::string(std::to_string(cost) + "ms");
  }

  // return a mark's cost in milliseconds
  std::string showlit_seconds(const std::string& smark) {
    double cost = current_milliseconds() - mark_[smark];
    return std::string(std::to_string(cost / 1000.0) + "s");
  }

  // remove mark and return its cost
  double stop(const std::string& smark) {
    double cost = current_milliseconds() - mark_[smark];
    mark_.erase(smark);
    return cost;
  }

 protected:
  std::map<std::string, double> mark_;
};

}  // namespace perf

#endif  // UTILS_PERF_H_
