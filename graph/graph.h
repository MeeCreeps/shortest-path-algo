#ifndef GRAPH_GRAPH_H_
#define GRAPH_GRAPH_H_

#include <fstream>
#include <string>
#include <vector>

#include "utils/perf.h"


#include "assert.h"
#include "glog/logging.h"

typedef uint32_t vid_t;
typedef std::pair<int, int> wpair;

class graph {
 public:
  void init_from_file(std::string file_name);
  void remove_edge(vid_t u, vid_t v);
  vid_t get_v_size() { return v_size_; };
  vid_t get_e_size() { return e_size_; };

 protected:
  std::vector<std::vector<wpair>> neighbors_;

  vid_t v_size_;
  vid_t e_size_;
};

#endif  // GRAPH_GRAPH_H_
