#ifndef GRAPH_GRAPH_H_
#define GRAPH_GRAPH_H_

#include <fstream>
#include <string>
#include <vector>

#include "assert.h"
#include "glog/logging.h"
#include "utils/perf.h"

typedef int vid_t;
typedef int w_t;
typedef std::pair<vid_t, w_t> vw_pair;
typedef std::pair<w_t,vid_t> wv_pair;
typedef std::pair<vid_t,vid_t> vv_pair;
typedef std::pair<vid_t, std::pair<vid_t, w_t>>
    ch_nei_pair;  // tuple <dst_id,middle_id,weight> for ch

class graph {
 public:
  void init_from_file(std::string file_name);
  void remove_edge(vid_t u, vid_t v);
  vid_t get_v_size() { return v_size_; };
  vid_t get_e_size() { return e_size_; };

  std::vector<std::vector<vw_pair>> neighbors_;

  vid_t v_size_;
  vid_t e_size_;
};

#endif  // GRAPH_GRAPH_H_
