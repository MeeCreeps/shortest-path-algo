#ifndef GRAPH_GRAPH_H_
#define GRAPH_GRAPH_H_

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "assert.h"
#include "glog/logging.h"
#include "utils/perf.h"

typedef int vid_t;
typedef int w_t;
typedef std::pair<vid_t, w_t> vw_pair;
typedef std::pair<w_t, vid_t> wv_pair;
typedef std::pair<vid_t, vid_t> vv_pair;

typedef std::pair<w_t, std::pair<vid_t, w_t>> wv_h_pair;
typedef std::pair<vid_t, std::pair<vid_t, w_t>> short_cut;  // tuple <dst_id,middle_id,weight> for ch
const w_t INF = 1e8;

class Graph {
   public:
    Graph() = default;
    Graph(std::string file_name) {
        if (file_name.empty()) {
            LOG(INFO) << "graph file is empty !";
            exit(-1);
        } else {
            init_from_file(file_name);
        }
    }
    void init_from_file(std::string file_name);
    void remove_edge(vid_t u, vid_t v);
    vid_t get_v_size() { return v_size_; };
    vid_t get_e_size() { return e_size_; };

    std::vector<std::vector<vw_pair>> neighbors_;

    vid_t v_size_;
    vid_t e_size_;
};

void Graph::init_from_file(std::string file_name) {
    std::ifstream fs(file_name);
    if (!fs) {
        LOG(INFO) << "file is not exist";
        exit(-1);
    }
    fs >> v_size_ >> e_size_;

    neighbors_.resize(v_size_);

    vid_t src, dst, weight;
    for (vid_t i = 0; i < e_size_; ++i) {
        fs >> src >> dst >> weight;
        neighbors_[src].push_back({dst, weight});
        neighbors_[dst].push_back({src, weight});
    }
}

#endif  // GRAPH_GRAPH_H_
