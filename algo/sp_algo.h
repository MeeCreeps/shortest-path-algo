#ifndef ALGO_SP_ALGO_H_
#define ALGO_SP_ALGO_H_

#include <memory>
#include <queue>

#include "graph/graph.h"
#include "utils/config.h"
#include "utils/perf.h"
class SPAlgo {
   public:
    SPAlgo() = default;

    SPAlgo(std::shared_ptr<Graph>& graph, std::string index_file) : graph_(graph), index_file_(index_file){};

    // just for query , 
    SPAlgo(std::string index_file) : index_file_(index_file){};

    virtual void processing(){};
    virtual inline w_t query(vid_t v, vid_t u);

    virtual void load_index(){};
    virtual void write_index(){};

    virtual void statistics(){};

    void batch_query(std::string pair_path);

    virtual void batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) {
        std::vector<w_t> weight;
        for (auto& p : v_pair_lists) {
            weight.push_back(query(p.first, p.second));
            SHOW_DIST(LOG(INFO) << " u:" << p.first << " v:" << p.second << " sp:" << weight.back();)
        }
    }

   protected:
    std::shared_ptr<Graph> graph_;
    std::string index_file_;
};

void SPAlgo::batch_query(std::string pair_path) {
    std::ifstream file(pair_path);
    if (!file.good()) {
        LOG(INFO) << "query pair file is not exist!";
        exit(-1);
    }

    std::vector<std::pair<vid_t, vid_t>> query_set;
    vid_t u, v;
    int size;
    file >> size;
    for (int i = 0; i < size; ++i) {
        file >> u >> v;
        query_set.push_back({u, v});
    }
    batch_query(query_set);
    
}

w_t SPAlgo::query(vid_t v, vid_t u) {
    if (v == u) return 0;
    std::vector<w_t> dists(graph_->v_size_, INF);
    std::vector<bool> visited(graph_->v_size_, false);
    std::priority_queue<wv_pair, std::vector<wv_pair>, std::greater<wv_pair>> dist_queue;

    dists[v] = 0;

    dist_queue.push({0, v});
    w_t d, new_weight;
    while (!dist_queue.empty()) {
        vid_t vid = dist_queue.top().second;
        w_t dist = dist_queue.top().first;
        dist_queue.pop();
        visited[vid] = true;
        if (vid == u) {
            d = dists[u];
            break;
        }
        for (auto& edge : graph_->neighbors_[vid]) {
            if (!visited[edge.first]) {
                new_weight = edge.second + dist;
                if (dists[edge.first] > new_weight) {
                    dists[edge.first] = new_weight;
                    dist_queue.push({new_weight, edge.first});
                }
            }
        }
    }
    return d;
}

#endif  // ALGO_SP_ALGO_H_
