#ifndef ALGO_SP_ALGO_H_
#define ALGO_SP_ALGO_H_

#include <memory>

#include "graph/graph.h"

class SPAlgo {
   public:
    SPAlgo(std::shared_ptr<Graph>& graph, std::string index_file) : graph_(graph), index_file_(index_file){};

    virtual void processing() = 0;
    virtual w_t query(vid_t v, vid_t u) = 0;

    virtual void load_index() = 0;
    virtual void write_index();

    void batch_query(std::string pair_path);

    virtual void batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) {
        std::vector<w_t> weight;
        for (auto& p : v_pair_lists) {
            weight.push_back(query(p.first, p.second));
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
#endif  // ALGO_SP_ALGO_H_
