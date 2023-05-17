#ifndef ANALYSIS_QUERY_SET_H_
#define ANALYSIS_QUERY_SET_H_

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "graph/graph.h"
struct QueryOpt {
    int size_ = 100000;
};

class QuerySet {
   public:
    QuerySet(std::shared_ptr<Graph>& graph, QueryOpt opt = QueryOpt()) : opt_(opt), graph_(graph){};

    std::vector<std::pair<vid_t, vid_t>> generate();

    static std::vector<std::pair<vid_t, vid_t>> read_from_file(std::string file_name);

    void write_to_file(std::string file_name, std::vector<std::pair<vid_t, vid_t>>& query_set);

   private:
    std::shared_ptr<Graph> graph_;
    QueryOpt opt_;
    std::vector<std::pair<vid_t, vid_t>> generate_randomly();
};

std::vector<std::pair<vid_t, vid_t>> QuerySet::generate() {
    // default:randomly
    return generate_randomly();
}

std::vector<std::pair<vid_t, vid_t>> QuerySet::generate_randomly() {
    std::vector<std::pair<vid_t, vid_t>> query_pairs;

    std::uniform_int_distribution<unsigned> u(0, graph_->v_size_);
    std::default_random_engine e;
    e.seed(time(NULL));

    vid_t v1, v2;
    while (query_pairs.size() < opt_.size_) {
        v1 = u(e);
        v2 = u(e);
        if (v1 != v2) {
            query_pairs.push_back({v1, v2});
        }
    }
    return query_pairs;
}

std::vector<std::pair<vid_t, vid_t>> QuerySet::read_from_file(std::string file_name) {
    std::ifstream file(file_name);
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
}

void QuerySet::write_to_file(std::string file_name, std::vector<std::pair<vid_t, vid_t>>& query_set) {
    std::ofstream file(file_name);
    file << query_set.size() << std::endl;
    for (auto& pair : query_set) {
        file << pair.first << " " << pair.second << std::endl;
    }
    LOG(INFO) << "finsh writing query set";
    file.close();
}

#endif  // ANALYSIS_QUERY_SET_H_
