#ifndef ANALYSIS_QUERY_SET_H_
#define ANALYSIS_QUERY_SET_H_

#include <memory>
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
};

std::vector<std::pair<vid_t, vid_t>> QuerySet::generate() {}

static std::vector<std::pair<vid_t, vid_t>> read_from_file(std::string file_name) {
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

void write_to_file(std::string file_name, std::vector<std::pair<vid_t, vid_t>>& query_set) {
    std::ofstream file(file_name);
    file << query_set.size();
    for (auto& pair : query_set) {
        file << pair.first << pair.second;
    }
    LOG(INFO) << "finsh writing query set";
    file.close();
}

#endif  // ANALYSIS_QUERY_SET_H_