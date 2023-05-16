#ifndef ANALYSIS_BENCHMARK_CPP_
#define ANALYSIS_BENCHMARK_CPP_

#include "algo/ch.h"
#include "algo/h2h.h"
#include "algo/pruned_highway_labeling.h"
#include "glog/logging.h"
#include "query_set.h"
#include "thirdpart/CLI11.hpp"

int main(int argc, char** argv) {
    CLI::App app{"query set generation"};
    std::string graph_file = "", query_file = "";

    QueryOpt opt;

    app.add_option("-g,--graph", graph_file, "graph data path")->required();
    app.add_option("-q,--query", query_file, "query pair path")->required();
    app.add_option("-s,--size", opt.size_, "query size, default 100000 ");

    CLI11_PARSE(app, argc, argv);
    std::shared_ptr<Graph> graph = std::make_shared<Graph>(graph_file);
    std::shared_ptr<QuerySet> query_set = std::make_shared<QuerySet>(graph_file, opt);

    std::vector<std::pair<vid_t, vid_t>> query_pairs = query_set->generate();
    query_set->write_to_file(query_file, query_pairs);

    LOG(INFO) << "write query file finished !";
}

#endif  // ANALYSIS_BENCHMARK_CPP_
