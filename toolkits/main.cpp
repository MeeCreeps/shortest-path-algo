#include <memory>

#include "algo/ch.h"
#include "algo/h2h.h"
#include "algo/pruned_highway_labeling.h"
#include "glog/logging.h"
#include "query_set.h"
#include "thirdpart/CLI11.hpp"

int main(int argc, char** argv) {
    // google::InitGoogleLogging(argv[0]);
    CLI::App app{"shortest path query"};

    std::string index_file = "", order_file = "", graph_file = "", query_file = "";

    // algorithm : 0:CH 1:PHL 2:H2H
    // operation : 0:build index  1: query
    int operation = -1, algorithm = 0;

    app.add_option("-i,--index", index_file, "index saving path")->required();
    app.add_option("-a,--algo", algorithm, "graph data path")->required();
    app.add_option("-o,--operator", operation, "operation")->required();
    app.add_option("-g,--graph", graph_file, "graph data path")->required();
    app.add_option("-q,--query", query_file, "query pair path");

    app.add_option("--or", order_file, "order file");
    CLI11_PARSE(app, argc, argv);

    std::shared_ptr<SPAlgo> algo;
    std::shared_ptr<Graph> graph = std::make_shared<Graph>();

    if (graph_file.empty()) {
        LOG(INFO) << "graph file is empty !";
        exit(-1);
    } else {
        graph->init_from_file(graph_file);
    }

    if ((algorithm == 0 || algorithm == 1) && order_file.empty()) {
        LOG(INFO) << "please provide with order file !";
        exit(-1);
    }

    if (algorithm == 0) {
        algo = std::make_shared<Ch>(graph, index_file, order_file);
    } else if (algorithm == 1) {
        algo = std::make_shared<PrunedHighwayLabeling>(graph, index_file);
    } else if (algorithm == 2) {
        algo = std::make_shared<H2H>(graph, index_file, order_file);
    } else {
        LOG(INFO) << "unknow algo:";
        exit(-1);
    }

    if (operation == 0) {
        algo->processing();
    } else if (operation == 1) {
        if (query_file.empty()) {
            LOG(INFO) << "query file is not exist!";
            exit(-1);
        }
        std::vector<std::pair<vid_t, vid_t>> query_pairs = QuerySet::read_from_file(query_file);

        LOG(INFO) << "start query";

        perf::Watch watch;
        watch.mark("t1");

        algo->batch_query(query_pairs);

        LOG(INFO) << "query finish, time cost:" << watch.showlit_mills("t1") << " ms";

    } else {
        LOG(INFO) << "unsupport operation ! ";
    }
}