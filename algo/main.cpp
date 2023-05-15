#include <memory>

#include "algo/ch.h"
#include "algo/h2h.h"
#include "algo/pruned_highway_labeling.h"
#include "glog/logging.h"
#include "thirdpart/CLI11.hpp"

int main(int argc, char** argv) {
    // google::InitGoogleLogging(argv[0]);
    CLI::App app{"shortest path query"};

    std::string index_file = "", order_file = "", graph_file = "", query_file = "";

    // algorithm : 0:CH 1:PHL 2:H2H
    // operation : 0:build index  1: query 2.analysis
    int operation = -1, algorithm = 0;

    app.add_option("-i,--index", index_file, "index saving path")->required();
    app.add_option("-a,--algo", algorithm, "graph data path")->required();
    app.add_option("-o,--operator", operation, "operation")->required();
    app.add_option("-g,--graph", graph_file, "graph data path");
    app.add_option("-q,--query", query_file, "query pair path");

    CLI11_PARSE(app, argc, argv);

    std::shared_ptr<SPAlgo> algo;

    if (algorithm == 0) {
        algo = std::make_shared<Ch>();
    } else if (algorithm == 1) {
        algo = std::make_shared<PrunedHighwayLabeling>();
    } else if (algorithm == 1) {
        algo = std::make_shared<H2H>();
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
        algo->batch_query(query_file);
    } else {
    }
}