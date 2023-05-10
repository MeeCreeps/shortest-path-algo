#include "glog/logging.h"
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
    app.add_option("-g,--graph", graph_file, "graph data path");
    app.add_option("-q,--query", query_file, "query pair path");

    CLI11_PARSE(app, argc, argv);
}