#ifndef ALGO_GENERATE_CPP_
#define ALGO_GENERATE_CPP_

#include "algo/h2h.h"
#include "glog/logging.h"
#include "graph/graph.h"
#include "query_set.h"
#include "thirdpart/CLI11.hpp"
int main(int argc, char** argv) {
    CLI::App app{"query set generation"};
    std::string graph_file = "", query_file = "", index_file = "";

    QueryOpt opt;

    app.add_option("-g,--graph", graph_file, "graph data path")->required();
    app.add_option("-q,--query", query_file, "query pair path")->required();
    app.add_option("-s,--size", opt.size_, "query size, default 100000 ");
    app.add_option("-t,--type", opt.type_, "query type")->required();
    app.add_option("-i,--index", index_file, "index file ");

    std::string log_dir;  // Variable to store log directory
    app.add_option("--log_dir", log_dir, "Set the log directory");

    CLI11_PARSE(app, argc, argv);

    if (!log_dir.empty()) {
        google::SetLogDestination(google::GLOG_INFO, log_dir.c_str());
    }
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "command :" << argv[0];

    std::shared_ptr<Graph> graph = std::make_shared<Graph>(graph_file);

    if (opt.type_ == 1) {
        std::shared_ptr<QuerySet> query_set = std::make_shared<QuerySet>(graph, opt);
        std::vector<std::pair<vid_t, vid_t>> query_pairs = query_set->generate_randomly();
        query_set->write_to_file(query_file, query_pairs);

    } else if (opt.type_ == 2) {
        std::shared_ptr<SPAlgo> algo = std::make_shared<H2H>(index_file);
        algo->load_index();

        std::shared_ptr<QuerySet> query_set = std::make_shared<QuerySet>(graph, algo);
        std::vector<std::vector<std::pair<vid_t, vid_t>>> query_pairs = query_set->generate_by_dis();

        for (int i = 1; i <= query_pairs.size(); ++i) {
            std::string file = query_file + "_Q" + std::to_string(i);
            query_set->write_to_file(file, query_pairs[i - 1]);
        }

    } else if (opt.type_ == 3) {

        std::shared_ptr<SPAlgo> algo = std::make_shared<H2H>(index_file);
        algo->load_index();

        std::shared_ptr<QuerySet> query_set = std::make_shared<QuerySet>(graph, algo);
        std::vector<std::vector<std::pair<vid_t, vid_t>>> query_pairs = query_set->generate_by_diameter();

        // for (int i = 1; i <= query_pairs.size(); ++i) {
        //     std::string file = query_file + "_Q" + std::to_string(i);
        //     query_set->write_to_file(file, query_pairs[i - 1]);
        // }


    } else {
        LOG(INFO) << " unkown query type!" << std::endl;
    }

    LOG(INFO) << "write query file finished !";
}

#endif  // ALGO_GENERATE_CPP_
