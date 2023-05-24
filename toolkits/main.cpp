#include <memory>

#include "algo/ch.h"
#include "algo/h2h.h"
#include "algo/pruned_highway_labeling.h"
#include "glog/logging.h"
#include "query_set.h"
#include "thirdpart/CLI11.hpp"

int main(int argc, char** argv) {
    CLI::App app{"shortest path query"};

    std::string log_dir;  // Variable to store log directory
    app.add_option("--log_dir", log_dir, "Set the log directory");

    std::string index_file = "", order_file = "", graph_file = "", query_file = "";

    // algorithm : 0:CH 1:PHL 2:H2H
    // operation : 0:build index  1: query 3:static
    int operation = -1, algorithm = 0, query_type = -1;

    app.add_option("-i,--index", index_file, "index saving path")->required();
    app.add_option("-a,--algo", algorithm, "algorithm")->required();
    app.add_option("-o,--operator", operation, "operation")->required();
    app.add_option("-g,--graph", graph_file, "graph data path")->required();
    app.add_option("-q,--query", query_file, "query pair path");
    app.add_option("-t,--type", query_type, "query type");

    app.add_option("--or", order_file, "order file");
    CLI11_PARSE(app, argc, argv);

    if (!log_dir.empty()) {
        google::SetLogDestination(google::GLOG_INFO, log_dir.c_str());
    }
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "command :" << argv[0];

    std::shared_ptr<SPAlgo> algo;
    std::shared_ptr<Graph> graph;

    if (algorithm != 1) {
        graph = std::make_shared<Graph>(graph_file);
    }

    if ((algorithm == 0 || algorithm == 2) && order_file.empty()) {
        LOG(INFO) << "please provide with order file !";
        exit(-1);
    }

    if (algorithm == 0) {
        algo = std::make_shared<Ch>(graph, index_file, order_file);
    } else if (algorithm == 1) {
        algo = std::make_shared<PrunedHighwayLabeling>(graph, index_file, graph_file);
    } else if (algorithm == 2) {
        algo = std::make_shared<H2H>(graph, index_file, order_file);
    } else if (algorithm == -1) {
        algo = std::make_shared<SPAlgo>(graph, index_file);
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

        algo->load_index();
        perf::Watch watch;
        if (query_type == 1) {
            std::vector<std::pair<vid_t, vid_t>> query_pairs = QuerySet::read_from_file(query_file);

            LOG(INFO) << "start query";

            watch.mark("t1");
            algo->batch_query(query_pairs);
            LOG(INFO) << "query [total time cost:" << watch.showlit_micros("t1") << "," << watch.showlit_mills("t1")
                      << "," << watch.showlit_seconds("t1") << "]"
                      << "[ average time cost:" << watch.showavg_micros("t1", query_pairs.size()) << ","
                      << watch.showavg_mills("t1", query_pairs.size()) << ","
                      << watch.showavg_seconds("t1", query_pairs.size()) << "]";

        } else if (query_type == 2) {
            for (int i = 1; i <= DEFAULT_GRROUP_COUNT; ++i) {
                std::string flag = "t" + std::to_string(i);
                std::string file_name = query_file + "_Q" + std::to_string(i);
                std::vector<std::pair<vid_t, vid_t>> query_pairs = QuerySet::read_from_file(file_name);

                watch.mark(flag);
                algo->batch_query(query_pairs);

                LOG(INFO) << "**********query type:"
                          << " Q" << std::to_string(i) << " **********" << std::endl;

                LOG(INFO) << "query [total time cost:" << watch.showlit_micros(flag) << "," << watch.showlit_mills(flag)
                          << "," << watch.showlit_seconds(flag) << "]"
                          << "[ average time cost:" << watch.showavg_micros(flag, query_pairs.size()) << ","
                          << watch.showavg_mills(flag, query_pairs.size()) << ","
                          << watch.showavg_seconds(flag, query_pairs.size()) << "]";
            }

        } else {
            LOG(INFO) << " unkown query type!" << std::endl;
            exit(-1);
        }

    } else if (operation == 2) {
        algo->load_index();
        algo->statistics();

    } else {
        LOG(INFO) << "unsupport operation ! ";
    }
}