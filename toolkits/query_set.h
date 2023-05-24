#ifndef ANALYSIS_QUERY_SET_H_
#define ANALYSIS_QUERY_SET_H_

#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "algo/sp_algo.h"
#include "graph/graph.h"

enum queryType { Random = 1, Bydis = 2 };

struct QueryOpt {
    int size_ = 10000;
    queryType type_ = queryType::Random;
};

const int DEFAULT_DELTA = 1000;
const int DEFAULT_GRROUP_COUNT = 10;

class QuerySet {
   public:
    QuerySet(std::shared_ptr<Graph>& graph, QueryOpt opt = QueryOpt()) : opt_(opt), graph_(graph){};

    QuerySet(std::shared_ptr<Graph>& graph, std::shared_ptr<SPAlgo>& algo, QueryOpt opt = QueryOpt())
        : graph_(graph), opt_(opt), algo_(algo){};

    static std::vector<std::pair<vid_t, vid_t>> read_from_file(std::string file_name);

    void write_to_file(std::string file_name, std::vector<std::pair<vid_t, vid_t>>& query_set);

    std::vector<std::pair<vid_t, vid_t>> generate_randomly() { return generate_randomly(opt_); };

    std::vector<std::vector<std::pair<vid_t, vid_t>>> generate_by_dis();

   private:
    std::shared_ptr<Graph> graph_;
    QueryOpt opt_;
    std::shared_ptr<SPAlgo> algo_;

    inline std::vector<std::pair<vid_t, vid_t>> generate_randomly(QueryOpt opt);
    int get_dist_level(w_t dist, std::vector<std::pair<double, double>>& dis_range);
};

std::vector<std::vector<std::pair<vid_t, vid_t>>> QuerySet::generate_by_dis() {
    int group_count = DEFAULT_GRROUP_COUNT;
    std::map<std::pair<vid_t, vid_t>, w_t> w_to_pvs;

    std::vector<std::vector<std::pair<vid_t, vid_t>>> group_pvs(group_count);
    std::vector<std::vector<std::pair<vid_t, vid_t>>> res(group_count);
    QueryOpt opt;
    opt.size_ = DEFAULT_DELTA * opt_.size_ * DEFAULT_GRROUP_COUNT;
    opt.type_ = queryType::Random;
    auto query_set = generate_randomly(opt);

    w_t max_dist = 0, min_dist = INF;
    for (auto pair : query_set) {
        int v1 = pair.first;
        int v2 = pair.second;
        w_t dist = algo_->query(v1, v2);
        if (dist == INF) continue;
        max_dist = std::max(dist, max_dist);
        min_dist = std::min(min_dist, dist);

        if (v1 > v2) {
            std::swap(v1, v2);
        }
        w_to_pvs.insert({{v1, v2}, dist});
    }

    LOG(INFO) << " min dists: " << min_dist << " , max dists:" << max_dist << std::endl;

    double x = std::pow((double)max_dist / min_dist, 0.1);

    std::vector<std::pair<double, double>> dis_range;

    for (int i = 1; i <= group_count; ++i) {
        double left = min_dist * std::pow(x, i - 1);
        if (i == 1) left = 0;
        double right = min_dist * std::pow(x, i);
        if (i == group_count) right = max_dist;
        dis_range.push_back(std::make_pair(left, right));
    }

    for (auto e : dis_range) {
        LOG(INFO) << " range [" << e.first << "," << e.second << "]" << std::endl;
    }

    int index = 0;
    for (auto& w_pair : w_to_pvs) {
        index++;

        int dist = w_pair.second;
        assert(dist != INF);
        int level = get_dist_level(dist, dis_range);
        group_pvs[level].push_back(w_pair.first);
    }
    LOG(INFO) << "finish leveling" << std::endl;

    int group_size = opt_.size_;

    for (int i = 0; i < group_count; ++i) {
        std::uniform_int_distribution<unsigned> u(0, group_pvs[i].size() - 1);
        std::default_random_engine e;
        e.seed(time(NULL));
        while (res[i].size() < group_size) {

            int index = u(e);
            res[i].push_back(group_pvs[i][index]);
        }
    }
    return res;
};

int QuerySet::get_dist_level(w_t dist, std::vector<std::pair<double, double>>& dis_range) {
    int res = -1;
    for (int i = 0; i < dis_range.size(); ++i) {
        double min = dis_range[i].first;
        double max = dis_range[i].second;
        if (dist > min && dist <= max) {
            res = i;
            break;
        }
    }
    assert(res != -1);
    return res;
}

std::vector<std::pair<vid_t, vid_t>> QuerySet::generate_randomly(QueryOpt opt) {
    std::vector<std::pair<vid_t, vid_t>> query_pairs;

    std::uniform_int_distribution<unsigned> u(0, graph_->v_size_ - 1);
    std::default_random_engine e;
    e.seed(time(NULL));

    vid_t v1, v2;
    while (query_pairs.size() < opt.size_) {
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

    return query_set;
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
