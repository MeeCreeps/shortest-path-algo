#ifndef ALGO_CH_H_
#define ALGO_CH_H_
#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <set>

#include "sp_algo.h"

class Ch : public SPAlgo {
   public:
    Ch(std::shared_ptr<Graph>& graph, std::string index_file, std::string order_file)
        : SPAlgo(graph, index_file), order_file_(order_file) {
        v_size_ = graph_->v_size_;
    }

    // void init_contracted_graph();
    void processing() override;
    void build_ch_index();

    virtual void contraction();

    void load_order();
    void write_order();
    void load_index();
    void write_index();

    void generate_order();

    inline w_t query(vid_t v, vid_t u) { return bi_dijkstra(v, u); }

    virtual void contract_node(vid_t vid, int index);

    void contract_node(vid_t vid){};

    void statistics() override;

   protected:
    w_t bi_dijkstra(vid_t s, vid_t t);

    std::vector<short_cut> get_shortcuts(vid_t b_node, vid_t m_node, std::vector<vw_pair>& pairs, w_t max_d);

    void limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop, std::vector<w_t>& dist, std::vector<vid_t>& visited);

    void delete_edge(std::vector<std::map<vid_t, w_t>>& graph, vid_t u, vid_t v);
    void add_edge(std::vector<std::map<vid_t, w_t>>& graph, vid_t u, vid_t v);

    // <order , vertex_id >
    std::vector<vid_t> order_;
    // <vertex_id , order>
    std::vector<vid_t> invert_order_;

    // perhaps use unordered_map
    std::vector<std::map<vid_t, w_t>> contracted_graph_;

    std::vector<std::map<vid_t, vid_t>> shortcut_node_;

    // use for contracting nodes
    std::vector<std::map<vid_t, w_t>> cgraph_;

    std::vector<bool> contracted_;
    vid_t v_size_;

    int c_index = 0;
    int index = 0;
    int in_index = 0;

    std::vector<int> changed;
    std::vector<w_t> dists;
    std::string order_file_;
};

void Ch::processing() {
    std::ifstream fs(index_file_);
    if (fs.good()) {
        fs.close();
        load_index();
        load_order();
    } else {
        build_ch_index();
        write_index();
    }
}

void Ch::build_ch_index() {
    perf::Watch watch;

    contracted_graph_.resize(v_size_);
    cgraph_.resize(v_size_);

    changed.assign(v_size_, false);
    dists.assign(v_size_, INF);
    // shortcut_node_.resize(v_size_);

    for (vid_t i = 0; i < v_size_; ++i) {
        for (auto& edge : graph_->neighbors_[i]) {
            contracted_graph_[i].insert({edge.first, edge.second});
            cgraph_[i].insert({edge.first, edge.second});
        }
    }

    LOG(INFO) << " begin generate order:";
    std::ifstream fs(order_file_);
    if (!fs.good()) {
        watch.mark("t-order");

        generate_order();

        LOG(INFO) << "ch generate order, time cost:" << watch.showlit_mills("t-order") << " ,"
                  << watch.showlit_seconds("t-order") << " ." << std::endl;

        write_order();
    } else {
        fs.close();
        load_order();
    }

    LOG(INFO) << "generating/load order finihsed!";
    std::cout << "generating/load  order finihsed!" << std::endl;

    contracted_.resize(graph_->get_v_size(), false);
    watch.mark("t1");

    LOG(INFO) << "Ch start contracting";

    contraction();

    LOG(INFO) << "Ch finish contracting, time cost:" << watch.showlit_mills("t1") << " ," << watch.showlit_seconds("t1")
              << " ." << std::endl;

    std::cout << "Ch finish contracting, time cost:" << watch.showlit_mills("t1") << " ," << watch.showlit_seconds("t1")
              << " ." << std::endl;
}

void Ch::contraction() {
    perf::Watch watch;
    watch.mark("t1");

    for (auto v : order_) {
        index++;
        if (index % 1000 == 0) {
            LOG(INFO) << "index:" << index << " " << watch.showlit_mills("t1") << " ," << watch.showlit_seconds("t1")
                      << " ." << std::endl;
        }
        contract_node(v, index);
    }
}

// void Ch::contract_node(vid_t vid) {
//     assert(!contracted_[vid]);
//     contracted_[vid] = true;

//     w_t max_dist = 0, sec_dist = 0;
//     for (auto& pair : contracted_graph_[vid]) {
//         if (contracted_[pair.first]) continue;
//         if (pair.second > max_dist) {
//             max_dist = pair.second;
//         } else if (pair.second > sec_dist) {
//             sec_dist = pair.second;
//         }
//     }
//     max_dist += sec_dist;

//     for (auto& pair : contracted_graph_[vid]) {
//         if (contracted_[pair.first]) continue;
//         std::vector<vid_t> visited;
//         limit_dijkstra(pair.first, max_dist, 2, dists, visited);
//         for (auto& neighbor : contracted_graph_[vid]) {
//             if (invert_order_[pair.first] >= invert_order_[neighbor.first]) continue;
//             w_t total_w = pair.second + neighbor.second;

//             if (total_w < dists[neighbor.first]) {
//                 // add shortcut
//                 contracted_graph_[pair.first][neighbor.first] = total_w;
//                 // contracted_graph_[neighbor.first][pair.first] = total_w;

//                 // TODO add support vertex
//                 // form low level to high level node

//                 // shortcut_node_[pair.first][neighbor.first] = vid;
//             }
//         }

//         for (auto v : visited) {
//             dists[v] = INF;
//             changed[v] = false;
//         }
//     }
// }

void Ch::contract_node(vid_t vid, int index) {
    assert(!contracted_[vid]);
    contracted_[vid] = true;

    std::vector<short_cut> final_short;

    w_t max_dist = 0;
    in_index = 0;

    if (cgraph_[vid].size() > 1) {
        for (auto& e1 : cgraph_[vid]) {
            vid_t b_node = e1.first;
            w_t d1 = e1.second;
            std::vector<vw_pair> pairs;
            for (auto& e2 : cgraph_[vid]) {
                if (invert_order_[e2.first] <= invert_order_[b_node]) continue;
                w_t d2 = e2.second + d1;
                if (d2 > max_dist) max_dist = d2;
                pairs.push_back({e2.first, d2});
            }
            // dijkstra serach
            if (pairs.empty()) continue;
            std::vector<short_cut> short_cuts = get_shortcuts(b_node, vid, pairs, max_dist);

            INULL(LOG(INFO) << "index:" << index << " neighbor size:" << cgraph_[vid].size()
                            << " inner index:" << in_index++ << " pair size:" << pairs.size()
                            << " short_cuts size:" << short_cuts.size() << " final size:" << final_short.size();)
            final_short.insert(final_short.end(), short_cuts.begin(), short_cuts.end());
        }
    }

    // modify graphs

    // remove edges related with vid
    for (auto& edge : cgraph_[vid]) {
        vid_t v = edge.first;
        cgraph_[v].erase(vid);
    }
    cgraph_[vid].clear();

    // add shortcuts
    for (auto& cuts : final_short) {
        vid_t v1 = cuts.first;
        vid_t v2 = cuts.second.first;
        w_t w = cuts.second.second;
        cgraph_[v1][v2] = w;
        cgraph_[v2][v1] = w;

        // add shortcuts for final graph (for index saving)
        contracted_graph_[v1][v2] = w;
        contracted_graph_[v2][v1] = w;
    }
}

std::vector<int> D, D2, _D, _D2;

struct DegComp {
    int v_;
    DegComp(int v) { v_ = v; }
    bool operator<(const DegComp d) const {
        if (_D[v_] != _D[d.v_]) return _D[v_] < _D[d.v_];
        if (_D2[v_] != _D2[d.v_]) return _D2[v_] < _D2[d.v_];
        return v_ < d.v_;
    }
};

// default : degree
// Edge difference: The edge difference is the number of shortcuts added minus
// the number of original edges removed during contraction. Nodes with a lower
// edge difference are contracted first to minimize the total number of
// shortcuts in the graph.
void Ch::generate_order() {
    int v_size = v_size_;

    invert_order_.resize(v_size, 0);

    // copy graph
    auto contracted_graph = contracted_graph_;

    _D.resize(v_size, 0);
    _D2.resize(v_size, 0);
    D.resize(v_size, 0);
    D2.resize(v_size, 0);

    std::vector<bool> removed(v_size, false);
    std::vector<bool> change(v_size, false);

    std::set<DegComp> deg;
    int degree;

    for (int i = 0; i < v_size; ++i) {
        degree = contracted_graph[i].size();
        if (degree != 0) {
            D[i] = degree;
            D2[i] = degree;
            _D[i] = degree;
            _D2[i] = degree;
            deg.insert(DegComp(i));
        }
    }

    // delay the change of degree
    int index = 0;
    while (!deg.empty()) {
        vid_t con_v = deg.begin()->v_;

        while (true) {
            if (change[con_v]) {
                deg.erase(DegComp(con_v));
                _D[con_v] = D[con_v];
                _D2[con_v] = D2[con_v];
                deg.insert(DegComp(con_v));
                change[con_v] = false;
                con_v = deg.begin()->v_;
            } else
                break;
        }

        deg.erase(deg.begin());
        order_.push_back(con_v);
        removed[con_v] = true;
        std::vector<vid_t> neigh;
        for (auto& edge : contracted_graph[con_v]) {
            if (removed[edge.first]) continue;
            neigh.push_back(edge.first);
        }

        for (auto& u : neigh) {
            change[u] = true;
            delete_edge(contracted_graph, con_v, u);
        }
        // add new edge for common neighbors
        for (int i = 0; i < neigh.size(); ++i) {
            for (int j = i + 1; j < neigh.size(); ++j) {
                add_edge(contracted_graph, neigh[i], neigh[j]);
            }
        }
    }

    for (int i = 0; i < order_.size(); ++i) {
        invert_order_[order_[i]] = i;
    }
}

void Ch::delete_edge(std::vector<std::map<vid_t, w_t>>& graph, vid_t u, vid_t v) {
    auto iter = graph[u].find(v);
    if (iter != graph[u].end()) {
        graph[u].erase(iter);
        D[u]--;
    }

    iter = graph[v].find(u);
    if (iter != graph[v].end()) {
        graph[v].erase(iter);
        D[v]--;
    }
}

void Ch::add_edge(std::vector<std::map<vid_t, w_t>>& graph, vid_t u, vid_t v) {
    auto iter = graph[u].find(v);

    if (iter == graph[u].end()) {
        graph[u].insert({v, 1});
        D[u]++;
        D2[u]++;
    }

    iter = graph[v].find(u);
    if (iter == graph[v].end()) {
        graph[v].insert({u, 1});
        D[v]++;
        D2[v]++;
    }
}

// max_hop = 2
void Ch::limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop, std::vector<w_t>& dists, std::vector<vid_t>& visited) {
    std::priority_queue<wv_h_pair, std::vector<wv_h_pair>, std::greater<wv_h_pair>> dist_queue;

    // TODO , add hop limit
    dists[u] = 0;

    dist_queue.push({0, {u, 0}});
    vid_t vid, dist, new_dist;

    while (!dist_queue.empty()) {
        vid_t vid = dist_queue.top().second.first;
        w_t dist = dist_queue.top().first;
        w_t hop = dist_queue.top().second.second;
        dist_queue.pop();

        changed[vid] = true;
        if (dist > dists[vid]) continue;

        visited.push_back(vid);

        // if (hop >= max_hop) continue;

        for (auto& vw : contracted_graph_[vid]) {
            if (contracted_[vw.first] || changed[vw.first]) continue;
            new_dist = dist + vw.second;
            if (new_dist >= max_dist) continue;

            if (dists[vw.first] > new_dist) {
                dists[vw.first] = new_dist;

                dist_queue.push({new_dist, {vw.first, hop + 1}});
                // add pre vertex
                // shortcut_node_[{u, vw.first}] = v;
            }
        }
    }
}

std::vector<short_cut> Ch::get_shortcuts(vid_t b_node, vid_t m_node, std::vector<vw_pair>& pairs, w_t max_d) {
    std::priority_queue<wv_h_pair, std::vector<wv_h_pair>, std::greater<wv_h_pair>> dq;

    INULL(int times = 0;)

    dists[b_node] = 0;
    dq.push({0, {b_node, 0}});
    std::vector<vid_t> visited;
    // add changed
    while (!dq.empty()) {
        vid_t v = dq.top().second.first;
        w_t w = dq.top().first;
        w_t hop = dq.top().second.second;
        INULL(times++;)
        dq.pop();
        visited.push_back(v);

        if (v == m_node) continue;
        // if (hop >= 2) continue;
        if (dists[v] > max_d) break;

        if (w <= dists[v] && hop < 2) {
            for (auto& edge : cgraph_[v]) {
                vid_t v2 = edge.first;
                w_t w2 = edge.second;
                if (v2 == m_node) continue;

                if (dists[v] + w2 < dists[v2]) {
                    dists[v2] = dists[v] + w2;
                    dq.push({dists[v2], {v2, hop + 1}});
                    visited.push_back(v2);
                }
            }
        }
    }

    std::vector<short_cut> short_cuts;

    for (int i = 0; i < pairs.size(); ++i) {
        vid_t v = pairs[i].first;
        w_t w = pairs[i].second;
        // must be >=
        if (dists[v] >= w) {
            if (b_node == v) continue;
            short_cuts.push_back({b_node, {v, w}});
        }
    }

    // reset dists
    for (int i = 0; i < visited.size(); i++) {
        dists[visited[i]] = INF;
    }
    INULL(LOG(INFO) << " djkstra "
                    << " index:" << index << " in_index:" << in_index << " times:" << times;)

    return short_cuts;
}

w_t Ch::bi_dijkstra(vid_t s, vid_t t) {
    std::priority_queue<wv_pair, std::vector<wv_pair>, std::greater<wv_pair>> ds_queue, dt_queue;

    std::vector<w_t> dist_s(v_size_, INF), dist_t(v_size_, INF);

    dist_s[s] = 0, dist_t[t] = 0;
    ds_queue.push({0, s});
    dt_queue.push({0, t});
    w_t min_dist = INF;

    vid_t s_u, t_u;
    w_t s_w, t_w;
    while (!ds_queue.empty() || !dt_queue.empty()) {
        if (!ds_queue.empty()) {
            s_u = ds_queue.top().second;
            s_w = ds_queue.top().first;
            ds_queue.pop();

            if (s_w > dist_s[s_u]) continue;
            if (dist_s[s_u] < min_dist) {
                for (auto& edge : contracted_graph_[s_u]) {
                    s_w = edge.second + dist_s[s_u];
                    if (dist_s[edge.first] <= s_w) continue;
                    dist_s[edge.first] = s_w;
                    ds_queue.push({s_w, edge.first});
                }
                min_dist = std::min(min_dist, dist_s[s_u] + dist_t[s_u]);
            }
        }

        if (!dt_queue.empty()) {
            t_u = dt_queue.top().second;
            t_w = dt_queue.top().first;
            dt_queue.pop();

            if (t_w > dist_t[t_u]) continue;
            if (dist_t[t_u] < min_dist) {
                for (auto& edge : contracted_graph_[t_u]) {
                    t_w = edge.second + dist_t[t_u];
                    if (dist_t[edge.first] <= t_w) continue;
                    dist_t[edge.first] = t_w;
                    dt_queue.push({t_w, edge.first});
                }
                min_dist = std::min(min_dist, dist_s[t_u] + dist_t[t_u]);
            }
        }
    }
    return min_dist;
}

void Ch::load_order() {
    std::ifstream fs(order_file_);
    order_.resize(v_size_);
    invert_order_.resize(v_size_);

    for (vid_t i = 0; i < order_.size(); ++i) {
        fs >> order_[i];
        invert_order_[order_[i]] = i;
    }
    fs.close();
    LOG(INFO) << "LOG << finish loading order file" << std::endl;
    std::cout << "finish loading order file" << std::endl;
}

void Ch::write_order() {
    assert(order_.size() == graph_->get_v_size());
    std::ofstream fs(order_file_);
    for (auto v : order_) {
        fs << v << std::endl;
    }
    fs.close();
    LOG(INFO) << "finsh writing order file";
}

void Ch::load_index() {
    std::ifstream fs(index_file_);

    if (!fs.good()) {
        LOG(INFO) << "index file is not exist ! processing first";
        exit(-1);
    }
    vid_t u, v;
    w_t weight;
    fs >> v_size_;

    contracted_graph_.resize(v_size_);

    while (fs >> u >> v >> weight) {
        // contracted graph saved same edges twice
        contracted_graph_[u].insert({v, weight});
        // contracted_graph_[v].insert({u, weight});
    }
    fs.close();

    // load_order
    load_order();
}

void Ch::write_index() {
    std::ofstream fs(index_file_);

    fs << contracted_graph_.size() << std::endl;

    for (int i = 0; i < contracted_graph_.size(); ++i) {
        for (auto& edge : contracted_graph_[i]) {
            if (invert_order_[edge.first] > invert_order_[i])
                fs << i << " " << edge.first << " " << edge.second << std::endl;
        }
    }
    fs.close();
}

void Ch::statistics() {
    long long index_size = 0;

    for (int i = 0; i < v_size_; ++i) {
        index_size += contracted_graph_[i].size();
    }

    long long mem_mb = index_size * sizeof(int) * 2 / 1024.0 / 1024;
    LOG(INFO) << "CH contracted graph edge size: " << index_size << ",aveage degree:" << (double)index_size / v_size_
              << "\n"
              << "CH index size:" << mem_mb << "MB , " << mem_mb / 1024.0 << "GB";

    perf::mem_status mstatus;
    perf::get_mem_usage(&mstatus);

    LOG(INFO) << "mem usage:" << (double)mstatus.vm_rss / 1024.0 << "MB"
              << " , " << (double)mstatus.vm_rss / 1024.0 / 1024.0 << "GB"<<std::endl;
}

#endif  // ALGO_CH_H_
