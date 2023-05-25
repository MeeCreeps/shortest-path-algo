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

    Ch(std::string index_file) : SPAlgo(index_file) {}

    // void init_contracted_graph();
    void processing() override;
    void build_ch_index();

    virtual void contraction();

    void load_order();
    void write_order();

    void load_index() { read_binary(); };
    void write_index() { write_binary(); };

    virtual void write_binary();
    virtual void read_binary();

    virtual void write_original();
    virtual void read_original();

    void generate_order();

    inline w_t query(vid_t v, vid_t u) { return bi_dijkstra(v, u); }

    void batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) override;

    virtual void contract_node(vid_t vid, bool is_ori);

    void contract_node(vid_t vid);

    void statistics() override;

   protected:
    inline w_t bi_dijkstra(vid_t s, vid_t t);

    std::vector<short_cut> get_shortcuts(vid_t b_node, vid_t m_node, std::vector<vw_pair>& pairs, w_t max_d);

    void limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop, std::vector<w_t>& dist, std::vector<vid_t>& visited);

    void delete_edge(std::vector<std::vector<vw_pair>>& graph, vid_t u, vid_t v);
    void add_edge(std::vector<std::vector<vw_pair>>& graph, vid_t u, vid_t v);

    // <order , vertex_id >
    std::vector<vid_t> order_;
    // <vertex_id , order>
    std::vector<vid_t> invert_order_;

    // perhaps use unordered_map
    std::vector<std::vector<vw_pair>> cgraph_;

    // std::vector<std::map<vid_t, vid_t>> shortcut_node_;

    std::vector<bool> contracted_;
    vid_t v_size_;

    int c_index = 0;
    int index = 0;
    int in_index = 0;

    // for bi_dijkstra
    std::vector<w_t> dist_s;
    std::vector<w_t> dist_t;

    // for dijkstra
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
    watch.mark("t1");
    cgraph_.resize(v_size_);

    dists.assign(v_size_, INF);
    // shortcut_node_.resize(v_size_);

    for (vid_t i = 0; i < v_size_; ++i) {
        for (auto& edge : graph_->neighbors_[i]) {
            cgraph_[i].push_back({edge.first, edge.second});
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

    LOG(INFO) << "Ch start contracting";

    contraction();

    LOG(INFO) << "Ch finish contracting, time cost:" << watch.showlit_mills("t1") << " ," << watch.showlit_seconds("t1")
              << " ." << std::endl;

    std::cout << "Ch finish contracting, time cost:" << watch.showlit_mills("t1") << " ," << watch.showlit_seconds("t1")
              << " ." << std::endl;
}

void Ch::contraction() {
    for (auto v : order_) {
        contract_node(v);
    }
}

void Ch::contract_node(vid_t vid) {
    assert(!contracted_[vid]);
    contracted_[vid] = true;

    std::vector<short_cut> final_short;

    if (cgraph_[vid].size() > 1) {
        for (auto& e1 : cgraph_[vid]) {
            vid_t v1 = e1.first;
            w_t d1 = e1.second;
            for (auto& e2 : cgraph_[vid]) {
                if (invert_order_[e2.first] <= invert_order_[v1]) continue;
                w_t d2 = e2.second + d1;
                vid_t v2 = e2.first;
                final_short.push_back({v1, {v2, d2}});
            }
        }
    }

    for (auto& edge : cgraph_[vid]) {
        vid_t v = edge.first;
        for (auto iter = cgraph_[v].begin(); iter != cgraph_[v].end(); ++iter) {
            if (iter->first == vid) {
                cgraph_[v].erase(iter);
                break;
            }
        }
    }
    // cgraph_[vid].clear();

    for (auto& cuts : final_short) {
        vid_t v1 = cuts.first;
        vid_t v2 = cuts.second.first;
        w_t w = cuts.second.second;

        // order(v1) < order(v2)

        // add shortcuts for final graph (for index saving)
        // TODO : change remained_graph
        bool find = false;
        for (auto iter = cgraph_[v1].begin(); iter != cgraph_[v1].end(); ++iter) {
            if (iter->first == v2) {
                find = true;
                if (iter->second > w) iter->second = w;
                break;
            }
        }

        if (!find) {
            cgraph_[v1].push_back({v2, w});
        }
    }
}

void Ch::contract_node(vid_t vid, bool is_ori) {
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
        for (auto iter = cgraph_[v].begin(); iter != cgraph_[v].end(); ++iter) {
            if (iter->first == vid) {
                cgraph_[v].erase(iter);
                break;
            }
        }
    }

    // add shortcuts
    for (auto& cuts : final_short) {
        vid_t v1 = cuts.first;
        vid_t v2 = cuts.second.first;
        w_t w = cuts.second.second;

        // add shortcuts for final graph (for index saving)
        bool find = false;
        for (auto iter = cgraph_[v1].begin(); iter != cgraph_[v1].end(); ++iter) {
            if (iter->first == v2) {
                find = true;
                if (iter->second > w) iter->second = w;
                break;
            }
        }

        if (!find) {
            cgraph_[v1].push_back({v2, w});
        }

        find = false;

        for (auto iter = cgraph_[v2].begin(); iter != cgraph_[v2].end(); ++iter) {
            if (iter->first == v1) {
                find = true;
                if (iter->second > w) iter->second = w;
                break;
            }
        }

        if (!find) {
            cgraph_[v2].push_back({v1, w});
        }
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
    auto contracted_graph = cgraph_;

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

void Ch::delete_edge(std::vector<std::vector<vw_pair>>& graph, vid_t u, vid_t v) {
    for (auto iter = graph[u].begin(); iter != graph[u].end(); ++iter) {
        if (iter->first == v) {
            graph[u].erase(iter);
            D[u]--;
            break;
        }
    }
    for (auto iter = graph[v].begin(); iter != graph[v].end(); ++iter) {
        if (iter->first == u) {
            graph[v].erase(iter);
            D[v]--;
            break;
        }
    }
}

void Ch::add_edge(std::vector<std::vector<vw_pair>>& graph, vid_t u, vid_t v) {
    bool find = false;
    for (auto iter = graph[u].begin(); iter != graph[u].end(); ++iter) {
        if (iter->first == v) {
            find = true;
            break;
        }
    }
    if (!find) {
        graph[u].push_back({v, 1});
        D[u]++;
        D2[u]++;
    }
    find = false;
    for (auto iter = graph[v].begin(); iter != graph[v].end(); ++iter) {
        if (iter->first == u) {
            find = true;
            break;
        }
    }

    if (!find) {
        graph[v].push_back({u, 1});
        D[v]++;
        D2[v]++;
    }
}

// max_hop = 2
// expired function
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

        if (dist > dists[vid]) continue;

        visited.push_back(vid);

        // if (hop >= max_hop) continue;

        for (auto& vw : cgraph_[vid]) {
            if (contracted_[vw.first]) continue;
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

    std::vector<vid_t> visted_s, visted_t;

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

            visted_s.push_back(s_u);

            if (s_w > dist_s[s_u]) continue;
            if (dist_s[s_u] < min_dist) {
                for (auto& edge : cgraph_[s_u]) {
                    s_w = edge.second + dist_s[s_u];
                    if (dist_s[edge.first] <= s_w) continue;
                    dist_s[edge.first] = s_w;
                    ds_queue.push({s_w, edge.first});

                    visted_s.push_back(edge.first);
                }
                min_dist = std::min(min_dist, dist_s[s_u] + dist_t[s_u]);
            }
        }

        if (!dt_queue.empty()) {
            t_u = dt_queue.top().second;
            t_w = dt_queue.top().first;
            dt_queue.pop();

            visted_t.push_back(t_u);

            if (t_w > dist_t[t_u]) continue;
            if (dist_t[t_u] < min_dist) {
                for (auto& edge : cgraph_[t_u]) {
                    t_w = edge.second + dist_t[t_u];
                    if (dist_t[edge.first] <= t_w) continue;
                    dist_t[edge.first] = t_w;

                    dt_queue.push({t_w, edge.first});
                    visted_t.push_back(edge.first);
                }
                min_dist = std::min(min_dist, dist_s[t_u] + dist_t[t_u]);
            }
        }
    }

    for (int i = 0; i < visted_s.size(); i++) {
        dist_s[visted_s[i]] = INF;
    }
    for (int i = 0; i < visted_t.size(); i++) {
        dist_t[visted_t[i]] = INF;
    }

    return min_dist;
}

void Ch::batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) {
    dist_s.assign(v_size_, INF);
    dist_t.assign(v_size_, INF);

    std::vector<w_t> weight;
    for (auto& p : v_pair_lists) {
        weight.push_back(query(p.first, p.second));
        SHOW_DIST(LOG(INFO) << " u:" << p.first << " v:" << p.second << " sp:" << weight.back();)
    }
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

void Ch::read_original() {
    std::ifstream fs(index_file_);

    if (!fs.good()) {
        LOG(INFO) << "index file is not exist ! processing first";
        exit(-1);
    }
    vid_t u, v;
    w_t weight;
    fs >> v_size_;

    cgraph_.resize(v_size_);

    while (fs >> u >> v >> weight) {
        // contracted graph saved same edges twice
        cgraph_[u].push_back({v, weight});
        // cgraph_[v].insert({u, weight});
    }
    fs.close();

    // load_order
    load_order();
}

void Ch::write_original() {
    std::ofstream fs(index_file_);

    fs << cgraph_.size() << std::endl;

    for (int i = 0; i < cgraph_.size(); ++i) {
        for (auto& edge : cgraph_[i]) {
            if (invert_order_[edge.first] > invert_order_[i])
                fs << i << " " << edge.first << " " << edge.second << std::endl;
        }
    }
    fs.close();
}

void Ch::statistics() {
    long long index_size = 0;

    for (int i = 0; i < v_size_; ++i) {
        index_size += cgraph_[i].size();
    }

    long long mem_mb = index_size * sizeof(int) * 2 / 1024.0 / 1024;
    LOG(INFO) << "CH contracted graph edge size: " << index_size << ",aveage degree:" << (double)index_size / v_size_
              << "\n"
              << "CH index size:" << mem_mb << "MB , " << mem_mb / 1024.0 << "GB";

    perf::mem_status mstatus;
    perf::get_mem_usage(&mstatus);

    LOG(INFO) << "mem usage:" << (double)mstatus.vm_rss / 1024.0 << "MB"
              << " , " << (double)mstatus.vm_rss / 1024.0 / 1024.0 << "GB" << std::endl;
}

void Ch::write_binary() {
    std::ofstream fs(index_file_, std::ios::binary);

    int size = cgraph_.size();

    fs.write((char*)&size, sizeof(int));

    for (int i = 0; i < cgraph_.size(); ++i) {
        for (auto& edge : cgraph_[i]) {
            if (invert_order_[edge.first] > invert_order_[i]) {
                fs.write((char*)&i, sizeof(i));
                fs.write((char*)&edge.first, sizeof(int));
                fs.write((char*)&edge.second, sizeof(int));
            }
        }
    }
    fs.close();
}
void Ch::read_binary() {
    std::ifstream fs(index_file_, std::ios::binary);

    if (!fs.good()) {
        LOG(INFO) << "index file is not exist ! processing first";
        exit(-1);
    }
    vid_t u, v;
    w_t weight;

    fs.read((char*)&v_size_, sizeof(int));
    cgraph_.resize(v_size_);

    while (!fs.eof()) {
        fs.read((char*)&u, sizeof(int));
        fs.read((char*)&v, sizeof(int));
        fs.read((char*)&weight, sizeof(int));
        cgraph_[u].push_back({v, weight});
    }

    fs.close();

    // load_order
    load_order();
}

#endif  // ALGO_CH_H_
