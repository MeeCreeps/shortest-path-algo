#ifndef ALGO_CH_H_
#define ALGO_CH_H_
#include <functional>
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

   protected:
    void contract_node(vid_t vid);
    w_t bi_dijkstra(vid_t s, vid_t t);
    void limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop, std::vector<w_t>& dist);

    void delete_edge(std::vector<std::map<vid_t, w_t>>& graph, vid_t u, vid_t v);
    void add_edge(std::vector<std::map<vid_t, w_t>>& graph, vid_t u, vid_t v);

    // <order , vertex_id >
    std::vector<vid_t> order_;
    // <vertex_id , order>
    std::vector<vid_t> invert_order_;

    // perhaps use unordered_map
    std::vector<std::map<vid_t, w_t>> contracted_graph_;
    std::vector<std::map<vid_t, vid_t>> shortcut_node_;

    std::vector<bool> contracted_;
    vid_t v_size_;

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
    contracted_graph_.resize(v_size_);
    // shortcut_node_.resize(v_size_);

    for (vid_t i = 0; i < v_size_; ++i) {
        for (auto& edge : graph_->neighbors_[i]) {
            contracted_graph_[i].insert({edge.first, edge.second});
        }
    }

    LOG(INFO) << " begin generate order:";
    if (order_.empty()) {
        generate_order();
        write_order();
    } else {
        load_order();
    }
    LOG(INFO) << "generating order finihsed!";

    perf::Watch watch;

    contracted_.resize(graph_->get_v_size(), false);
    watch.mark("t1");

    LOG(INFO) << "Ch start contracting";

    contraction();

    LOG(INFO) << "Ch finish contracting, time cost:" << watch.showlit_mills("t1") << " ms";
}

void Ch::contraction() {
    for (auto v : order_) {
        contract_node(v);
    }
}

void Ch::contract_node(vid_t vid) {
    assert(!contracted_[vid]);
    contracted_[vid] = true;

    w_t max_dist = 0, sec_dist = 0;
    for (auto& pair : contracted_graph_[vid]) {
        if (contracted_[pair.first]) continue;
        if (pair.second > max_dist) {
            max_dist = pair.second;
        } else if (pair.second > sec_dist) {
            sec_dist = pair.second;
        }
    }
    max_dist += sec_dist;

    for (auto& pair : contracted_graph_[vid]) {
        if (contracted_[pair.first]) continue;
        std::vector<w_t> dists(v_size_, -1);
        limit_dijkstra(pair.first, max_dist, 2, dists);
        for (auto& neighbor : contracted_graph_[vid]) {
            if (invert_order_[pair.first] > invert_order_[neighbor.first]) continue;
            w_t total_w = pair.second + neighbor.second;
            if (total_w < dists[neighbor.second] || dists[neighbor.second] == -1) {
                // add shortcut
                contracted_graph_[pair.first][neighbor.first] = total_w;
                contracted_graph_[neighbor.first][pair.first] = total_w;

                // TODO add support vertex
                // form low level to high level node

                // shortcut_node_[pair.first][neighbor.first] = vid;
            }
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
    auto contracted_graph = contracted_graph_;

    _D.resize(v_size, 0);
    _D2.resize(v_size, 0);
    D2.resize(v_size, 0);
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
    while (!deg.empty()) {
        vid_t con_v = deg.begin()->v_;
        order_.push_back(con_v);
        removed[con_v] = true;
        deg.erase(deg.begin());

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

        for (int i = 0; i < order_.size(); ++i) {
            invert_order_[order_[i]] = i;
        }
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
void Ch::limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop, std::vector<w_t>& dists) {
    std::priority_queue<wv_pair, std::vector<wv_pair>, std::greater<wv_pair>> dist_queue;

    std::vector<int> hops(v_size_, 0);

    dist_queue.push({0, u});
    while (!dist_queue.empty()) {
        vid_t vid = dist_queue.top().second;
        w_t dist = dist_queue.top().first;

        if (dist > dists[vid]) continue;
        if (hops[vid] > max_hop) continue;
        if (dists[vid] > max_dist) break;

        for (auto& vw : contracted_graph_[vid]) {
            // undirected
            if (contracted_[vw.first]) continue;
            dist = dists[vid] + vw.second;
            if (dists[vw.first] == -1 || dists[vw.first] > dist) {
                dists[vw.first] = dist;
                hops[vw.first] = hops[vid] + 1;
                dist_queue.push({dist, vw.first});
                // add pre vertex
                // shortcut_node_[{u, vw.first}] = v;
            }
        }
    }
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
        while (!ds_queue.empty()) {
            s_u = ds_queue.top().second;
            s_w = ds_queue.top().first;
            ds_queue.pop();
            // Is s_w never less than dist_s[s_u] ?
            if (s_w > dist_s[s_u]) continue;
            if (dist_s[s_u] < min_dist) {
                for (auto& edge : contracted_graph_[s_u]) {
                    if (invert_order_[s_u] < invert_order_[edge.first]) {
                        s_w = edge.second + dist_s[s_u];
                        if (dist_s[edge.first] <= s_w) continue;
                        dist_s[edge.first] = s_w;
                        ds_queue.push({s_w, edge.first});
                    }
                }
                min_dist = std::min(min_dist, dist_s[s_u] + dist_t[s_u]);
            }
        }

        while (!dt_queue.empty()) {
            t_u = dt_queue.top().second;
            t_w = dt_queue.top().first;
            dt_queue.pop();

            if (t_w > dist_t[t_u]) continue;
            if (dist_t[t_u] < min_dist) {
                for (auto& edge : contracted_graph_[t_u]) {
                    if (invert_order_[t_u] < invert_order_[edge.first]) {
                        t_w = edge.second + dist_t[t_u];
                        if (dist_t[edge.first] <= t_w) continue;
                        dist_t[edge.first] = t_w;
                        dt_queue.push({t_w, edge.first});
                    }
                }
                min_dist = std::min(min_dist, dist_t[t_u] + dist_t[t_u]);
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
    LOG(INFO) << "finish loading order file";
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
}

void Ch::write_index() {
    std::ofstream fs(index_file_);

    fs << contracted_graph_.size();

    for (int i = 0; i < contracted_graph_.size(); ++i) {
        for (auto& edge : contracted_graph_[i]) {
            fs << edge.first << " " << edge.second << std::endl;
        }
    }
    fs.close();
}

#endif  // ALGO_CH_H_
