#include <functional>
#include <memory>
#include <queue>
#include <set>

#include "basic_algo.h"

class Ch : public BasicAlgo {
 public:
  void processing();
  void contraction();

  int query(vid_t v, vid_t u);
  void batCh_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists);

  void load_order(std::string order_file);
  void write_order(std::string order_file);

  void load_index(std::string index_file);
  void write_index(std::string index_file);
  void generate_order();

 private:
  void contract_node(vid_t vid);
  int bi_dijkstra(vid_t src, vid_t dst);
  std::vector<w_t>& limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop);

  std::vector<bool> contracted_;
  std::vector<vid_t> order_;
  std::vector<vid_t> invert_order_;
  std::string order_file_;
  std::string index_file_;
  std::vector<std::map<vid_t, vid_t>> shortcut_node_;
};

void Ch::processing() {
  perf::Watch watCh;

  contracted_.resize(graph_->get_v_size(), false);

  LOG(INFO) << " begin generate order:";

  watCh.mark("t1");

  if (order_.empty()) {
    generate_order();
    write_order(order_file_);
  } else {
    load_order(order_file_);
  }

  LOG(INFO) << "Ch start contracting";

  contraction();

  LOG(INFO) << "Ch finish contracting, time cost:" << watCh.showlit_mills("t1")
            << " ms";
}

void Ch::load_index(std::string index_file) {}
void Ch::write_index(std::string index_file) {}
int Ch::query(vid_t v, vid_t u) {}
void Ch::batCh_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) {
}

void Ch::contraction() {
  for (auto v : order_) {
    contract_node(v);
  }
}

void Ch::load_order(std::string order_file) {
  std::ifstream fs(order_file);
  order_.resize(graph_->v_size_);
  invert_order_.resize(graph_->v_size_);

  for (vid_t i = 0; i < order_.size(); ++i) {
    fs >> order_[i];
    invert_order_[order_[i]] = i;
  }
  fs.close();
  LOG(INFO) << "finish loading order file";
}

void Ch::write_order(std::string order_file) {
  assert(order_.size() == graph_->get_v_size());
  std::ofstream fs(order_file);
  for (auto v : order_) {
    fs << v;
  }
  fs.close();
  LOG(INFO) << "finsh writing order file";
}

std::vector<int> D, D2;

struct DegComp {
  int v_;
  DegComp(int v) { v_ = v; }
  bool operator<(const DegComp d) const {
    if (D[v_] != D[d.v_]) return D[v_] < D[d.v_];
    if (D2[v_] != D2[d.v_]) return D2[v_] < D2[d.v_];
    return v_ < d.v_;
  }
};

// default : degree
// Edge difference: The edge difference is the number of shortcuts added minus
// the number of original edges removed during contraction. Nodes with a lower
// edge difference are contracted first to minimize the total number of
// shortcuts in the graph.
void Ch::generate_order() {
  std::vector<std::vector<vw_pair>> neighbors = graph_->neighbors_;
  int v_size = graph_->v_size_;

  order_.resize(v_size, 0);
  invert_order_.resize(v_size, 0);

  // copy graph

  D.resize(v_size, 0);
  D2.resize(v_size, 0);

  std::vector<bool> removed(v_size, false);

  std::set<DegComp> deg;
  int degree;

  for (int i = 0; i < v_size; ++i) {
    degree = neighbors[i].size();
    if (degree != 0) {
      D[i] = degree;
      D2[i] = degree;
      deg.insert(DegComp(i));
    }
  }

  while (!deg.empty()) {
    vid_t con_v = deg.begin()->v_;
    order_.push_back(con_v);
    removed[con_v] = true; 

    

  }

  for (int i = 0; i < order_.size(); ++i) {
    invert_order_[order_[i]] = i;
  }
}

void Ch::contract_node(vid_t vid) {
  assert(!contracted_[vid]);
  contracted_[vid] = true;

  w_t max_dist = 0, sec_dist = 0;
  for (auto& pair : graph_->neighbors_[vid]) {
    if (contracted_[pair.first]) continue;
    if (pair.second > max_dist) {
      max_dist = pair.second;
    } else if (pair.second > sec_dist) {
      sec_dist = pair.second;
    }
  }
  max_dist += sec_dist;

  for (auto& pair : graph_->neighbors_[vid]) {
    if (contracted_[pair.first]) continue;
    std::vector<w_t>& dists = limit_dijkstra(pair.first, max_dist, 2);
    for (auto& neighbor : graph_->neighbors_[vid]) {
      if (invert_order_[pair.first] > invert_order_[neighbor.first]) continue;
      w_t total_w = pair.second + neighbor.second;
      if (total_w < dists[neighbor.second] || dists[neighbor.second] == -1) {
        // add shortcut
        graph_->neighbors_[pair.first].push_back({neighbor.first, total_w});
        graph_->neighbors_[neighbor.first].push_back({pair.first, total_w});
        // TODO add support vertex
      }
    }
  }
}

// max_hop = 2
std::vector<w_t>& Ch::limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop) {
  std::priority_queue<wv_pair, std::vector<wv_pair>, std::greater<wv_pair>>
      dist_queue;

  std::vector<w_t> dists(graph_->v_size_, -1);
  std::vector<int> hops(graph_->v_size_, 0);

  dist_queue.push({0, u});
  while (!dist_queue.empty()) {
    vid_t vid = dist_queue.top().second;
    w_t dist = dist_queue.top().first;

    if (dist > dists[vid]) continue;
    if (hops[vid] > max_hop) continue;
    if (dists[vid] > max_dist) break;

    for (auto& vw : graph_->neighbors_[vid]) {
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

  return dists;
}

int Ch::bi_dijkstra(vid_t src, vid_t dst) {}