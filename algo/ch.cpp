#include <functional>
#include <memory>
#include <queue>

#include "basic_algo.h"
struct dgree_cmp {
  bool operator()(const int& a, const int& b) { return a < b; }
};

class ch : public basic_algo {
 public:
  void processing();
  void load_index(std::string index_file);
  void write_index(std::string index_file);
  int query(vid_t v, vid_t u);
  void batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists);

  void contraction();
  void load_order(std::string order_file);
  void write_order(std::string order_file);

  void generate_order();

 private:
  void contract_node(vid_t vid);
  int bi_dijkstra(vid_t src, vid_t dst);
  std::vector<w_t>& limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop);

  std::vector<bool> contracted_;
  std::vector<vid_t> order_;
  std::vector<vid_t> invert_order_;
  std::string order_file_;
  std::vector<std::map<vid_t, vid_t>> shortcut_node_;
};

void ch::processing() {
  perf::watch watch;
  watch.mark("t1");
  contracted_.resize(graph_->get_v_size(), false);

  LOG(INFO) << "ch start contracting";

  contraction();

  LOG(INFO) << "ch finish contracting, time cost:" << watch.showlit_mills("t1")
            << " ms";
}

void ch::load_index(std::string index_file) {}
void ch::write_index(std::string index_file) {}
int ch::query(vid_t v, vid_t u) {}
void ch::batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) {
}

void ch::contraction() {
  for (auto v : order_) {
    contract_node(v);
  }
}

void ch::load_order(std::string order_file) {
  std::ifstream fs(order_file);
  order_.resize(graph_->get_v_size());
  for (vid_t i = 0; i < order_.size(); ++i) {
    fs >> order_[i];
    invert_order_[order_[i]] = i;
  }
  fs.close();
  LOG(INFO) << "finish loading order file";
}

void ch::write_order(std::string order_file) {
  assert(order_.size() == graph_->get_v_size());
  std::ofstream fs(order_file);
  for (auto v : order_) {
    fs << v;
  }
  fs.close();
  LOG(INFO) << "finsh writing order file";
}

void ch::generate_order() {}

void ch::contract_node(vid_t vid) {
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

int ch::bi_dijkstra(vid_t src, vid_t dst) {}

// max_hop = 2
std::vector<w_t>& ch::limit_dijkstra(vid_t u, vid_t max_dist, vid_t max_hop) {
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