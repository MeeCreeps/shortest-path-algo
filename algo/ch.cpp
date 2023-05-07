#include <memory>

#include "graph/graph.h"

struct dgree_cmp {
  bool operator()(const int& a, const int& b) { return a < b; }
};

class ch {
 public:
  void processing();
  void contraction();
  void query(vid_t u, vid_t v);

 private:
  void contract_node(vid_t vid);
  std::shared_ptr<graph> graph_;
  std::vector<bool> contracted_;
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

void ch::contract_node(vid_t vid) {}
