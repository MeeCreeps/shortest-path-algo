#include <memory>

#include "graph/graph.h"

class SPAlgo {
   public:
    virtual void processing() = 0;
    virtual void load_index(std::string index_file) = 0;
    virtual void write_index(std::string index_file);
    virtual w_t query(vid_t v, vid_t u);
    virtual void batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) {
        std::vector<w_t> weight;
        for (auto& p : v_pair_lists) {
            weight.push_back(query(p.first, p.second));
        }
    }

   protected:
    std::shared_ptr<Graph> graph_;
    std::string order_file_;
    std::string index_file_;
};
