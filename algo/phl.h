#ifndef ALGO_PHL_H_
#define ALGO_PHL_H_

#include "basic_algo.h"

class PHL {
 public:
  void processing();

  inline w_t query(vid_t v, vid_t u);
  void batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists);

  void load_order(std::string order_file);
  void write_order(std::string order_file);

  void load_index(std::string index_file);
  void write_index(std::string index_file);

 private:
};

void PHL::processing() {}

inline w_t PHL::query(vid_t v, vid_t u) {}
void PHL::batch_query(
    const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) {}

void PHL::load_order(std::string order_file) {}
void PHL::write_order(std::string order_file) {}

void PHL::load_index(std::string index_file) {}
void PHL::write_index(std::string index_file) {}

#endif  // ALGO_PHL_H_
