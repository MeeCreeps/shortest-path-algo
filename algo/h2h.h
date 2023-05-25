#ifndef ALGO_H2H_H_
#define ALGO_H2H_H_

#include <math.h>

#include <algorithm>

#include "ch.h"
struct  Node {
    int parent;
    vid_t master_v;
    size_t height;

    std::vector<int> child;  // child node ids
    // std::vector<int> belongs;  // the nodes contains v
    std::vector<int> pivot;  // pivot vertex  ,  for path retrieval

    std::vector<int> pos;
    std::vector<w_t> dis;
};  // tree node

struct Tree {
    std::vector<Node> nodes;
    std::vector<int> vid2node;
    size_t height;  // tree height
    size_t width;   // tree width

    std::vector<int> EulerSeq;  // prepare for the LCA calculation
    std::vector<int> toRMQ;
    std::vector<std::vector<int>> RMQIndex;

    Tree() : height(0), width(0) {}
};

class H2H : public Ch {
   public:
    H2H(std::shared_ptr<Graph>& graph, std::string index_file, std::string order_file)
        : Ch(graph, index_file, order_file) {}

    H2H(std::string index_file) : Ch(index_file) {}

    void processing() override;

    void contraction() override;

    void build_tree();

    void build_index();

    inline w_t query(vid_t v, vid_t u) override;

    void batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) override;

    // void load_index() { read_original(); } override;
    // void write_index() { write_original(); } override;

    void write_original() override;
    void read_original() override;

    void write_binary() override;
    void read_binary() override;

    void statistics() override;

   private:
    int get_parent(vid_t v, std::vector<vw_pair>& neigh);

    inline int get_lca(int _p, int _q);

    void dfs_build_index(int p, std::vector<int>& list);
    // build index  for lca query.
    //  refer to  M. A. Bender and M. Farach-Colton. The lca problem revisited.
    //  In Proc. of LASTI’00 ,  pages 88–94 ,  2000
    void build_rmq();
    void rmq_dfs(int p);

    std::vector<std::vector<vw_pair>> remained_neigh;
    Tree tree_;
};

void H2H::processing() {
    std::ifstream fs(index_file_);
    if (fs.good()) {
        fs.close();
        load_index();
        // load_order(order_file_);
    } else {
        perf::Watch watch;
        VERBOSE(watch.mark("t-total");)
        build_ch_index();

        VERBOSE(watch.mark("t-tree");)
        build_tree();
        VERBOSE(LOG(INFO) << "build tree structure  time cost:" << watch.showlit_mills("t-tree") << "  , "
                          << watch.showlit_seconds("t-tree") << " ." << std::endl;)

        VERBOSE(watch.mark("t-index");)
        build_index();
        VERBOSE(LOG(INFO) << "build index array ,  time cost:" << watch.showlit_mills("t-index") << "  , "
                          << watch.showlit_seconds("t-index") << " ." << std::endl;

                LOG(INFO) << "build total index  time cost:" << watch.showlit_mills("t-total") << "  , "
                          << watch.showlit_seconds("t-total") << " ." << std::endl;)
        write_index();
    }
}

void H2H::contraction() {
    remained_neigh.resize(v_size_);
    for (auto v : order_) {
        for (auto& edge : cgraph_[v]) {
            remained_neigh[v].push_back(edge);
        }
        contract_node(v);
    }
}

void H2H::build_tree() {
    // follow the tree node
    int idx = order_.size() - 1;
    int v = order_[idx];

    while (v == -1) {
        idx--;
        v = order_[idx];
    }

    Node node;
    node.master_v = v;
    node.height = 1;
    node.parent = -1;
    tree_.nodes.push_back(node);
    tree_.vid2node.resize(v_size_, -1);
    tree_.vid2node[v] = 0;
    idx--;
    for (; idx >= 0; idx--) {
        v = order_[idx];
        int parent = get_parent(v, remained_neigh[v]);
        node.parent = parent;
        node.height = tree_.nodes[parent].height + 1;
        node.master_v = v;

        // add vid to node id list
        tree_.nodes[parent].child.push_back(tree_.nodes.size());

        tree_.vid2node[v] = tree_.nodes.size();
        tree_.nodes.push_back(node);
        tree_.width = std::max(tree_.width, remained_neigh[v].size());
        tree_.height = std::max(tree_.height, node.height);
    }
}

void H2H::build_index() {
    build_rmq();

    // initialize
    std::vector<int> list;
    list.push_back(tree_.nodes[0].master_v);
    tree_.nodes[0].pos.clear();
    tree_.nodes[0].pos.push_back(0);

    for (int i = 0; i < tree_.nodes[0].child.size(); i++) {
        dfs_build_index(tree_.nodes[0].child[i], list);
    }
}

void H2H::rmq_dfs(int p) {
    tree_.toRMQ[p] = tree_.EulerSeq.size();
    tree_.EulerSeq.push_back(p);
    for (int i = 0; i < tree_.nodes[p].child.size(); i++) {
        rmq_dfs(tree_.nodes[p].child[i]);
        tree_.EulerSeq.push_back(p);
    }
}

void H2H::dfs_build_index(int p, std::vector<int>& list) {
    // initialize
    vid_t vid = tree_.nodes[p].master_v;
    int nei_size = remained_neigh[vid].size();
    tree_.nodes[p].pos.assign(nei_size + 1, 0);
    tree_.nodes[p].dis.assign(list.size(), INF);

    // pos
    // map<int , Nei> Nmap; Nmap.clear();//shortcut infor ordered by the pos ID

    // TODO  sort by order ?
    for (int i = 0; i < nei_size; i++) {
        for (int j = 0; j < list.size(); j++) {
            if (remained_neigh[vid][i].first == list[j]) {
                tree_.nodes[p].pos[i] = j;
                tree_.nodes[p].dis[j] = remained_neigh[vid][i].second;
                break;
            }
        }
    }

    // add self
    tree_.nodes[p].pos[nei_size] = list.size();

    // dis
    for (int i = 0; i < nei_size; i++) {
        int x = remained_neigh[vid][i].first;
        int disvb = remained_neigh[vid][i].second;
        int k = tree_.nodes[p].pos[i];  // the kth ancestor is x

        for (int j = 0; j < list.size(); j++) {
            int y = list[j];  // the jth ancestor is y

            int z;  // the distance from x to y
            if (k != j) {
                // the distrance form kth ancestor to jth ancestor if (k<j)
                if (k < j)
                    z = tree_.nodes[tree_.vid2node[y]].dis[k];
                else if (k > j)
                    z = tree_.nodes[tree_.vid2node[x]].dis[j];

                if (tree_.nodes[p].dis[j] > z + disvb) {
                    tree_.nodes[p].dis[j] = z + disvb;
                }
            }
        }
    }

    // nested loop
    list.push_back(tree_.nodes[p].master_v);
    for (int i = 0; i < tree_.nodes[p].child.size(); i++) {
        dfs_build_index(tree_.nodes[p].child[i], list);
    }
    list.pop_back();
}

void H2H::build_rmq() {
    // EulerSeq.clear();
    tree_.toRMQ.assign(v_size_, 0);
    // RMQIndex.clear();
    rmq_dfs(0);
    tree_.RMQIndex.push_back(tree_.EulerSeq);

    int m = tree_.EulerSeq.size();
    for (int i = 2, k = 1; i < m; i = i * 2, k++) {
        std::vector<int> tmp;
        // tmp.clear();
        tmp.assign(m, 0);
        for (int j = 0; j < m - i; j++) {
            int x = tree_.RMQIndex[k - 1][j], y = tree_.RMQIndex[k - 1][j + i / 2];
            if (tree_.nodes[x].height < tree_.nodes[y].height)
                tmp[j] = x;
            else
                tmp[j] = y;
        }
        tree_.RMQIndex.push_back(tmp);
    }
}

int H2H::get_parent(vid_t v, std::vector<vw_pair>& neigh) {
    assert(!neigh.empty());
    int parent = tree_.vid2node[neigh[0].first];
    for (int i = 1; i < neigh.size(); ++i) {
        assert(tree_.vid2node[neigh[i].first] != -1);
        if (parent < tree_.vid2node[neigh[i].first]) {
            parent = tree_.vid2node[neigh[i].first];
        }
    }
    return parent;
}

inline int H2H::get_lca(int _p, int _q) {
    int p = tree_.toRMQ[_p], q = tree_.toRMQ[_q];
    if (p > q) std::swap(p, q);

    int len = q - p + 1;
    // int k = (int)std::log(len);
    // int i = std::pow(2, k);
    // if (i == k) {
    //     k--;
    //     i /= 2;
    // }
    int i = 1, k = 0;
    while (i * 2 < len) {
        i *= 2;
        k++;
    }
    q = q - i + 1;
    if (tree_.nodes[tree_.RMQIndex[k][p]].height < tree_.nodes[tree_.RMQIndex[k][q]].height)
        return tree_.RMQIndex[k][p];
    else
        return tree_.RMQIndex[k][q];
}

inline w_t H2H::query(vid_t v, vid_t u) {
    if (v == u) return 0;
    // if (invert_order_[v] == -1 || invert_order_[u] == -1) return INF;

    int n1 = tree_.vid2node[v], n2 = tree_.vid2node[u];
    int LCA = get_lca(n1, n2);
    // return LCA;
    if (LCA == n1)
        return tree_.nodes[n2].dis[tree_.nodes[n1].pos.back()];
    else if (LCA == n2)
        return tree_.nodes[n1].dis[tree_.nodes[n2].pos.back()];
    else {
        w_t tmp = INF;
        for (int i = 0; i < tree_.nodes[LCA].pos.size(); i++) {
            if (tmp > tree_.nodes[n1].dis[tree_.nodes[LCA].pos[i]] + tree_.nodes[n2].dis[tree_.nodes[LCA].pos[i]])
                tmp = tree_.nodes[n1].dis[tree_.nodes[LCA].pos[i]] + tree_.nodes[n2].dis[tree_.nodes[LCA].pos[i]];
        }
        return tmp;
    }
}

void H2H::batch_query(const std::vector<std::pair<vid_t, vid_t>>& v_pair_lists) {
    std::vector<w_t> weight;
    for (auto& p : v_pair_lists) {
        weight.push_back(query(p.first, p.second));
        SHOW_DIST(LOG(INFO) << " u:" << p.first << " v:" << p.second << " sp:" << weight.back();)
    }
}

void H2H::read_original() {
    std::ifstream fs(index_file_);
    int node_size, v_size, euler_s, rmq_s, rmqi_s;
    if (!fs.good()) {
        LOG(INFO) << "H2H index file is not exist !";
        exit(-1);
    }

    fs >> tree_.height >> tree_.width >> node_size >> v_size >> euler_s >> rmq_s >> rmqi_s;

    tree_.vid2node.resize(v_size);
    tree_.nodes.resize(node_size);
    tree_.EulerSeq.resize(euler_s);
    tree_.toRMQ.resize(rmq_s);
    tree_.RMQIndex.resize(rmqi_s);

    for (int i = 0; i < v_size; ++i) {
        fs >> tree_.vid2node[i];
    }

    for (int i = 0; i < node_size; ++i) {
        Node& node = tree_.nodes[i];
        int pos_s, dis_s, child_s, pivot_s;
        fs >> node.parent >> node.master_v >> node.height >> pos_s >> dis_s >> child_s >> pivot_s;

        node.pos.resize(pos_s);
        node.dis.resize(dis_s);
        node.child.resize(child_s);
        node.pivot.resize(pivot_s);

        for (int j = 0; j < pos_s; ++j) fs >> node.pos[j];
        for (int j = 0; j < dis_s; ++j) fs >> node.dis[j];
        for (int j = 0; j < child_s; ++j) fs >> node.child[j];
        for (int j = 0; j < pivot_s; ++j) fs >> node.pivot[j];
    }

    for (int i = 0; i < euler_s; ++i) fs >> tree_.EulerSeq[i];
    for (int i = 0; i < rmq_s; ++i) fs >> tree_.toRMQ[i];

    for (int i = 0; i < rmqi_s; ++i) {
        int i_size;
        fs >> i_size;
        tree_.RMQIndex[i].resize(i_size);
        for (int j = 0; j < i_size; ++j) {
            fs >> tree_.RMQIndex[i][j];
        }
    }
    fs.close();

    LOG(INFO) << "finish writing H2H tree index !";
}

void H2H::write_original() {
    std::ofstream fs(index_file_);

    // [height , width , node size  ,  vertex size , EulerSql_s ,  toRMQ_size ,  RMQIndex size ]
    fs << tree_.height << " " << tree_.width << " " << tree_.nodes.size() << " " << tree_.vid2node.size() << " "
       << tree_.EulerSeq.size() << " " << tree_.toRMQ.size() << " " << tree_.RMQIndex.size();
    fs << std::endl;

    // write vid2node
    for (int i = 0; i < tree_.vid2node.size(); ++i) {
        fs << tree_.vid2node[i] << " ";
    }

    fs << std::endl;

    // write nodes
    for (int i = 0; i < tree_.nodes.size(); ++i) {
        Node& node = tree_.nodes[i];
        fs << node.parent << " " << node.master_v << " " << node.height << " " << node.pos.size() << " "
           << node.dis.size() << " " << node.child.size() << " " << node.pivot.size() << std::endl;

        // pos vector
        for (auto p : node.pos) fs << p << " ";
        fs << std::endl;
        for (auto d : node.dis) fs << d << " ";
        fs << std::endl;
        for (auto c : node.child) fs << c << " ";
        fs << std::endl;
        for (auto p : node.pivot) fs << p << " ";
        fs << std::endl;
    }

    for (auto s : tree_.EulerSeq) fs << s << " ";
    fs << std::endl;

    for (auto rmq : tree_.toRMQ) fs << rmq << " ";
    fs << std::endl;

    for (auto& RMQi : tree_.RMQIndex) {
        fs << RMQi.size() << std::endl;

        for (auto i : RMQi) {
            fs << i << " ";
        }
        fs << std::endl;
    }

    fs.close();
    LOG(INFO) << "H2H load index finished ";
}

void H2H::statistics() {
    long long psize = 0, dsize = 0;

    for (int i = 0; i < tree_.nodes.size(); ++i) {
        Node& node = tree_.nodes[i];
        psize += node.pos.size();
        dsize += node.dis.size();
    }

    long long rmqs = 0;
    for (int i = 0; i < tree_.RMQIndex.size(); ++i) {
        rmqs += tree_.RMQIndex[i].size();
    }

    LOG(INFO) << "**********tree statistics**********" << std::endl;
    LOG(INFO) << "tree height:" << tree_.height << " , tree width:" << tree_.width << std::endl;

    LOG(INFO) << "tree pos size:" << psize << " , average:" << (double)psize / tree_.nodes.size()
              << " , mem usage:" << psize * sizeof(int) / 1024.0 / 1024.0 << "MB"
              << " , " << psize * sizeof(int) / 1024.0 / 1024.0 / 1024.0 << "GB" << std::endl;

    LOG(INFO) << "tree dis size:" << dsize << " , average:" << (double)dsize / tree_.nodes.size()
              << " , mem usage:" << dsize * sizeof(int) / 1024.0 / 1024.0 << "MB"
              << " , " << dsize * sizeof(int) / 1024.0 / 1024.0 / 1024.0 << "GB" << std::endl;

    LOG(INFO) << "**********LCA statistics**********" << std::endl;
    LOG(INFO) << "EulerSeq size:" << tree_.EulerSeq.size() << " , mem "
              << " , mem usage:" << tree_.EulerSeq.size() * sizeof(int) / 1024.0 / 1024.0 << "MB"
              << " , " << tree_.EulerSeq.size() * sizeof(int) / 1024.0 / 1024.0 / 1024.0 << "GB" << std::endl;
    LOG(INFO) << "toRMQ size:" << tree_.toRMQ.size() << " , mem "
              << " , mem usage:" << tree_.toRMQ.size() * sizeof(int) / 1024.0 / 1024.0 << "MB"
              << " , " << tree_.toRMQ.size() * sizeof(int) / 1024.0 / 1024.0 / 1024.0 << "GB" << std::endl;
    LOG(INFO) << "RMQIndex size: 1." << tree_.RMQIndex.size() << " 2." << rmqs
              << " , mem usage:" << rmqs * sizeof(int) / 1024.0 / 1024.0 << "MB"
              << " , " << rmqs * sizeof(int) / 1024.0 / 1024.0 / 1024.0 << "GB" << std::endl;

    double lca_mem = (rmqs + tree_.EulerSeq.size() + tree_.RMQIndex.size()) * sizeof(int) / 1024.0 / 1024.0;
    double index_mem = (dsize + psize) * sizeof(int) / 1024.0 / 1024.0;

    LOG(INFO) << "LCA total mem:" << lca_mem << "MB"
              << " , " << lca_mem / 1024.0 << "GB" << std::endl;

    LOG(INFO) << "tree index mem :" << index_mem << "MB"
              << " , " << index_mem / 1024 << "GB" << std::endl;

    LOG(INFO) << "total mem :" << (index_mem + lca_mem) << "MB"
              << " , " << (index_mem + lca_mem) / 1024 << "GB" << std::endl;

    perf::mem_status mstatus;
    perf::get_mem_usage(&mstatus);

    LOG(INFO) << "exact mem usage(system ):" << (double)mstatus.vm_rss / 1024.0 << "MB"
              << " , " << (double)mstatus.vm_rss / 1024.0 / 1024.0 << "GB" << std::endl;
}

void H2H::write_binary() {
    std::ofstream fs(index_file_, std::ios::binary);

    int node_size = tree_.nodes.size(), v_size = tree_.vid2node.size(), euler_s = tree_.EulerSeq.size(),
        rmq_s = tree_.toRMQ.size(), rmqi_s = tree_.RMQIndex.size();

    fs.write((char*)&tree_.height, sizeof(int));
    fs.write((char*)&tree_.width, sizeof(int));
    fs.write((char*)&node_size, sizeof(int));
    fs.write((char*)&v_size, sizeof(int));
    fs.write((char*)&euler_s, sizeof(int));
    fs.write((char*)&rmq_s, sizeof(int));
    fs.write((char*)&rmqi_s, sizeof(int));

    for (int i = 0; i < v_size; ++i) fs.write((char*)&tree_.vid2node[i], sizeof(int));

    for (int i = 0; i < node_size; ++i) {
        Node& node = tree_.nodes[i];

        int pos_s = node.pos.size(), dis_s = node.dis.size(), child_s = node.child.size(), pivot_s = node.pivot.size();
        fs.write((char*)&node.parent, sizeof(int));
        fs.write((char*)&node.master_v, sizeof(int));
        fs.write((char*)&node.height, sizeof(int));
        fs.write((char*)&pos_s, sizeof(int));
        fs.write((char*)&dis_s, sizeof(int));
        fs.write((char*)&child_s, sizeof(int));
        fs.write((char*)&pivot_s, sizeof(int));

        for (int i = 0; i < pos_s; ++i) fs.write((char*)&node.pos[i], sizeof(int));
        for (int i = 0; i < dis_s; ++i) fs.write((char*)&node.dis[i], sizeof(int));
        for (int i = 0; i < child_s; ++i) fs.write((char*)&node.child[i], sizeof(int));
        for (int i = 0; i < pivot_s; ++i) fs.write((char*)&node.pivot[i], sizeof(int));
    }

    for (int i = 0; i < euler_s; ++i) fs.write((char*)&tree_.EulerSeq[i], sizeof(int));
    for (int i = 0; i < rmq_s; ++i) fs.write((char*)&tree_.toRMQ[i], sizeof(int));

    for (int i = 0; i < rmqi_s; ++i) {
        int s = tree_.RMQIndex[i].size();
        fs.write((char*)&s, sizeof(int));

        for (int j = 0; j < s; ++j) fs.write((char*)&tree_.RMQIndex[i][j], sizeof(int));
    }

    fs.close();
    LOG(INFO) << "H2H load index finished ";
};

void H2H::read_binary() {
    std::ifstream fs(index_file_, std::ios::binary);
    int node_size, v_size, euler_s, rmq_s, rmqi_s;
    if (!fs.good()) {
        LOG(INFO) << "H2H index file is not exist !";
        exit(-1);
    }

    fs.read((char*)&tree_.height, sizeof(int));
    fs.read((char*)&tree_.width, sizeof(int));
    fs.read((char*)&node_size, sizeof(int));
    fs.read((char*)&v_size, sizeof(int));
    fs.read((char*)&euler_s, sizeof(int));
    fs.read((char*)&rmq_s, sizeof(int));
    fs.read((char*)&rmqi_s, sizeof(int));

    tree_.vid2node.resize(v_size);
    tree_.nodes.resize(node_size);
    tree_.EulerSeq.resize(euler_s);
    tree_.toRMQ.resize(rmq_s);
    tree_.RMQIndex.resize(rmqi_s);

    for (int i = 0; i < v_size; ++i) fs.read((char*)&tree_.vid2node[i], sizeof(int));

    for (int i = 0; i < node_size; ++i) {
        Node& node = tree_.nodes[i];
        int pos_s, dis_s, child_s, pivot_s;
        fs.read((char*)&node.parent, sizeof(int));
        fs.read((char*)&node.master_v, sizeof(int));
        fs.read((char*)&node.height, sizeof(int));
        fs.read((char*)&pos_s, sizeof(int));
        fs.read((char*)&dis_s, sizeof(int));
        fs.read((char*)&child_s, sizeof(int));
        fs.read((char*)&pivot_s, sizeof(int));

        node.pos.resize(pos_s);
        node.dis.resize(dis_s);
        node.child.resize(child_s);
        node.pivot.resize(pivot_s);
        for (int i = 0; i < pos_s; ++i) fs.read((char*)&node.pos[i], sizeof(int));
        for (int i = 0; i < dis_s; ++i) fs.read((char*)&node.dis[i], sizeof(int));
        for (int i = 0; i < child_s; ++i) fs.read((char*)&node.child[i], sizeof(int));
        for (int i = 0; i < pivot_s; ++i) fs.read((char*)&node.pivot[i], sizeof(int));
    }

    for (int i = 0; i < euler_s; ++i) fs.read((char*)&tree_.EulerSeq[i], sizeof(int));
    for (int i = 0; i < rmq_s; ++i) fs.read((char*)&tree_.toRMQ[i], sizeof(int));

    for (int i = 0; i < rmqi_s; ++i) {
        int i_size;
        fs.read((char*)&i_size, sizeof(int));

        tree_.RMQIndex[i].resize(i_size);
        for (int j = 0; j < i_size; ++j) {
            fs.read((char*)&tree_.RMQIndex[i][j], sizeof(int));
        }
    }
    fs.close();

    LOG(INFO) << "finish writing H2H tree index !";
};

#endif  // ALGO_H2H_H_
