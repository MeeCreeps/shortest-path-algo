#ifndef ALGO_H2H_H_
#define ALGO_H2H_H_

#include <algorithm>

#include "ch.h"
#include "sp_algo.h"

struct Node {
    std::vector<int> pos;
    std::vector<w_t> dis;

    std::vector<int> child;  // child node ids
    // std::vector<int> belongs;  // the nodes contains v

    size_t parent;
    vid_t master_v;
    size_t height;
    std::vector<int> pivot;  // pivot vertex , for path retrieval

};  // tree node

struct Tree {
    std::vector<Node> nodes;
    std::vector<int> vid2node;
    size_t height;  // tree height
    size_t wdith;   // tree width

    std::vector<int> EulerSeq;  // prepare for the LCA calculation
    std::vector<int> toRMQ;
    std::vector<std::vector<int>> RMQIndex;

    Tree() : height(0), wdith(0) {}
};

class H2H : public Ch {
   public:
    void processing() override;

    void contraction() override;

    void build_tree();

    void build_index();

    inline w_t query(vid_t v, vid_t u) override;

    void load_index(std::string index_file) override;
    void write_index(std::string index_file) override;

   private:
    int get_parent(vid_t v, std::vector<vw_pair>& neigh);
    int get_lca(int _p, int _q);

    void dfs_build_index(int p, std::vector<int>& list);
    // build index  for lca query.
    //  refer to  M. A. Bender and M. Farach-Colton. The lca problem revisited.
    //  In Proc. of LASTI’00, pages 88–94, 2000
    void build_rmq();
    void rmq_dfs(int p);

    std::vector<std::vector<vw_pair>> remained_neigh;
    Tree tree_;
};

void H2H::processing() {
    std::ifstream fs(index_file_);
    if (fs.good()) {
        fs.close();
        load_index(index_file_);
        load_order(order_file_);
    } else {
        build_ch_index();
        build_tree();
        build_index();
        write_index(index_file_);
    }
}

void H2H::contraction() {
    remained_neigh.resize(v_size_);
    for (auto v : order_) {
        for (auto& edge : contracted_graph_[v]) {
            if (contracted_[edge.first])
                continue;
            else
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
        tree_.wdith = std::max(tree_.wdith, remained_neigh[v].size());
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
    // map<int,Nei> Nmap; Nmap.clear();//shortcut infor ordered by the pos ID

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
        if (parent > tree_.vid2node[neigh[i].first]) {
            parent = tree_.vid2node[neigh[i].first];
        }
    }
    return parent;
}

int H2H::get_lca(int _p, int _q) {
    int p = tree_.toRMQ[_p], q = tree_.toRMQ[_q];
    if (p > q) {
        int x = p;
        p = q;
        q = x;
    }
    int len = q - p + 1;
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
    if (invert_order_[v] == -1 || invert_order_[u] == -1) return INF;

    int n1 = tree_.vid2node[v], n2 = tree_.vid2node[u];
    int LCA = get_lca(n1, n2);

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

void H2H::load_index(std::string index_file) {}
void H2H::write_index(std::string index_file) {}

#endif  // ALGO_H2H_H_
