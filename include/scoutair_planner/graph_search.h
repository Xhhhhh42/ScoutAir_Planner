#ifndef _GRAPH_SEARCH_H_
#define _GRAPH_SEARCH_H_

#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Eigen>

namespace scoutair_planner {
  
// GraphSearch that operates on different types of node using Dijkstra algorithm
// 实现了一个基于Dijkstra算法的通用图搜索类，可以用于任何符合NodeT接口要求的节点类型
template <typename NodeT>
class GraphSearch {
public:
  GraphSearch() {
    node_num_ = 0;
    edge_num_ = 0;
  }
  ~GraphSearch() {
  }

  void print();
  void addNode(const std::shared_ptr<NodeT>& node);
  void addEdge(const int& from, const int& to);
  void DijkstraSearch(const int& start, const int& goal, std::vector<std::shared_ptr<NodeT>>& path);

private:
  std::vector<std::shared_ptr<NodeT>> nodes_;
  int node_num_;
  int edge_num_;
};

template <typename NodeT>
class NodeCompare {
public:
  bool operator()(const std::shared_ptr<NodeT>& node1, const std::shared_ptr<NodeT>& node2) {
    return node1->g_value_ > node2->g_value_;
  }
};

// template <typename NodeT>
// void GraphSearch<NodeT>::print() {
//   for (auto v : nodes_) {
//     v->print();
//     v->printNeighbors();
//   }
// }

template <typename NodeT>
void GraphSearch<NodeT>::addNode(const std::shared_ptr<NodeT>& node) {
  nodes_.push_back(node);
  nodes_.back()->id_ = node_num_++;
}

template <typename NodeT>
void GraphSearch<NodeT>::addEdge(const int& from, const int& to) {
  nodes_[from]->neighbors_.push_back(nodes_[to]);
  ++edge_num_;
}

template <typename NodeT>
void GraphSearch<NodeT>::DijkstraSearch(const int& start, const int& goal,
                                        std::vector<std::shared_ptr<NodeT>>& path) {
  std::cout << "Node: " << node_num_ << ", edge: " << edge_num_ << std::endl;
  // Basic structure used by Dijkstra
  // std::unordered_map<int, int> close_set;
  std::priority_queue<std::shared_ptr<NodeT>, std::vector<std::shared_ptr<NodeT>>, NodeCompare<NodeT>> open_set;

  std::shared_ptr<NodeT> start_v = nodes_[start];
  std::shared_ptr<NodeT> end_v = nodes_[goal];
  start_v->g_value_ = 0.0;
  open_set.push(start_v);

  while (!open_set.empty()) {
    auto vc = open_set.top();
    open_set.pop();
    vc->closed_ = true;
    // close_set[vc->id_] = 1;

    // Check if reach target
    if (vc == end_v) {
      // std::cout << "Dijkstra reach target" << std::endl;
      std::shared_ptr<NodeT> vit = vc;
      while (vit != nullptr) {
        path.push_back(vit);
        vit = vit->parent_;
      }
      reverse(path.begin(), path.end());
      return;
    }
    for (auto vb : vc->neighbors_) {
      // Check if in close set
      if (vb->closed_) continue;

      // Add new node or updated node in open set
      double g_tmp = vc->g_value_ + vc->costTo(vb);
      if (g_tmp < vb->g_value_) {
        vb->g_value_ = g_tmp;
        vb->parent_ = vc;
        open_set.push(vb);
      }
    }
  }
}
}

#endif