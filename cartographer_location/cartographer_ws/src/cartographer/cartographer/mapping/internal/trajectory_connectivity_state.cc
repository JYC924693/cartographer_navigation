/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/trajectory_connectivity_state.h"

namespace cartographer {
namespace mapping {

// 添加一个最初只连接到自身的轨迹
void TrajectoryConnectivityState::Add(const int trajectory_id) {
  connected_components_.Add(trajectory_id);
}

// 连接两条轨迹. 如果任一轨迹未跟踪, 则将对其进行跟踪. 
// 此函数对其参数的顺序是不变的. 重复调用 Connect 会增加连接计数并更新上次连接时间
void TrajectoryConnectivityState::Connect(const int trajectory_id_a,
                                          const int trajectory_id_b,
                                          const common::Time time) {
  if (TransitivelyConnected(trajectory_id_a, trajectory_id_b)) {
    // The trajectories are transitively connected, i.e. they belong to the same
    // connected component. In this case we only update the last connection time
    // of those two trajectories.
    auto sorted_pair = std::minmax(trajectory_id_a, trajectory_id_b);
    if (last_connection_time_map_[sorted_pair] < time) {
      last_connection_time_map_[sorted_pair] = time;
    }
  } else {
    // The connection between these two trajectories is about to join to
    // connected components. Here we update all bipartite trajectory pairs for
    // the two connected components with the connection time. This is to quickly
    // change to a more efficient loop closure search (by constraining the
    // search window) when connected components are joined.
    // 这两条轨迹之间的连接即将加入连接的组件.在这里, 我们使用连接时间更新两个连接组件的所有二分轨迹对
    // 这是为了在连接组件连接时快速更改为更有效的回环搜索（通过约束搜索窗口）.
    std::vector<int> component_a =
        connected_components_.GetComponent(trajectory_id_a);
    std::vector<int> component_b =
        connected_components_.GetComponent(trajectory_id_b);
    for (const auto id_a : component_a) {
      for (const auto id_b : component_b) {
        auto id_pair = std::minmax(id_a, id_b);
        last_connection_time_map_[id_pair] = time;
      }
    }
  }
  connected_components_.Connect(trajectory_id_a, trajectory_id_b);
}

// 确定两个轨迹是否已经（传递）连接. 如果没有跟踪任一轨迹, 则返回 false, 除非它是相同的轨迹, 
// 否则返回 true. 此函数对其参数的顺序是不变的. 
bool TrajectoryConnectivityState::TransitivelyConnected(
    const int trajectory_id_a, const int trajectory_id_b) const {
  return connected_components_.TransitivelyConnected(trajectory_id_a,
                                                     trajectory_id_b);
}

// 轨迹ID, 按连通性分组
std::vector<std::vector<int>> TrajectoryConnectivityState::Components() const {
  return connected_components_.Components();
}

// 返回两个轨迹之间的最后一个连接计数. 如果任一轨迹未跟踪或它们从未连接过, 则返回时间的开始
common::Time TrajectoryConnectivityState::LastConnectionTime(
    const int trajectory_id_a, const int trajectory_id_b) {
  const auto sorted_pair = std::minmax(trajectory_id_a, trajectory_id_b);
  return last_connection_time_map_[sorted_pair];
}

}  // namespace mapping
}  // namespace cartographer
