#include <iostream>
#include <vector>
#include <stack>
#include <unordered_set>

struct Pose {
    double x;
    double y;
    double z;
};

struct RelativePose {
    int target_node; // 目标节点的索引
    Pose relative_pose; // 相对位姿
};

void dfs(int current_node, const Pose& current_pose, 
         const std::vector<std::vector<RelativePose>>& adjacency_list, 
         std::vector<Pose>& poses, 
         std::unordered_set<int>& visited) {
    visited.insert(current_node);
    poses[current_node] = current_pose;

    for (const auto& rel_pose : adjacency_list[current_node]) {
        if (visited.find(rel_pose.target_node) == visited.end()) {
            Pose new_pose = {
                current_pose.x + rel_pose.relative_pose.x,
                current_pose.y + rel_pose.relative_pose.y,
                current_pose.z + rel_pose.relative_pose.z
            };
            dfs(rel_pose.target_node, new_pose, adjacency_list, poses, visited);
        }
    }
}

int main() {
    std::vector<Pose> poses(10);
    poses[0] = {0.0, 0.0, 0.0}; // 第一个节点的位姿

    // 存储相对位姿
    std::vector<std::vector<RelativePose>> adjacency_list(10);

    // 添加相对位姿
    adjacency_list[0].push_back({1, {1.0, 0.0, 0.0}});
    adjacency_list[0].push_back({8, {0.0, 2.0, 0.0}});
    adjacency_list[1].push_back({2, {0.0, 1.0, 0.0}});
    adjacency_list[1].push_back({0, {-1.0, 0.0, 0.0}});
    // 继续添加其他节点的相对位姿...

    // 使用 DFS 推导其他节点的位姿
    std::unordered_set<int> visited;
    dfs(0, poses[0], adjacency_list, poses, visited);

    // 输出所有节点的位姿
    for (const auto& pose : poses) {
        std::cout << "Pose: (" << pose.x << ", " << pose.y << ", " << pose.z << ")\n";
    }

    return 0;
}
