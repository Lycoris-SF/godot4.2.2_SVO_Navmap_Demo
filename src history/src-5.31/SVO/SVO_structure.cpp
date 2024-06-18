#include "SVO_structure.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <map>
#include <set>

using namespace godot;

OctreeNode::OctreeNode() : isLeaf(true), isHomogeneous(false), voxel(nullptr) {
    std::fill(std::begin(children), std::end(children), nullptr);
    std::fill(std::begin(neighbors), std::end(neighbors), nullptr);
}

OctreeNode::~OctreeNode() {
    delete voxel;
    for (int i = 0; i < 8; ++i) {
        delete children[i];
    }
}

bool OctreeNode::isEmpty() const {
    return isLeaf && !voxel;
}

SparseVoxelOctree::SparseVoxelOctree(float voxel_size, int max_depth, Vector3 origin)
    : voxelSize(voxel_size), maxDepth(max_depth), origin(origin) {
    root = new OctreeNode();
}

SparseVoxelOctree::~SparseVoxelOctree() {
    delete root;
}

void SparseVoxelOctree::insert(float x, float y, float z) {
    Vector3 grid_coords = worldToGrid(x, y, z);
    insert(root, static_cast<int>(grid_coords.x), static_cast<int>(grid_coords.y), static_cast<int>(grid_coords.z), 0);
}

void SparseVoxelOctree::insert(OctreeNode* node, int x, int y, int z, int depth) {
    if (depth == maxDepth) {
        if (!node->voxel) {
            node->voxel = new Voxel(true);
        }
        else {
            node->voxel->isSolid = true;
        }
        node->isLeaf = true;
        node->isHomogeneous = true;
        return;
    }

    int halfSize = 1 << (maxDepth - depth - 1);
    int index = (x >= halfSize) | ((y >= halfSize) << 1) | ((z >= halfSize) << 2);

    if (!node->children[index]) {
        node->children[index] = new OctreeNode();
        node->isLeaf = false;
    }

    insert(node->children[index], x % halfSize, y % halfSize, z % halfSize, depth + 1);

    // 检查子节点是否为同质节点，如果是则合并
    bool allHomogeneous = true;
    bool firstSolid = node->children[0]->voxel && node->children[0]->voxel->isSolid;
    for (int i = 0; i < 8; ++i) {
        if (!node->children[i] || !node->children[i]->isHomogeneous || (node->children[i]->voxel && node->children[i]->voxel->isSolid != firstSolid)) {
            allHomogeneous = false;
            break;
        }
    }
    if (allHomogeneous) {
        for (int i = 0; i < 8; ++i) {
            delete node->children[i];
            node->children[i] = nullptr;
        }
        node->isLeaf = true;
        node->isHomogeneous = true;
        node->voxel = new Voxel(firstSolid);
    }
}

Vector3 SparseVoxelOctree::worldToGrid(float x, float y, float z) const {
    return Vector3(
        (x - origin.x) / voxelSize,
        (y - origin.y) / voxelSize,
        (z - origin.z) / voxelSize
    );
}

bool SparseVoxelOctree::query(float x, float y, float z) const {
    Vector3 grid_coords = worldToGrid(x, y, z);
    return query(root, static_cast<int>(grid_coords.x), static_cast<int>(grid_coords.y), static_cast<int>(grid_coords.z), 0);
}

bool SparseVoxelOctree::query(OctreeNode* node, int x, int y, int z, int depth) const {
    if (!node || node->isEmpty()) {
        return false; // 空节点或未占用区域
    }
    if (node->isHomogeneous) {
        return node->voxel ? node->voxel->isSolid : false; // 同质节点直接返回结果
    }
    if (depth == maxDepth) {
        return node->voxel ? node->voxel->isSolid : false; // 叶节点
    }

    int halfSize = 1 << (maxDepth - depth - 1);
    int index = (x >= halfSize) | ((y >= halfSize) << 1) | ((z >= halfSize) << 2);

    return query(node->children[index], x % halfSize, y % halfSize, z % halfSize, depth + 1);
}

/*std::vector<Vector3> SparseVoxelOctree::find_path(Vector3 start, Vector3 end) const {
    // A* algorithm
    std::priority_queue<std::pair<float, Vector3>, std::vector<std::pair<float, Vector3>>, std::greater<std::pair<float, Vector3>>> open_set;
    std::map<Vector3, Vector3> came_from;
    std::map<Vector3, float> g_score;
    std::map<Vector3, float> f_score;
    Vector3 start_grid = worldToGrid(start.x, start.y, start.z);
    Vector3 end_grid = worldToGrid(end.x, end.y, end.z);
    g_score[start_grid] = 0.0f;
    f_score[start_grid] = g_score[start_grid] + start_grid.distance_to(end_grid);
    open_set.emplace(f_score[start_grid], start_grid);
    while (!open_set.empty()) {
        Vector3 current = open_set.top().second;
        open_set.pop();
        if (current == end_grid) {
            return reconstruct_path(came_from, current);
        }
        std::vector<Vector3> neighbors = {
            Vector3(current.x + 1, current.y, current.z),
            Vector3(current.x - 1, current.y, current.z),
            Vector3(current.x, current.y + 1, current.z),
            Vector3(current.x, current.y - 1, current.z),
            Vector3(current.x, current.y, current.z + 1),
            Vector3(current.x, current.y, current.z - 1)
        };
        for (const Vector3& neighbor : neighbors) {
            if (!query(neighbor.x, neighbor.y, neighbor.z)) {
                continue;
            }
            float tentative_g_score = g_score[current] + current.distance_to(neighbor);
            if (tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = g_score[neighbor] + neighbor.distance_to(end_grid);
                open_set.emplace(f_score[neighbor], neighbor);
            }
        }
    }
    return {}; // No path found
}
std::vector<Vector3> SparseVoxelOctree::reconstruct_path(const std::map<Vector3, Vector3>& came_from, Vector3 current) const {
    std::vector<Vector3> total_path = { current };
    auto it = came_from.find(current);
    while (it != came_from.end()) {
        current = it->second;
        total_path.push_back(current);
        it = came_from.find(current);
    }
    std::reverse(total_path.begin(), total_path.end());
    return total_path;
}*/