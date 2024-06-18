#include "SVO_structure.h"
#include <godot_cpp/templates/vector.hpp>
#include <cmath>
#include <algorithm>
#include <queue>
#include <map>
#include <set>

using namespace godot;

Voxel::Voxel() { isSolid = false; size = 1.0f; }
Voxel::Voxel(bool solid = false, float voxel_size = 1.0f) : isSolid(solid), size(voxel_size) {}

OctreeNode::OctreeNode(int depth) : currentDepth(depth), isLeaf(true), isHomogeneous(false), voxel(nullptr) {
    for (int i = 0; i < 8; ++i) {
        children[i] = nullptr;  // 初始化所有子节点指针为 nullptr
    }
    for (int i = 0; i < 6; ++i) {
        neighbors[i] = nullptr;  // 初始化所有邻居指针为 nullptr
    }
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

SparseVoxelOctree::SparseVoxelOctree() {
    maxDepth = 3;
    voxelSize = 1.0f;
    root = new OctreeNode();
}

SparseVoxelOctree::SparseVoxelOctree(int max_depth, float voxelSize)
    : maxDepth(max_depth), voxelSize(voxelSize){
    root = new OctreeNode();
}

SparseVoxelOctree::~SparseVoxelOctree() {
    delete root;
}

void SparseVoxelOctree::insert(int x, int y, int z) {
    insert(root, x, y, z, 0);
}

void SparseVoxelOctree::insert(OctreeNode* node, int x, int y, int z, int depth) {
    if (depth == maxDepth - 1) { // 确保只有三层
        if (!node->voxel) {
            node->voxel = new Voxel(true, calActualVoxelSize(depth));
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
        node->children[index] = new OctreeNode(depth + 1); // 创建新节点并传递当前深度
        node->isLeaf = false;
    }

    insert(node->children[index], x % halfSize, y % halfSize, z % halfSize, depth + 1);

    // 检查子节点是否为同质节点，如果是则合并
    bool allHomogeneous = true;
    bool firstSolid = node->children[0] && node->children[0]->voxel && node->children[0]->voxel->isSolid;
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
        node->voxel = new Voxel(firstSolid, calActualVoxelSize(depth));
    }
}

bool SparseVoxelOctree::query(int x, int y, int z) const {
    return query(root, x, y, x, 0);
}

bool SparseVoxelOctree::query(OctreeNode* node, int x, int y, int z, int depth) const {
    if (!node || node->isEmpty()) {
        return false; // 空节点或未占用区域
    }
    if (node->isHomogeneous) {
        return node->voxel ? node->voxel->isSolid : false; // 同质节点直接返回结果
    }
    if (depth == maxDepth-1) {
        return node->voxel ? node->voxel->isSolid : false; // 叶节点
    }

    int halfSize = 1 << (maxDepth - depth - 1);
    int index = (x >= halfSize) | ((y >= halfSize) << 1) | ((z >= halfSize) << 2);

    return query(node->children[index], x % halfSize, y % halfSize, z % halfSize, depth + 1);
}

void SparseVoxelOctree::update(int x, int y, int z, bool isSolid) {
    update(root, x, y, z, 0, isSolid);
}

void SparseVoxelOctree::update(OctreeNode* node, int x, int y, int z, int depth, bool isSolid) {
    if (!node) return; // 如果节点不存在，则不做任何操作

    if (depth == maxDepth-1) {
        if (!node->voxel) {
            node->voxel = new Voxel(isSolid, calActualVoxelSize(depth)); // 如果体素不存在，则创建新体素
        }
        else {
            node->voxel->isSolid = isSolid; // 更新体素的状态
        }
        node->isLeaf = true;
        node->isHomogeneous = true; // 在最大深度，节点总是同质的
        return;
    }

    int halfSize = 1 << (maxDepth - depth - 1);
    int index = (x >= halfSize) | ((y >= halfSize) << 1) | ((z >= halfSize) << 2);

    if (!node->children[index]) {
        node->children[index] = new OctreeNode(); // 如果子节点不存在，则创建新的子节点
        node->isLeaf = false;
    }

    update(node->children[index], x % halfSize, y % halfSize, z % halfSize, depth + 1, isSolid);

    // 检查并更新节点的同质性状态
    bool allHomogeneous = true;
    bool firstSolid = node->children[0] && node->children[0]->voxel ? node->children[0]->voxel->isSolid : false;

    for (int i = 0; i < 8; ++i) {
        if (!node->children[i] || !node->children[i]->isHomogeneous || (node->children[i]->voxel && node->children[i]->voxel->isSolid != firstSolid)) {
            allHomogeneous = false;
            break;
        }
    }

    if (allHomogeneous) {
        // 如果所有子节点都是同质的，则合并这些子节点
        deleteChildren(node); // 删除所有子节点
        node->isLeaf = true;
        node->isHomogeneous = true;
        node->voxel = new Voxel(firstSolid, calActualVoxelSize(depth)); // 创建新的体素表示所有子节点的统一状态
    }
}

void SparseVoxelOctree::deleteChildren(OctreeNode* node) {
    for (int i = 0; i < 8; ++i) {
        if (node->children[i]) {
            deleteNode(node->children[i]);
            node->children[i] = nullptr; // 确保指针被置为 nullptr，避免野指针
        }
    }
    /*for (int i = 0; i < 8; ++i) {
        delete node->children[i];
        node->children[i] = nullptr;
    }*/
}

void SparseVoxelOctree::deleteNode(OctreeNode* node) {
    if (!node) return;

    godot::Vector<OctreeNode*> stack;
    stack.push_back(node);

    while (!stack.is_empty()) {
        OctreeNode* current = stack[stack.size() - 1];
        stack.remove_at(stack.size() - 1);

        for (int i = 0; i < 8; ++i) {
            if (current->children[i]) {
                stack.push_back(current->children[i]);
            }
        }

        delete current->voxel;  // 删除体素
        delete current;  // 删除当前节点
    }
}

void SparseVoxelOctree::clear() {
    // 删除整个树
    deleteNode(root);
    // 重新初始化根节点
    root = new OctreeNode();
}
float SparseVoxelOctree::calActualVoxelSize(int depth) {
    return voxelSize / (1 << depth);
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
