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
        children[i] = nullptr;  // ��ʼ�������ӽڵ�ָ��Ϊ nullptr
    }
    for (int i = 0; i < 6; ++i) {
        neighbors[i] = nullptr;  // ��ʼ�������ھ�ָ��Ϊ nullptr
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
    if (depth == maxDepth - 1) { // ȷ��ֻ������
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
        node->children[index] = new OctreeNode(depth + 1); // �����½ڵ㲢���ݵ�ǰ���
        node->isLeaf = false;
    }

    insert(node->children[index], x % halfSize, y % halfSize, z % halfSize, depth + 1);

    // ����ӽڵ��Ƿ�Ϊͬ�ʽڵ㣬�������ϲ�
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
        return false; // �սڵ��δռ������
    }
    if (node->isHomogeneous) {
        return node->voxel ? node->voxel->isSolid : false; // ͬ�ʽڵ�ֱ�ӷ��ؽ��
    }
    if (depth == maxDepth-1) {
        return node->voxel ? node->voxel->isSolid : false; // Ҷ�ڵ�
    }

    int halfSize = 1 << (maxDepth - depth - 1);
    int index = (x >= halfSize) | ((y >= halfSize) << 1) | ((z >= halfSize) << 2);

    return query(node->children[index], x % halfSize, y % halfSize, z % halfSize, depth + 1);
}

void SparseVoxelOctree::update(int x, int y, int z, bool isSolid) {
    update(root, x, y, z, 0, isSolid);
}

void SparseVoxelOctree::update(OctreeNode* node, int x, int y, int z, int depth, bool isSolid) {
    if (!node) return; // ����ڵ㲻���ڣ������κβ���

    if (depth == maxDepth-1) {
        if (!node->voxel) {
            node->voxel = new Voxel(isSolid, calActualVoxelSize(depth)); // ������ز����ڣ��򴴽�������
        }
        else {
            node->voxel->isSolid = isSolid; // �������ص�״̬
        }
        node->isLeaf = true;
        node->isHomogeneous = true; // �������ȣ��ڵ�����ͬ�ʵ�
        return;
    }

    int halfSize = 1 << (maxDepth - depth - 1);
    int index = (x >= halfSize) | ((y >= halfSize) << 1) | ((z >= halfSize) << 2);

    if (!node->children[index]) {
        node->children[index] = new OctreeNode(); // ����ӽڵ㲻���ڣ��򴴽��µ��ӽڵ�
        node->isLeaf = false;
    }

    update(node->children[index], x % halfSize, y % halfSize, z % halfSize, depth + 1, isSolid);

    // ��鲢���½ڵ��ͬ����״̬
    bool allHomogeneous = true;
    bool firstSolid = node->children[0] && node->children[0]->voxel ? node->children[0]->voxel->isSolid : false;

    for (int i = 0; i < 8; ++i) {
        if (!node->children[i] || !node->children[i]->isHomogeneous || (node->children[i]->voxel && node->children[i]->voxel->isSolid != firstSolid)) {
            allHomogeneous = false;
            break;
        }
    }

    if (allHomogeneous) {
        // ��������ӽڵ㶼��ͬ�ʵģ���ϲ���Щ�ӽڵ�
        deleteChildren(node); // ɾ�������ӽڵ�
        node->isLeaf = true;
        node->isHomogeneous = true;
        node->voxel = new Voxel(firstSolid, calActualVoxelSize(depth)); // �����µ����ر�ʾ�����ӽڵ��ͳһ״̬
    }
}

void SparseVoxelOctree::deleteChildren(OctreeNode* node) {
    for (int i = 0; i < 8; ++i) {
        if (node->children[i]) {
            deleteNode(node->children[i]);
            node->children[i] = nullptr; // ȷ��ָ�뱻��Ϊ nullptr������Ұָ��
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

        delete current->voxel;  // ɾ������
        delete current;  // ɾ����ǰ�ڵ�
    }
}

void SparseVoxelOctree::clear() {
    // ɾ��������
    deleteNode(root);
    // ���³�ʼ�����ڵ�
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
