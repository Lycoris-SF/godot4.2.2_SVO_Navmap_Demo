#include "svo_structure.h"
#include "svo_navmesh.h"

using namespace godot;

Voxel::Voxel() { state = VS_EMPTY; size = 1.0f; }
Voxel::Voxel(VoxelState voxel_state = VS_EMPTY, float voxel_size = 1.0f):state(voxel_state), size(voxel_size){}
bool Voxel::isSolid()
{
    return state == VS_SOLID;
}
bool Voxel::isEmpty()
{
    return state == VS_EMPTY;
}
bool Voxel::isLiquid()
{
    return state == VS_LIQUID;
}

OctreeNode::OctreeNode(OctreeNode* father_node, int depth, int index) : 
    currentDepth(depth), currentIndex(index), isLeaf(true), voxel(nullptr)
{
    father = father_node;
    for (int i = 0; i < 8; ++i) {
        children[i] = nullptr;
    }
    for (int i = 0; i < 6; ++i) {
        neighbors[i] = nullptr;
    }
    center = Vector3(0, 0, 0);
}
OctreeNode::~OctreeNode() {
    if (voxel) memdelete(voxel); 
    if (!debugBoxMesh.is_null()) { 
        debugBoxMesh.unref(); 
    }
    debugMesh.queue_free();
    for (int i = 0; i < 8; ++i) {
        if (children[i]) memdelete(children[i]);  
        children[i] = nullptr;
    }
}

SparseVoxelOctree::SparseVoxelOctree() {
    maxDepth = 3;
    voxelSize = 1.0f;
    root = memnew(OctreeNode(nullptr, 1, 0));
    root->voxel = memnew(Voxel(VS_EMPTY, voxelSize));
    create_empty_children(root, 1);
}
SparseVoxelOctree::SparseVoxelOctree(int max_depth, float voxel_size)
    : maxDepth(max_depth), voxelSize(voxel_size){
    root = memnew(OctreeNode(nullptr,1,0));
    root->voxel = memnew(Voxel(VS_EMPTY, voxelSize));
    create_empty_children(root, 1);
}
SparseVoxelOctree::~SparseVoxelOctree() {
    memdelete(root);
}

float SparseVoxelOctree::calActualVoxelSize(int depth) {
    return voxelSize / (1 << depth);
}


void updateCenterCoordinates(Vector3& center, int index, float halfSize) {
    if (index & 1) center.x += halfSize / 2;  // X方向
    else center.x -= halfSize / 2;

    if (index & 2) center.y += halfSize / 2;  // Y方向
    else center.y -= halfSize / 2;

    if (index & 4) center.z += halfSize / 2;  // Z方向
    else center.z -= halfSize / 2;
}
Vector3 octant_offset(int index, float size) {
    return Vector3(
        (index & 1) ? size : -size,
        (index & 2) ? size : -size,
        (index & 4) ? size : -size
    ) / 2.0;
}

void SparseVoxelOctree::insert(Vector3 pos) {
    insert(root, pos, Vector3(0,0,0), 1);
}
void SparseVoxelOctree::insert(OctreeNode* node, Vector3 pos, Vector3 center, int depth) {
    if (depth == maxDepth) {
        if (!node->voxel) node->voxel = memnew(Voxel(VS_SOLID, calActualVoxelSize(depth)));
        else {
            node->voxel->state = VS_SOLID;
            node->voxel->size = calActualVoxelSize(depth);
        }
        node->isLeaf = true;
        return;
    }

    float halfSize = voxelSize / pow(2, depth);
    int index = (pos.x >= center.x) | ((pos.y >= center.y) << 1) | ((pos.z >= center.z) << 2);

    for (int i = 0; i < 8; ++i) {
        if (!node->children[i]) {
            node->children[i] = memnew(OctreeNode(node, depth + 1, i));
            node->children[i]->voxel = memnew(Voxel(VS_EMPTY, calActualVoxelSize(depth)));
            if (depth+1 != maxDepth) node->children[i]->isLeaf = false;

            Vector3 childCenter = center + octant_offset(i, halfSize);
            node->children[i]->center = childCenter;
        }
    }
    node->isLeaf = false; // Make sure the node is marked as a non-leaf node; 确保节点被标记为非叶节点

    updateCenterCoordinates(center, index, halfSize);
    insert(node->children[index], pos, center, depth + 1);

    // Re-evaluate homogeneity, make sure the parent node status is correct
    // 重新评估同质性，确保父节点状态正确
    reevaluate_homogeneity_insert(node);
}
void SparseVoxelOctree::reevaluate_homogeneity_insert(OctreeNode* node) {
    if (node->isLeaf) return;

    bool allSolid = true;
    bool anySolid = false;
    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            allSolid &= (node->children[i]->voxel->state == VS_SOLID);
            anySolid |= (node->children[i]->voxel->state != VS_EMPTY);
        }
        else {
            allSolid = false;
        }
    }

    if (allSolid) {
        if(!node->voxel) node->voxel = memnew(Voxel(VS_SOLID, calActualVoxelSize(node->currentDepth)));
        else {
            node->voxel->state = VS_SOLID;
        }
    }
    else if (anySolid) {
        if (!node->voxel) node->voxel = memnew(Voxel(VS_LIQUID, calActualVoxelSize(node->currentDepth)));
        else {
            node->voxel->state = VS_LIQUID;
        }
    }
    else {
        memdelete(node->voxel);
    }
}

bool SparseVoxelOctree::query(Vector3 pos) const {
    return query(root, pos, Vector3(0, 0, 0), 1);
}
bool SparseVoxelOctree::query(OctreeNode* node, Vector3 pos, Vector3 center, int depth) const {
    if (!node || !node->voxel) {
        return false;
    }

    if (node->voxel->isEmpty()) {
        return false;
    }

    if (node->voxel->isSolid()) {
        return true;
    }

    if (depth == maxDepth) {
        return node->voxel->isSolid();
    }

    float halfSize = voxelSize / pow(2, depth);
    int index = (pos.x >= center.x) | ((pos.y >= center.y) << 1) | ((pos.z >= center.z) << 2);
    updateCenterCoordinates(center, index, halfSize);

    return query(node->children[index], pos, center, depth + 1);
}

//临时弃用; Temp Abandon
void SparseVoxelOctree::update(Vector3 pos, bool isSolid) {
    update(root, pos, Vector3(0,0,0), 1, isSolid ? VS_SOLID : VS_EMPTY);
}
//临时弃用; Temp Abandon
void SparseVoxelOctree::update(OctreeNode* node, Vector3 pos, Vector3 center, int depth, VoxelState newState) {
    if (!node) return;

    // 检查是否达到最大深度
    if (depth == maxDepth) {
        if (!node->voxel) {
            node->voxel = memnew(Voxel(newState, calActualVoxelSize(depth)));
        }
        else {
            node->voxel->state = newState; // 更新体素的状态
        }
        node->isLeaf = true;
        return;
    }

    // 更新中心坐标为下一层
    float halfSize = voxelSize / pow(2, depth);
    int index = (pos.x >= center.x) | ((pos.y >= center.y) << 1) | ((pos.z >= center.z) << 2);
    updateCenterCoordinates(center, index, halfSize);

    // 创建或继续更新子节点
    if (!node->children[index]) {
        node->children[index] = memnew(OctreeNode(node,depth + 1,index));
        node->children[index]->voxel = memnew(Voxel(VS_EMPTY, calActualVoxelSize(depth + 1)));
        node->children[index]->center = center;
    }

    node->isLeaf = false; // 确保节点被标记为非叶节点

    // 递归更新子节点
    update(node->children[index], pos, center, depth + 1, newState);

    // 重新检查并更新同质性和占据状态
    reevaluate_homogeneity_update(node);
}
//临时弃用; Temp Abandon
void SparseVoxelOctree::reevaluate_homogeneity_update(OctreeNode* node) {
    if (!node || node->isLeaf) return; // 如果节点不存在或是叶节点，无需评估

    bool anyChildExists = false;
    bool allChildrenHomogeneousSolid = true;
    bool anyChildSolid = false;

    for (int i = 0; i < 8; ++i) {
        if (node->children[i]) {
            anyChildExists = true;
            if (!node->children[i]->voxel->isLiquid() || !node->children[i]->voxel || node->children[i]->voxel->state != VS_SOLID) {
                allChildrenHomogeneousSolid = false;
            }
            if (node->children[i]->voxel && node->children[i]->voxel->state != VS_EMPTY) {
                anyChildSolid = true;
            }
        }
        else {
            allChildrenHomogeneousSolid = false; // 如果任一子节点不存在，则父节点不能是完全同质的实体
        }
    }

    if (!anyChildExists) {
        // 如果没有任何子节点存在，父节点应该是空的
        memdelete(node->voxel);
        node->voxel = memnew(Voxel(VS_EMPTY, calActualVoxelSize(node->currentDepth)));
    }
    else if (allChildrenHomogeneousSolid) {
        // 所有子节点都是同质的实体
        if (node->voxel) node->voxel->state = VS_SOLID;
        else node->voxel = memnew(Voxel(VS_SOLID, calActualVoxelSize(node->currentDepth)));
    }
    else if (anyChildSolid) {
        // 至少一个子节点是实体（占据或部分占据）
        if (node->voxel) node->voxel->state = VS_LIQUID;
        else node->voxel = memnew(Voxel(VS_LIQUID, calActualVoxelSize(node->currentDepth)));
    }
    else {
        // 所有子节点都存在，但都是空的
        if (node->voxel) node->voxel->state = VS_EMPTY;
        else node->voxel = memnew(Voxel(VS_EMPTY, calActualVoxelSize(node->currentDepth)));
    }
}

void SparseVoxelOctree::update_node_centers()
{
    update_node_centers(root, Vector3(0, 0, 0), 1);
}
void SparseVoxelOctree::update_node_centers(OctreeNode* node, Vector3 center, int depth)
{
    float halfSize = voxelSize / pow(2, depth);
    node->center = center;

    for (int i = 0; i < 8; i++) {
        if (node->children[i]) {
            Vector3 childCenter = center + octant_offset(i, halfSize);
            update_node_centers(node->children[i], childCenter, depth + 1);
        }
    }
}

void SparseVoxelOctree::compress_node()
{
    compress_node(root, 1);
}
void SparseVoxelOctree::compress_node(OctreeNode* node, int depth)
{
    if (!node || depth > maxDepth) return;
    // If the new maximum depth has been reached, start merging child nodes
    // 如果已达到新的最大深度，开始合并子节点
    if (depth == maxDepth) {
        merge_children(node);
        deleteChildren(node);
        return;
    }

    for (int i = 0; i < 8; ++i) {
        if (node->children[i]) {
            compress_node(node->children[i], depth + 1);
        }
    }
}

void SparseVoxelOctree::expand_node()
{
    expand_node(root, 1);
}
void SparseVoxelOctree::expand_node(OctreeNode* node, int depth)
{
    if (!node) return;

    if (depth < maxDepth && node->voxel->isSolid()) {
        create_solid_children(node, depth);
    }
    for (int i = 0; i < 8; ++i) {
        if (node->children[i]) {
            expand_node(node->children[i], depth + 1);
        }
    }
}

void SparseVoxelOctree::merge_children(OctreeNode* node) {
    bool isOccupied = false;

    if (!node->voxel->isLiquid()) {
        node->isLeaf = true;
        return;
    }
    for (int i = 0; i < 8; ++i) {
        if (node->children[i] && !node->children[i]->voxel->isLiquid()) {
            isOccupied = isOccupied || (node->children[i]->voxel && !node->children[i]->voxel->isEmpty());
        }
    }

    if (isOccupied) {
        if (!node->voxel) {
            node->voxel = memnew(Voxel(VS_SOLID, calActualVoxelSize(node->currentDepth)));
        }
        else {
            node->voxel->state = VS_SOLID;
        }
        node->isLeaf = true;
    }
    else {
        // TODEBUG: 如果没有子节点占据，则父节点也不占据
        if (node->voxel) {
            memdelete(node->voxel);
        }
        node->isLeaf = false;
    }
}
void SparseVoxelOctree::create_empty_children(OctreeNode* node, int depth) {
    node->isLeaf = false;
    for (int i = 0; i < 8; ++i) {
        if(!node->children[i]) node->children[i] = memnew(OctreeNode(node, depth + 1, i));
        if (!node->children[i]->voxel) node->children[i]->voxel = memnew(Voxel(VS_EMPTY, calActualVoxelSize(depth)));
        else {
            node->children[i]->voxel->state = VS_EMPTY;
            node->children[i]->voxel->size = calActualVoxelSize(depth);
        }
        if (depth + 1 != maxDepth) node->children[i]->isLeaf = false;

        // Calculate the center of the child node
        // 计算子节点的中心
        float halfSize = voxelSize / pow(2, depth);
        Vector3 childCenter = node->center + octant_offset(i, halfSize);
        node->children[i]->center = childCenter;
    }
}
void SparseVoxelOctree::create_solid_children(OctreeNode* node, int depth) {
    node->isLeaf = false;
    for (int i = 0; i < 8; ++i) {
        if (!node->children[i]) node->children[i] = memnew(OctreeNode(node, depth + 1, i));
        if (!node->children[i]->voxel) node->children[i]->voxel = memnew(Voxel(VS_SOLID, calActualVoxelSize(depth)));
        else {
            node->children[i]->voxel->state = VS_SOLID;
            node->children[i]->voxel->size = calActualVoxelSize(depth);
        }
        if (depth + 1 != maxDepth) node->children[i]->isLeaf = false;

        // Calculate the center of the child node
        // 计算子节点的中心
        float halfSize = voxelSize / pow(2, depth);
        Vector3 childCenter = node->center + octant_offset(i, halfSize);
        node->children[i]->center = childCenter;
    }
}

void SparseVoxelOctree::deleteChildren(OctreeNode* node) {
    for (int i = 0; i < 8; ++i) {
        if (node->children[i]) {
            memdelete(node->children[i]); 
            node->children[i] = nullptr;
        }
    }
    /*for (int i = 0; i < 8; ++i) {
        delete node->children[i];
        node->children[i] = nullptr;
    }*/
}
void SparseVoxelOctree::clear() {
    // Delete the entire tree
    // 删除整个树
    memdelete(root);
    // Reinitialize the root node
    // 重新初始化根节点
    root = memnew(OctreeNode(nullptr,1,0));
    root->voxel = memnew(Voxel(VS_EMPTY, voxelSize));
    create_empty_children(root, 1);
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
