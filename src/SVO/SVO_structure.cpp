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
String Voxel::to_string() const {
    return vformat("Voxel(State: %s, Size: %f)", state_to_string(state), size);
}
String Voxel::state_to_string(VoxelState state) const {
    switch (state) {
    case VS_EMPTY: return "EMPTY";
    case VS_SOLID: return "SOLID";
    case VS_LIQUID: return "LIQUID";
    default: return "ERROR_UNKNOWN";
    }
}

void OctreeNode::_bind_methods()
{

}
OctreeNode::OctreeNode():
    currentDepth(1), currentIndex(0), isLeaf(true), voxel(nullptr), debugChecked(false)
{
    father = nullptr;
    for (int i = 0; i < 8; ++i) {
        children[i] = nullptr;
    }
    for (int i = 0; i < 6; ++i) {
        neighbors[i] = nullptr;
    }
    center = Vector3(0, 0, 0);
}
OctreeNode::OctreeNode(OctreeNode* father_node, int depth, int index) :
    currentDepth(depth), currentIndex(index), isLeaf(true), voxel(nullptr), 
    debugChecked(false), debugMesh(nullptr)
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

    call_deferred("queue_free_debug_mesh");

    for (int i = 0; i < 8; ++i) {
        if (children[i] != nullptr) {
            //UtilityFunctions::print(vformat("Deleting child: %s", get_voxel_info()));
            memdelete(children[i]);
            children[i] = nullptr;
        }
        else {
            //UtilityFunctions::print(vformat("Child index %d is already null", i));
        }
    }
}
void OctreeNode::queue_free_debug_mesh() {
    if (debugMesh->is_inside_tree()) {
        debugMesh->queue_free();
    }
}

String OctreeNode::get_voxel_info() const{
    String voxel_info = (voxel != nullptr) ? voxel->to_string() : "Voxel: null";
    return vformat("OctreeNode(Leaf: %s, Depth: %d, Index: %d, Center: %s, %s)",
        isLeaf ? "true" : "false",
        currentDepth,
        currentIndex,
        center,
        voxel_info);
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
    if(root) memdelete(root);
}

float SparseVoxelOctree::calActualVoxelSize(int depth) {
    return voxelSize / (1 << depth);
}

void updateCenterCoordinates(Vector3& center, int index, float halfSize) {
    if (index & 1) center.x += halfSize / 2;  // X����
    else center.x -= halfSize / 2;

    if (index & 2) center.y += halfSize / 2;  // Y����
    else center.y -= halfSize / 2;

    if (index & 4) center.z += halfSize / 2;  // Z����
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
    node->isLeaf = false; // Make sure the node is marked as a non-leaf node; ȷ���ڵ㱻���Ϊ��Ҷ�ڵ�

    updateCenterCoordinates(center, index, halfSize);
    insert(node->children[index], pos, center, depth + 1);

    // Re-evaluate homogeneity, make sure the parent node status is correct
    // ��������ͬ���ԣ�ȷ�����ڵ�״̬��ȷ
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
        // its not possible to have full empty right after insert
    }
}

bool SparseVoxelOctree::query(Vector3 pos) const {
    return query(root, pos);
}
bool SparseVoxelOctree::query(OctreeNode* node, Vector3 pos) const {
    if (!node || !node->voxel) {
        return false;
    }

    if (node->voxel->isEmpty()) {
        return false;
    }

    if (node->voxel->isSolid()) {
        return true;
    }

    if (node->currentDepth == maxDepth) {
        return node->voxel->isSolid();
    }

    int index = (pos.x >= node->center.x) | ((pos.y >= node->center.y) << 1) | ((pos.z >= node->center.z) << 2);
    return query(node->children[index], pos);
}
OctreeNode* SparseVoxelOctree::get_deepest_node(Vector3 pos) {
    OctreeNode* node = root;
    while (node && !node->isLeaf) {
        int index = (pos.x >= node->center.x) | ((pos.y >= node->center.y) << 1) | ((pos.z >= node->center.z) << 2);
        if (node->children[index]) {
            node = node->children[index];
        }
        else {
            break;  
        }
    }
    return node;
}


//��ʱ����; Temp Abandon
void SparseVoxelOctree::update(Vector3 pos, bool isSolid) {
    update(root, pos, Vector3(0,0,0), 1, isSolid ? VS_SOLID : VS_EMPTY);
}
//��ʱ����; Temp Abandon
void SparseVoxelOctree::update(OctreeNode* node, Vector3 pos, Vector3 center, int depth, VoxelState newState) {
    if (!node) return;

    // ����Ƿ�ﵽ������
    if (depth == maxDepth) {
        if (!node->voxel) {
            node->voxel = memnew(Voxel(newState, calActualVoxelSize(depth)));
        }
        else {
            node->voxel->state = newState; // �������ص�״̬
        }
        node->isLeaf = true;
        return;
    }

    // ������������Ϊ��һ��
    float halfSize = voxelSize / pow(2, depth);
    int index = (pos.x >= center.x) | ((pos.y >= center.y) << 1) | ((pos.z >= center.z) << 2);
    updateCenterCoordinates(center, index, halfSize);

    // ��������������ӽڵ�
    if (!node->children[index]) {
        node->children[index] = memnew(OctreeNode(node,depth + 1,index));
        node->children[index]->voxel = memnew(Voxel(VS_EMPTY, calActualVoxelSize(depth + 1)));
        node->children[index]->center = center;
    }

    node->isLeaf = false; // ȷ���ڵ㱻���Ϊ��Ҷ�ڵ�

    // �ݹ�����ӽڵ�
    update(node->children[index], pos, center, depth + 1, newState);

    // ���¼�鲢����ͬ���Ժ�ռ��״̬
    reevaluate_homogeneity_update(node);
}
//��ʱ����; Temp Abandon
void SparseVoxelOctree::reevaluate_homogeneity_update(OctreeNode* node) {
    if (!node || node->isLeaf) return; // ����ڵ㲻���ڻ���Ҷ�ڵ㣬��������

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
            allChildrenHomogeneousSolid = false; // �����һ�ӽڵ㲻���ڣ��򸸽ڵ㲻������ȫͬ�ʵ�ʵ��
        }
    }

    if (!anyChildExists) {
        // ���û���κ��ӽڵ���ڣ����ڵ�Ӧ���ǿյ�
        memdelete(node->voxel);
        node->voxel = memnew(Voxel(VS_EMPTY, calActualVoxelSize(node->currentDepth)));
    }
    else if (allChildrenHomogeneousSolid) {
        // �����ӽڵ㶼��ͬ�ʵ�ʵ��
        if (node->voxel) node->voxel->state = VS_SOLID;
        else node->voxel = memnew(Voxel(VS_SOLID, calActualVoxelSize(node->currentDepth)));
    }
    else if (anyChildSolid) {
        // ����һ���ӽڵ���ʵ�壨ռ�ݻ򲿷�ռ�ݣ�
        if (node->voxel) node->voxel->state = VS_LIQUID;
        else node->voxel = memnew(Voxel(VS_LIQUID, calActualVoxelSize(node->currentDepth)));
    }
    else {
        // �����ӽڵ㶼���ڣ������ǿյ�
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
    // ����Ѵﵽ�µ������ȣ���ʼ�ϲ��ӽڵ�
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

    if (depth < maxDepth && node->voxel->isSolid() && node->isLeaf) {
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
        // TODEBUG: ���û���ӽڵ�ռ�ݣ��򸸽ڵ�Ҳ��ռ��
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
        //if (depth + 1 != maxDepth) node->children[i]->isLeaf = false;

        // Calculate the center of the child node
        // �����ӽڵ������
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
        // �����ӽڵ������
        float halfSize = voxelSize / pow(2, depth);
        Vector3 childCenter = node->center + octant_offset(i, halfSize);
        node->children[i]->center = childCenter;
    }
}
void SparseVoxelOctree::evaluate_homogeneity(OctreeNode* node) {
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
        if (!node->voxel) node->voxel = memnew(Voxel(VS_SOLID, calActualVoxelSize(node->currentDepth)));
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
        if (!node->voxel) node->voxel = memnew(Voxel(VS_EMPTY, calActualVoxelSize(node->currentDepth)));
        else {
            node->voxel->state = VS_EMPTY;
        }
        node->isLeaf = true;
        deleteChildren(node);
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
    // ɾ��������
    memdelete(root);
    // Reinitialize the root node
    // ���³�ʼ�����ڵ�
    root = memnew(OctreeNode(nullptr,1,0));
    root->voxel = memnew(Voxel(VS_EMPTY, voxelSize));
    create_empty_children(root, 1);
}
