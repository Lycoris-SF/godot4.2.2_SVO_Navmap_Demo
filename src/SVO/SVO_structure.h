#pragma once

#include <godot_cpp/variant/vector3.hpp>

namespace godot {
    enum VoxelState {
        VS_EMPTY,
        VS_SOLID,
        VS_LIQUID
    };

    class Voxel {
    public:
        VoxelState state;
        float size;
        Voxel();
        Voxel(VoxelState state, float voxel_size);
        bool isSolid();
    };

    class OctreeNode {
    public:
        bool isLeaf;
        bool isHomogeneous; // 表示该节点是否为同质节点
        int currentDepth;
        Vector3 center;
        Voxel* voxel;
        OctreeNode* children[8];
        OctreeNode* neighbors[6];

        OctreeNode(int depth = -1);
        ~OctreeNode();
        bool isEmpty() const;
    };

    class SparseVoxelOctree {
    public:
        int maxDepth;
        float voxelSize;
        OctreeNode* root;

        SparseVoxelOctree();
        SparseVoxelOctree(int max_depth, float voxelSize);
        ~SparseVoxelOctree();

        void insert(Vector3 pos);
        bool query(Vector3 pos) const;
        void update(Vector3 pos, bool isSolid);
        void update_node_centers();
        void compress_node();
        void expand_node();
        void clear();

    private:
        void insert(OctreeNode* node, Vector3 pos, Vector3 center, int depth);
        bool query(OctreeNode* node, Vector3 pos, Vector3 center, int depth) const;
        void update(OctreeNode* node, Vector3 pos, Vector3 center, int depth, VoxelState newState);
        // functional update
        void update_node_centers(OctreeNode* node, Vector3 center, int depth);
        void compress_node(OctreeNode* node, int depth);
        void expand_node(OctreeNode* node, int depth);
        void reevaluate_homogeneity_insert(OctreeNode* node);
        void reevaluate_homogeneity_update(OctreeNode* node);
        // delete
        void deleteChildren(OctreeNode* node);
        void deleteNode(OctreeNode* node);  // 递归删除节点
        // other tool
        float calActualVoxelSize(int depth); // 计算实际体素大小
        void merge_children(OctreeNode* node);
        void create_children(OctreeNode* node, int Depth);
    };

}
