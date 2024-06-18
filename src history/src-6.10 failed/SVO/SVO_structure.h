#pragma once

#include <godot_cpp/variant/vector3.hpp>

class Voxel {
public:
    bool isSolid;
    float size;
    Voxel(bool solid = false, float voxel_size = 1.0f) : isSolid(solid), size(voxel_size) {}
};

class OctreeNode {
public:
    bool isLeaf;
    bool isHomogeneous; // 表示该节点是否为同质节点
    Voxel* voxel;
    OctreeNode* children[8];
    OctreeNode* neighbors[6];

    OctreeNode();
    ~OctreeNode();
    bool isEmpty() const;
};

namespace godot {

    class SparseVoxelOctree {
    public:
        int maxDepth;
        OctreeNode* root;

        SparseVoxelOctree();
        SparseVoxelOctree(int max_depth);
        ~SparseVoxelOctree();

        void insert(int x, int y, int z);
        bool query(int x, int y, int z) const;
        void update(int x, int y, int z, bool isSolid);
        void clear();

    private:
        void insert(OctreeNode* node, int x, int y, int z, int depth);
        bool query(OctreeNode* node, int x, int y, int z, int depth) const;
        void update(OctreeNode* node, int x, int y, int z, int depth, bool isSolid);
        void updateHomogeneity(OctreeNode* node);
        void deleteChildren(OctreeNode* node);
        void deleteNode(OctreeNode* node);  // 递归删除节点
    };

}
