#pragma once

#include <godot_cpp/variant/vector3.hpp>

namespace godot {
    class Voxel {
    public:
        bool isSolid;
        float size;
        Voxel();
        Voxel(bool solid, float voxel_size);
    };

    class OctreeNode {
    public:
        bool isLeaf;
        bool isHomogeneous; // 表示该节点是否为同质节点
        int currentDepth;
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

        void insert(int x, int y, int z);
        bool query(int x, int y, int z) const;
        void update(int x, int y, int z, bool isSolid);
        void clear();

    private:
        void insert(OctreeNode* node, int x, int y, int z, int depth);
        bool query(OctreeNode* node, int x, int y, int z, int depth) const;
        void update(OctreeNode* node, int x, int y, int z, int depth, bool isSolid);
        void deleteChildren(OctreeNode* node);
        void deleteNode(OctreeNode* node);  // 递归删除节点
        float calActualVoxelSize(int depth); // 计算实际体素大小
    };

}
