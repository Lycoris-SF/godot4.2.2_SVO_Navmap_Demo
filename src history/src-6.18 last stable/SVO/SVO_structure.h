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

        void insert(float x, float y, float z);
        bool query(float x, float y, float z) const;
        void update(float x, float y, float z, bool isSolid);
        void clear();

    private:
        void insert(OctreeNode* node, float x, float y, float z, float centerX, float centerY, float centerZ, int depth);
        bool query(OctreeNode* node, float x, float y, float z, int depth) const;
        void update(OctreeNode* node, float x, float y, float z, int depth, bool isSolid);
        void deleteChildren(OctreeNode* node);
        void deleteNode(OctreeNode* node);  // 递归删除节点
        float calActualVoxelSize(int depth); // 计算实际体素大小
    };

}
