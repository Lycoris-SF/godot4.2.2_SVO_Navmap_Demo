#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include "SVO_structure.h"

namespace godot {

    class SvoNavmesh : public Node3D {
        GDCLASS(SvoNavmesh, Node3D)

    private:
        SparseVoxelOctree *svo;
        Vector3 origin_position; // 体素空间的原点位置
        Quaternion origin_rotation; // 体素空间的原点旋转
        int maxDepth;
        float voxelSize; // 体素大小
        double testdouble;

        Vector3 worldToGrid(Vector3 world_position);

    protected:
        static void _bind_methods();

    public:
        SvoNavmesh();
        SvoNavmesh(float size, Vector3 position, Quaternion rotation);
        ~SvoNavmesh();

        void insert_voxel(Vector3 position);
        bool query_voxel(Vector3 position);

        //svo setting
        Vector3 get_origin_position() const;
        void set_origin_position(Vector3 position);
        Quaternion get_origin_rotation() const;
        void set_origin_rotation(Quaternion rotation);
        float get_voxel_size() const;
        void set_voxel_size(float size);
        int get_max_depth() const;
        void set_max_depth(int depth);
        SparseVoxelOctree& get_svo();
        void rebuild_svo();

        //bind test
        void set_info(float p_info);
        float get_info();

        void _process(double delta);
    };

}