#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include "SVO_structure.h"

namespace godot {

    class SvoNavmesh : public Node3D {
        GDCLASS(SvoNavmesh, Node3D)

    private:
        SparseVoxelOctree svo;
        double testdouble;

    protected:
        static void _bind_methods();

    public:
        SvoNavmesh();
        ~SvoNavmesh();

        void insert_voxel(Vector3 position);
        bool query_voxel(Vector3 position);

        void set_info(float p_info);
        float get_info();

        void _process(double delta);
        SparseVoxelOctree& get_svo();
    };

}