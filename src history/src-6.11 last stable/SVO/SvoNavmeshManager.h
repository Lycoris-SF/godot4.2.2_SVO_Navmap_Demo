#pragma once

#include "SvoNavmesh.h"
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/templates/vector.hpp>

namespace godot {

    class SvoNavmeshManager {
    private:
        Vector<SvoNavmesh*> navmeshes;
        SvoNavmesh* find_navmesh_containing(Vector3 position) const;

    public:
        SvoNavmeshManager();
        ~SvoNavmeshManager();

        void add_navmesh(SvoNavmesh* navmesh);
        bool query(Vector3 position) const;
        Vector<Vector3> find_path(Vector3 start, Vector3 end) const;
    };

}
