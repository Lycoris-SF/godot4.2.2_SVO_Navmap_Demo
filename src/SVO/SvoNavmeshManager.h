#pragma once

#include "SvoNavmesh.h"
#include <godot_cpp/variant/vector3.hpp>

namespace godot {

    class SvoNavmeshManager {
    public:
        SvoNavmeshManager();
        ~SvoNavmeshManager();

        void add_navmesh(SvoNavmesh* navmesh);
        bool query(Vector3 position) const;
        std::vector<Vector3> find_path(Vector3 start, Vector3 end) const;

    private:
        std::vector<SvoNavmesh*> navmeshes;
        SvoNavmesh* find_navmesh_containing(Vector3 position) const;
    };

}
