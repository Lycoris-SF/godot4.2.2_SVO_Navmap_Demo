#include "SvoNavmesh.h"
#include "SVO_structure.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void SvoNavmesh::_bind_methods() {
    ClassDB::bind_method(D_METHOD("insert_voxel", "position"), &SvoNavmesh::insert_voxel);
    ClassDB::bind_method(D_METHOD("query_voxel", "position"), &SvoNavmesh::query_voxel);
    ClassDB::bind_method(D_METHOD("get_info"), &SvoNavmesh::get_info);
    ClassDB::bind_method(D_METHOD("set_info", "p_info"), &SvoNavmesh::set_info);
    ClassDB::add_property("SvoNavmesh", PropertyInfo(Variant::FLOAT, "testdouble"), "set_info", "get_info");
    //ClassDB::bind_method(D_METHOD("find_path", "start", "end"), &SvoNavmesh::find_path);
}

SvoNavmesh::SvoNavmesh() : svo(1.0f, 3) {
    testdouble = 1.14;
    // 初始化任何变量
}

SvoNavmesh::~SvoNavmesh() {
    // 添加清理代码
}

void SvoNavmesh::insert_voxel(Vector3 position) {
    svo.insert(position.x, position.y, position.z);
}

bool SvoNavmesh::query_voxel(Vector3 position) {
    return svo.query(position.x, position.y, position.z);
}

void SvoNavmesh::set_info(float p_info) {
    testdouble = p_info;
}

float SvoNavmesh::get_info() {
    return testdouble;
}

void SvoNavmesh::_process(double delta) {
    // 实现处理方法，如果需要的话
}

SparseVoxelOctree& SvoNavmesh::get_svo() {
    return svo;
}