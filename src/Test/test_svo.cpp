#include "test_svo.h"
#include "SVO/SVO_structure.h"

using namespace godot;

int main() {

    SparseVoxelOctree svo_test;  // ��������ʵ��

    // ��������Ӳ��Դ���
    // ���磺svo.insert(x, y, z); ���� svo.query(x, y, z);
    svo_test.insert(1, 1, 1);

    return 0;
}