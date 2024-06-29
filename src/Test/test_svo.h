#pragma once

#include <iostream>  // �����������
#include <string>    // �������� std::string

#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/godot.hpp>

class TestLog {
public:
	TestLog();

	int test_countA;
	int test_countB;

	std::string log_textA;
	std::string log_textB;

	void test_print();
};