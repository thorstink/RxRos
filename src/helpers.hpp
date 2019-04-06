#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

std::string get_pid()
{
  std::stringstream s;
  s << std::this_thread::get_id();
  return s.str();
}

void print_pid(const std::string& what = "not specified")
{
  printf("[thread %s] task %s\n", get_pid().c_str(), what.c_str());
}