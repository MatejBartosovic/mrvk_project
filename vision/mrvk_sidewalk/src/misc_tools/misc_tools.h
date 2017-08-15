#ifndef PROJECT_MISCTOOLS_H
#define PROJECT_MISCTOOLS_H

//looking for home directory
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>
#include <vector>

std::string get_directory(std::string file_directory_home, std::string file_name, std::string file_num, std::string file_type);
double intMedian(std::vector<int> data);
double doubleMedian(std::vector<double> data);
#endif //PROJECT_MISCTOOLS_H