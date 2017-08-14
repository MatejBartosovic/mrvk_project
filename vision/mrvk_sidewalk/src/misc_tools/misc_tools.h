#ifndef PROJECT_MISCTOOLS_H
#define PROJECT_MISCTOOLS_H

//looking for home directory
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>

std::string get_directory(std::string file_directory_home, std::string file_name, std::string file_num, std::string file_type);

#endif //PROJECT_MISCTOOLS_H