//
// Created by smadas on 3.7.2017.
//

#include "misc_tools.h"

std::string get_directory(std::string file_directory_home, std::string file_name, std::string file_num, std::string file_type)
{
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string file_directory;
    file_directory = homedir;
    file_directory = file_directory + file_directory_home + file_name + file_num + "." + file_type;
    return file_directory;
}
