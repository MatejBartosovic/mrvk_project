//
// Created by smadas on 3.7.2017.
//

#include "misc_tools.h"
#include <algorithm>
#include <ros/ros.h>

std::string get_directory(std::string file_directory_home, std::string file_name, std::string file_num, std::string file_type)
{
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string file_directory;
    file_directory = homedir;
    file_directory = file_directory + file_directory_home + file_name + file_num + "." + file_type;
    return file_directory;

}
double intMedian(std::vector<int> data)
{
    double median = 0;
    std::sort(data.begin(), data.end());
    if (data.size() > 1)
    {
        if (data.size()%2 == 0)
        {
            median = (data.at((int)(data.size()/2)) + data.at((int)(data.size()/2 - 1)))/2;
        }
        else
        {
            median = data.at((int)(data.size()/2));
        }
    }
    else if (data.size() > 0)
    {
        median = data.at(0);
    }
    return median;
}
double doubleMedian(std::vector<double> data)
{
    double median = 0;
    std::sort(data.begin(), data.end());
    if (data.size() > 1)
    {
        if (data.size()%2 == 0)
        {
            median = (data.at((int)(data.size()/2)) + data.at((int)(data.size()/2 - 1)))/2;
        }
        else
        {
            median = data.at((int)(data.size()/2));
        }
    }
    else if (data.size() > 0)
    {
        median = data.at(0);
    }
    return median;
}