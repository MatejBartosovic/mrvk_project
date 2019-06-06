//
// Created by root on 13.4.2019.
//

#include "../include/mrvk_sidewalk/FilterCloud.h"

#define MAP_WIDTH 1000
#define MAP_HEIGHT 1000
#define MAP_CELL_SIZE 0.25

FilterCloud::FilterCloud()
{


    // topic names
    std::string pav_map_topic = "pav_map";
    std::string cloud_topic = "pav_pointCloud";
    std::string pav_map_cloud_topic = "pav_map_cloud";
    n_.getParam("sidewalk_params/pav_map_topic", pav_map_topic);
    n_.getParam("sidewalk_params/cloud_topic", cloud_topic);
    n_.getParam("sidewalk_params/pav_map_cloud_topic", pav_map_cloud_topic);

    // Publisher
    pav_map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>(pav_map_topic, 1);
    pav_map_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>(pav_map_cloud_topic, 1);
    // Subscriber
    cloud_sub_ = n_.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1, &FilterCloud::cloudCallback, this);

    constructMap();
}

void FilterCloud::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //plate_angle_ = msg->position;
    //ROS_ERROR("Rcvd pointcloud.");
    pav_map_pub_.publish(pav_map);
    pav_map_cloud_pub_.publish(msg);

}

void FilterCloud::constructMap()
{
    int getParamValInt = 0;
    float getParamValFlt = 0;
    pav_map.header.stamp = ros::Time::now();
    pav_map.header.frame_id = "map"; //todo parameter
    pav_map.info.map_load_time = ros::Time::now();
    getParamValFlt = MAP_CELL_SIZE;
    n_.getParam("pav_map/cell_size", getParamValFlt);
    pav_map.info.resolution = getParamValFlt;
    getParamValInt = MAP_WIDTH;
    n_.getParam("pav_map/width", getParamValInt);
    pav_map.info.width = (unsigned int)getParamValInt;
    getParamValInt = MAP_HEIGHT;
    n_.getParam("pav_map/height", getParamValInt);
    pav_map.info.height = (unsigned int)getParamValInt;

    //pav_map.info.origin.

    //construct occupationGrid
    for (int i = 0; i < pav_map.info.height; i++)
    {
        for (int ii = 0; ii < pav_map.info.width; ii++)
        {
            pav_map.data.push_back(0);
        }
    }
    pav_map_pub_.publish(pav_map);


}

FilterCloud::~FilterCloud()
{

}






int main(int argc, char **argv)
{

    ros::init(argc, argv, "filter_cloud");


    ros::NodeHandle n;

    FilterCloud filterCloud;

    ros::spin();


    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);


    int count = 0;
    while (ros::ok())
    {

        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());


        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
