#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  
  ros::Rate rate(100);
  while(ros::ok()){
    
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0,0, 1) );
    tf::Quaternion q;
    q.setRPY(1.57, 1.57, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "my_kinect_link", "my_kinect"));

    rate.sleep();

  }
    
  return 0;
}
