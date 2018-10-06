//
// Created by matejko on 6.10.2018.
//

#include <wait_and_clear_costmap_recovery/wait_and_clear_costmap_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

//register this planner as a RecoveryBehavior plugin
//PLUGINLIB_EXPORT_CLASS(wait_and_clear_costmap_recovery::WaitAndClearCostmapRecovery, nav_core::RecoveryBehavior);


namespace wait_and_clear_costmap_recovery {
    WaitAndClearCostmapRecovery::WaitAndClearCostmapRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {} 

void WaitAndClearCostmapRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ROS_INFO("waccr name = %s",name_.c_str());
    private_nh.param("reset_distance", reset_distance_, 3.0);
    private_nh.param("check_distance", check_distance_, 0.8);
    private_nh.param("wait_time", wait_time_, 2.0);

    ROS_INFO("waccr reset_distance %lf",reset_distance_);
    ROS_INFO("waccr check_distance %lf",check_distance_);
    ROS_INFO("waccr wait_time %lf",wait_time_);

    std::vector<std::string> clearable_layers_default, clearable_layers;
    clearable_layers_default.push_back( std::string("obstacle_layer") );
    clearable_layers_default.push_back( std::string("inflation_layer") );
    private_nh.param("layer_names", clearable_layers, clearable_layers_default);

    for(unsigned i=0; i < clearable_layers.size(); i++) {
        ROS_INFO("Recovery behavior will clear layer %s", clearable_layers[i].c_str());
        clearable_layers_.insert(clearable_layers[i]);
    }
    runBehaviorService = private_nh.advertiseService("runwaccr", &WaitAndClearCostmapRecovery::runBehavior,this);

    initialized_ = true;
    ROS_INFO("wait_and_clear_costmap_recovery inicialized");
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void WaitAndClearCostmapRecovery::runBehavior(){
  ROS_WARN("Waiting to clear obstacle");
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Clearing costmap to unstuck robot (%fm).", reset_distance_);
    ros::Duration(wait_time_).sleep();
  while (ros::ok() && (checkForObstacle(local_costmap_) || checkForObstacle(global_costmap_))){
    clear(global_costmap_);
    clear(local_costmap_);
    ROS_INFO("Waitnig to clear obstacle");
      ros::Duration(wait_time_).sleep();
      ROS_INFO("~~~~~~~~~~~~~~new cycle~~~~~~~~~~~~~~");

  }
  ROS_INFO("Obstacel cleared");

}

void WaitAndClearCostmapRecovery::clear(costmap_2d::Costmap2DROS* costmap){
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  tf::Stamped<tf::Pose> pose;

  if(!costmap->getRobotPose(pose)){
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.getOrigin().x();
  double y = pose.getOrigin().y();

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    std::string name = plugin->getName();
    int slash = name.rfind('/');
    if( slash != std::string::npos ){
        name = name.substr(slash+1);
    }

    if(clearable_layers_.count(name)!=0){
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y);
    }
  }
}


void WaitAndClearCostmapRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap,
                                        double pose_x, double pose_y){
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
 ROS_ERROR("aaa");
  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

    ROS_INFO("Clearing world x start = %lf x stop = %lf",start_point_x,end_point_x);
    ROS_INFO("Clearing world y start = %lf y stop = %lf",start_point_y,end_point_y);

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

    ROS_INFO("Clearing map x start = %d x stop = %d",start_x,end_x);
    ROS_INFO("Clearing map y start = %d y stop = %d",start_y,end_y);

  unsigned char* grid = costmap->getCharMap();
  for(long int x=start_x; x<end_x; x++){
    for(long int y=start_y; y<end_y; y++){
      long int index = costmap->getIndex(x,y);
        //ROS_INFO("x = %d y= %d index = %d",x,y,index);
        grid[index] = costmap_2d::NO_INFORMATION;
    }
  }

  /*double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);*/
  return;
}
  bool WaitAndClearCostmapRecovery::checkForObstacle(costmap_2d::Costmap2DROS* costmap){
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

    tf::Stamped<tf::Pose> pose;

    if(!costmap->getRobotPose(pose)){
      ROS_ERROR("Abborting wait and clear costmap recovery. Pose cannot be retrieved");
      return false;
    }

    double x = pose.getOrigin().x();
    double y = pose.getOrigin().y();

    bool status = true;
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
      boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
      std::string name = plugin->getName();
      int slash = name.rfind('/');
      if( slash != std::string::npos ){
        name = name.substr(slash+1);
      }
      if(clearable_layers_.count(name)!=0){
          ROS_INFO("checing %s for obstacle",name.c_str());
        boost::shared_ptr<costmap_2d::CostmapLayer> costmap;

          costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
        status &= checkForObstacles(costmap, x, y);
      }
    }
    return status;
  }
    bool WaitAndClearCostmapRecovery::checkForObstacles(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y){

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

        ROS_INFO("Robot world pose x= %lf y=%lf",pose_x,pose_y);
      double start_point_x = pose_x;//- check_distance_ / 2;
      double start_point_y = pose_y - check_distance_ / 2;
      double end_point_x = start_point_x + check_distance_;
      double end_point_y = start_point_y + check_distance_;
        ROS_INFO("Checking world x start = %lf x stop = %lf",start_point_x,end_point_x);
        ROS_INFO("Checking world y start = %lf y stop = %lf",start_point_y,end_point_y);

      int start_x, start_y, end_x, end_y;
      costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
      costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);
        ROS_INFO("Checking map x start = %d x stop = %d",start_x,end_x);
        ROS_INFO("Checking map y start = %d y stop = %d",start_y,end_y);

        unsigned char* grid = costmap->getCharMap();
      for(int x=start_x; x<end_x; x++){
        for(int y = start_y; y<end_y; y++){
            int index = costmap->getIndex(x,y);
            //ROS_INFO("x = %d y= %d index = %d",x,y,index);
          if(grid[index]==costmap_2d::LETHAL_OBSTACLE){
              ROS_INFO("obstacle x = %d y= %d index = %d",x,y,index);
            return true;
          }
            //grid[index] = costmap_2d::LETHAL_OBSTACLE;
        }
      }
	ROS_INFO("end of cycle");
	return false;
    }

    bool WaitAndClearCostmapRecovery::runBehavior(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
        runBehavior();
        return true;
    }


    };
