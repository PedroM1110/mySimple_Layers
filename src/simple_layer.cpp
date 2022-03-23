#include<simple_layers/simple_layer.h>  
#include <pluginlib/class_list_macros.h>  
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/layer.h>  
#include <costmap_2d/layered_costmap.h> 
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int8.h"
#include <string>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/service_client.h>
#include <pthread.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>




  
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)//Register plug-in
  
using costmap_2d::LETHAL_OBSTACLE;  
using costmap_2d::FREE_SPACE; 
  
namespace simple_layer_namespace  
{  
  
SimpleLayer::SimpleLayer() {}  
  
void SimpleLayer::onInitialize()  
{  
	
  ros::NodeHandle n("~/" + name_);
	
  ros::NodeHandle nh("~/" + name_);
  
  sub = n.subscribe("/wsn", 1000, &SimpleLayer::callback, this);
  
  pthread_mutex_init(&mutex, NULL);
	
  current_ = true;								
  
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind( &SimpleLayer::reconfigureCB, this, _1, _2);
  
  dsrv_->setCallback(cb);

}  
  
  

	
void SimpleLayer::callback(const std_msgs::UInt8MultiArray::ConstPtr& array){
  
	ROS_INFO("Zonas: %d, %d, %d, %d", array->data[0],array->data[1],array->data[2],array->data[3]);

	place_1 = array->data[0];
  place_2 = array->data[1];
  place_3 = array->data[2];
  place_4 = array->data[3];
    
}


void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)  
{  
  enabled_ = config.enabled; 
}  
  
void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,  
                                           double* min_y, double* max_x, double* max_y)
  
{  
  if (!enabled_)
  
    return;
  
  mark_x_ = origin_x + cos(origin_yaw);
  
  mark_y_ = origin_y + sin(origin_yaw);
  
  *min_x = std::min(*min_x, mark_x_);
  
  *min_y = std::min(*min_y, mark_y_);
  
  *max_x = std::max(*max_x, mark_x_);
  
  *max_y = std::max(*max_y, mark_y_);
  

}  
  
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,  int max_j)
{  
  if (!enabled_)
  
    return;
  
  if(place_1 == 1){
    for(int posx = 328; posx < 342; posx++){
      for(int posy = 260; posy < 275; posy++){
        mx[posx] = posx; 
        my[posy] = posy;
        master_grid.setCost(mx[posx], my[posy], LETHAL_OBSTACLE);
      }
	  }
  }

  if(place_2 == 1){
    for(int posx = 285; posx < 295; posx++){
      for(int posy = 215; posy < 220; posy++){
        mx[posx] = posx; 
        my[posy] = posy;
        master_grid.setCost(mx[posx], my[posy], LETHAL_OBSTACLE);
      }
	  }
  }

  if(place_3 == 1){
    for(int posx = 312; posx < 325; posx++){
      for(int posy = 175; posy < 180; posy++){
        mx[posx] = posx; 
        my[posy] = posy;
        master_grid.setCost(mx[posx], my[posy], LETHAL_OBSTACLE);
      }
	  }
  }

  if(place_4 == 1){
    for(int posx = 323; posx < 338; posx++){
      for(int posy = 430; posy < 445; posy++){
        mx[posx] = posx; 
        my[posy] = posy;
        master_grid.setCost(mx[posx], my[posy], LETHAL_OBSTACLE);
      }
	  }
  }
	  
    
  
  
}  
} //end namespace