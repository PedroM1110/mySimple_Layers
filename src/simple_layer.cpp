#include<simple_layers/simple_layer.h>  
#include <pluginlib/class_list_macros.h>  
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/layer.h>  
#include <costmap_2d/layered_costmap.h> 
#include "std_msgs/String.h"
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
  
  

	
void SimpleLayer::callback(const std_msgs::String::ConstPtr& msg){
  
	
	ROS_INFO("Zona %s", msg->data.c_str());

  pthread_mutex_lock(&mutex);
  mx[0] = 200;
	my[0] = 200;
	
	if(strcmp(msg->data.c_str(),"1")==0){
    
		for(int pos = 1; pos < 21; pos++){
		mx[pos] = mx[pos-1] - 1;
		my[pos] = 200;
		}
	}

	else if(strcmp(msg->data.c_str(),"2")==0){
		
    my[0] = 179;
		for(int pos = 1; pos < 21; pos++){
		mx[pos] = mx[pos-1] - 1;
		my[pos] = 179;
    }
  }
  pthread_mutex_unlock(&mutex);
   
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
	
	for(int pos = 0; pos < 21; pos++){
    
    master_grid.setCost(mx[pos], my[pos], LETHAL_OBSTACLE);
		
	}
    
  
  
}  
} //end namespace