#ifndef SIMPLE_LAYER_H_  
#define SIMPLE_LAYER_H_  
#include <ros/ros.h>  
#include <costmap_2d/layer.h>  
#include <costmap_2d/layered_costmap.h>  
#include <costmap_2d/GenericPluginConfig.h>  
#include <dynamic_reconfigure/server.h>  
#include <pluginlib/class_list_macros.h>  
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <pthread.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>



void reconfigure(double freq);
   
namespace simple_layer_namespace  
{  
      
	class SimpleLayer : public costmap_2d::Layer  
  	{  
    	public:  
      		SimpleLayer();
      		
      		void callback(const std_msgs::String::ConstPtr& msg);
		
			ros::Subscriber sub;
      
      		virtual  void onInitialize();
      
      		virtual  void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      	
      		virtual  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
			

			
     	
    	private:  
      		void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

      		double mark_x_, mark_y_;
		
			unsigned int mx[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  
       		unsigned int my[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		
			pthread_mutex_t mutex; 
      
      		dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
      
    };  
}  
    #endif 


