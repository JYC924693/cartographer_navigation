#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>


// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <tf/tf.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>

#include <vector>
#include <Eigen/Core>

#define PI 3.1416

using namespace std;

namespace local_planner{

  class LocalPlanner : public nav_core::BaseLocalPlanner{

    public:
      LocalPlanner();

      LocalPlanner(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      ~LocalPlanner();

      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      geometry_msgs::PoseStamped getTargetPoint(std::vector<geometry_msgs::PoseStamped> path, geometry_msgs::PoseStamped robot_pose,float lookAhead);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      tf2_ros::Buffer* tf_;
      bool initialized_;

      // Guarda el path global
      std::vector<geometry_msgs::PoseStamped> global_path_;
      geometry_msgs::PoseStamped goal;

      // Para usar odometría
      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;

      // Pose del robot
      geometry_msgs::PoseStamped current_pose_;
      bool goal_reached;

      // Node handler para leer parámetros
      ros::NodeHandle nh;
      
  };
};

#endif
