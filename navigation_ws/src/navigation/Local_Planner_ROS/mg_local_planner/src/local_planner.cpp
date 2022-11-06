#include <mg_local_planner/local_planner.hpp>
#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>

#include <nav_core/parameter_magic.h>

// PLUGINLIB_EXPORT_CLASS(local_planner, LocalPlanner, local_planner::LocalPlanner, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{
	void printQuaternion(geometry_msgs::Quaternion quat)
	{
		tf2::Quaternion q;
		tf2::fromMsg(quat, q);
		q.normalize();
		ROS_INFO("Quaternion: %f %f %f %f -- %f", q.getW(), q.getX(), q.getY(), q.getZ(), q.getAngle());
	}

	void printPose(geometry_msgs::PoseStamped data, std::string name)
	{
		const char *namechr = name.c_str();
		auto posX = data.pose.position.x;
		auto posY = data.pose.position.y;
		auto posZ = data.pose.position.z;
		auto ori = tf2::getYaw(data.pose.orientation);
		ROS_INFO("%s: %f , %f, %f --- %f", namechr, posX, posY, posZ, ori);
		// printQuaternion(data.pose.orientation);
	}

	void getPose(geometry_msgs::PoseStamped data, Eigen::Vector3f &pos)
	{
		pos[0] = data.pose.position.x;
		pos[1] = data.pose.position.y;
		pos[2] = tf2::getYaw(data.pose.orientation);
	}

	void calcErrors(geometry_msgs::Pose pose, geometry_msgs::Pose goal, double &error_dist, double &error_ori)
	{
		auto posX = pose.position.x;
		auto posY = pose.position.y;
		auto posZ = pose.position.z;
		auto ori = 2.0 * asin(pose.orientation.z) / PI * 180.0;

		auto objX = goal.position.x;
		auto objY = goal.position.y;

		error_dist = 100.0 * sqrt(pow(posX - objX, 2) + pow(posY - objY, 2));
		double objAng = std::atan2(objY - posY, objX - posX);
		error_ori = (objAng - ori);
		// ROS_INFO_STREAM(ori);
		while (error_ori < -2.0 * PI)
			error_ori += 2.0 * PI;
		while (error_ori > 2.0 * PI)
			error_ori -= 2.0 * PI;

		error_ori -= PI;
	}

	void calcErrors(Eigen::Vector3f pose, Eigen::Vector3f goal, double &error_dist, double &error_ori, bool approaching)
	{
		error_dist = 10.0 * sqrt(pow(pose[0] - goal[0], 2) + pow(pose[1] - goal[1], 2));

		if (approaching)
		{
			double objAng = std::atan2((goal - pose)[1], (goal - pose)[0]);
			error_ori = objAng - pose[2];
		}
		else
			error_ori = goal[2] - pose[2];

		// Orientation error has to be in [-180º, 180º]. Currently in [-360º, 360º]
		while (error_ori < -PI)
			error_ori += 2.0 * PI;
		while (error_ori > PI)
			error_ori -= 2.0 * PI;
	}

	double saturate(double data, double min, double max)
	{
		if (data > max)
			return max;
		else if (data < min)
			return min;
		else
			return data;
	}

	double calcDistance(geometry_msgs::PoseStamped A, geometry_msgs::PoseStamped B)
	{
		double Ax, Bx, Ay, By;
		Ax = A.pose.position.x;
		Ay = A.pose.position.y;
		Bx = B.pose.position.x;
		By = B.pose.position.y;

		double distance = sqrt(pow(Ax - Bx, 2) + pow(Ay - By, 2));

		return distance;
	}

	LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false), global_path_(NULL)
	{
	}

	LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
		: costmap_ros_(NULL), tf_(NULL), initialized_(false), global_path_(NULL), goal_reached(false)
	{
		initialize(name, tf, costmap_ros);
	}

	LocalPlanner::~LocalPlanner()
	{
	}

	void LocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
	{
		/*
		Funcion heredada del base_local_planner. Inicializacion del planificador.
		*/
		if (!initialized_)
		{
			ROS_INFO("INITIALIZING!!\n");
			ros::NodeHandle private_nh("~/" + name);

			tf_ = tf;
			costmap_ros_ = costmap_ros;
			costmap_ros_->getRobotPose(current_pose_);

			costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();

			if (private_nh.getParam("odom_topic", odom_topic_))
			{
				odom_helper_.setOdomTopic(odom_topic_);
			}

			initialized_ = true;

			ROS_INFO("Planner initialized!\n");
		}
		else
			ROS_INFO("The planner has already been initialized");
	}

	geometry_msgs::PoseStamped LocalPlanner::getTargetPoint(std::vector<geometry_msgs::PoseStamped> path,
															geometry_msgs::PoseStamped robot_pose, float lookAhead)
	{
		/*
			Pure pursuit function to get target point from path.
			@ path : path to follow
			@ lookAhead : look_ahead parameter for the purepursuit.

			@returns target_point : next point to follow
		*/

		int N = path.size();
		double distance = 0.0;

		// Start at the back of the path (goal point)
		geometry_msgs::PoseStamped target_point = path.back();

		for (int i = 0; i < N && distance < lookAhead; i++)
		{
			// Iterate through path to find the next point with distance >= lookAhead
			distance = 10.0 * calcDistance(robot_pose, path[i]);
			if (distance > lookAhead)
				target_point = path[i];
		}

		return target_point;
	}

	bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
	{
		/*
		Funcion heredada del base_local_planner. Actualiza el path global.
		*/
		if (!initialized_)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}
		global_path_ = orig_global_plan;
		if (goal_reached)
			goal_reached = false;

		return true;
	}

	bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
	{
		/*
		Funcion heredada del base_local_planner. Sirve para publicar la velocidad del robot que queramos con
		el cmd_vel pasado por referencia.
		*/
		if (!initialized_)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		// Get public params----

		// Get the gain for the controller
		double K_lin, K_ang;
		if (!nh.getParam("K_lin", K_lin))
		{
			ROS_ERROR("Failed to get linear gain parameters");
			K_lin = 0.1;
		}
		if (!nh.getParam("K_ang", K_ang))
		{
			ROS_ERROR("Failed to get angular gain parameters");
			K_ang = 0.3;
		}

		// Get the saturation for the velocities
		double v_max, w_max;
		if (!nh.getParam("w_max", w_max))
		{
			ROS_ERROR("Failed to get linear vel. max");
			w_max = 1.0;
		}
		if (!nh.getParam("v_max", v_max))
		{
			ROS_ERROR("Failed to get angular vel. max");
			v_max = 1.0;
		}

		// Get the debug mode param
		int debug_flag;
		if (!nh.getParam("debug_flag", debug_flag))
		{
			ROS_ERROR("Failed to get debug mode param");
			debug_flag = 0;
		}

		// Get the look ahead param
		double look_ahead;
		if (!nh.getParam("lh", look_ahead))
		{
			ROS_ERROR("Failed to get look ahead param");
			look_ahead = 2.0;
		}
		//--------------------

		// Check if both the pose and goal are availiable
		double error_dist;
		double error_ori;
		Eigen::Vector3f robot_pose, goal_pose;

		if (!global_path_.empty() && costmap_ros_->getRobotPose(current_pose_))
		{
			// goal = global_path_.back();
			goal = getTargetPoint(global_path_, current_pose_, look_ahead);
			getPose(goal, goal_pose);
			getPose(current_pose_, robot_pose);
		}
		else
			return false;

		calcErrors(robot_pose, goal_pose, error_dist, error_ori, true);

		if (error_dist > 0.1)
		{
			// Control
			double w, velX;
			w = K_ang * error_ori;
			w = saturate(w, -w_max, w_max);

			// Velocidad proporcional a la distancia con saturacion en la dada.
			// Si ya se ha orientado aprox.
			if (abs(error_ori / PI * 180.0) < 45.0)
			{
				velX = K_lin * error_dist;
				velX = saturate(velX, -v_max, v_max);
			}
			else
				velX = 0;

			cmd_vel.linear.x = velX;
			cmd_vel.angular.z = w;
			if (debug_flag)
			{	
				ROS_INFO("Errors calculated: %f , %f", error_dist, error_ori / PI * 180.0);
				ROS_INFO("Velocities calculated: %f , %f", velX, w);
			}
		}
		else
		{
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			goal_reached = true;
			ROS_INFO("GOAL REACHED");
		}

		return true;
	}

	bool LocalPlanner::isGoalReached()
		/*
		Funcion heredada del base_local_planner. Indica a move_base si se ha alcanzado el objetivo.
		*/
	{
		if (!initialized_)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		return goal_reached;
	}
}