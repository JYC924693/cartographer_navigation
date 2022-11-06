/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

#include "cartographer_ros/msg_conversion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

void Reset_InitPose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
cartographer_ros::Node* node_handle;
cartographer_ros::TrajectoryOptions* trajectory_options_handle;

template<typename T>
T getParam_Func(ros::NodeHandle& pnh,
                    const std::string& param_name, const T & default_val)
{
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}
//修改结束

/**
 * note: gflags是一套命令行参数解析工具
 * DEFINE_bool在gflags.h中定义
 * gflags主要支持的参数类型包括bool, int32, int64, uint64, double, string等
 * 定义参数通过DEFINE_type宏实现, 该宏的三个参数含义分别为命令行参数名, 参数默认值, 以及参数的帮助信息
 * 当参数被定义后, 通过FLAGS_name就可访问到对应的参数
 */
// collect_metrics ：激活运行时度量的集合.如果激活, 可以通过ROS服务访问度量
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  // 开启监听tf的独立线程
  tf2_ros::TransformListener tf(tf_buffer);

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;

  // c++11: std::tie()函数可以将变量连接到一个给定的tuple上,生成一个元素类型全是引用的tuple

  // 根据Lua配置文件中的内容, 为node_options, trajectory_options 赋值
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // MapBuilder类是完整的SLAM算法类
  // 包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  
  // c++11: std::move 是将对象的状态或者所有权从一个对象转移到另一个对象, 
  // 只是转移, 没有内存的搬迁或者内存拷贝所以可以提高利用效率,改善性能..
  // 右值引用是用来支持转移语义的.转移语义可以将资源 ( 堆, 系统对象等 ) 从一个对象转移到另一个对象, 
  // 这样能够减少不必要的临时对象的创建、拷贝以及销毁, 能够大幅度提高 C++ 应用程序的性能.
  // 临时对象的维护 ( 创建和销毁 ) 对性能有严重影响.

  // Node类的初始化, 将ROS的topic传入SLAM, 也就是MapBuilder
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
  
  trajectory_options_handle = &(trajectory_options);  // 修改
  node_handle = &(node);  //修改
  ros::Subscriber initPose_sub = node.node_handle()->subscribe("/initialpose", 1, Reset_InitPose_callback);  //修改

  // 修改开始
  // 获取轨迹配置项指针
  auto trajectory_options_handle = &(trajectory_options);  //原博文这里少了个auto
  ros::NodeHandle pnh("~");
  // 获取配置参数
  bool localization = getParam_Func<bool>(pnh, "/localization", 0);      //是否为纯定位模式
  double pos_x = getParam_Func<double>(pnh, "/set_inital_pose_x", 0.0); 
  double pos_y = getParam_Func<double>(pnh, "/set_inital_pose_y", 0.0); 
  double pos_z = getParam_Func<double>(pnh, "/set_inital_pose_z", 0.0); 
  double pos_ox = getParam_Func<double>(pnh, "/set_inital_pose_ox", 0.0); 
  double pos_oy = getParam_Func<double>(pnh, "/set_inital_pose_oy", 0.0); 
  double pos_oz = getParam_Func<double>(pnh, "/set_inital_pose_oz", 0.0); 
  double pos_ow = getParam_Func<double>(pnh, "/set_inital_pose_ow", 1.0); 
  geometry_msgs::Pose init_pose;
  init_pose.position.x = pos_x;
  init_pose.position.y = pos_y;
  init_pose.position.z = pos_z;
  init_pose.orientation.x = pos_ox;
  init_pose.orientation.y = pos_oy;
  init_pose.orientation.z = pos_oz;
  init_pose.orientation.w = pos_ow;

  if(localization == true)
  {
  //更改轨迹配置项中的初始位姿值
  *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
      = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(init_pose));
  }
  // 修改结束

  // 如果加载了pbstream文件, 就执行这个函数
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  // 使用默认topic 开始轨迹
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::spin();

  // 结束所有处于活动状态的轨迹
  node.FinishAllTrajectories();

  // 当所有的轨迹结束时, 再执行一次全局优化
  node.RunFinalOptimization();

  // 如果save_state_filename非空, 就保存pbstream文件
  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

void Reset_InitPose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  // 关闭当前运行的Trajectories
  node_handle->FinishAllTrajectories();
  // 给轨迹设置起点 msg->pose.pose
  // start trajectory with initial pose
  *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
    = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(msg->pose.pose));
  // 重新开启Trajectory
  if (FLAGS_start_trajectory_with_default_topics) 
  {
    node_handle->StartTrajectoryWithDefaultTopics(*trajectory_options_handle);
  }
}

int main(int argc, char** argv) {

  // note: 初始化glog库
  google::InitGoogleLogging(argv[0]);
  
  // 使用gflags进行参数的初始化. 其中第三个参数为remove_flag
  // 如果为true, gflags会移除parse过的参数, 否则gflags就会保留这些参数, 但可能会对参数顺序进行调整.
  google::ParseCommandLineFlags(&argc, &argv, true);

  /**
   * @brief glog里提供的CHECK系列的宏, 检测某个表达式是否为真
   * 检测expression如果不为真, 则打印后面的description和栈上的信息
   * 然后退出程序, 出错后的处理过程和FATAL比较像.
   */
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  // ros节点的初始化
  ::ros::init(argc, argv, "cartographer_node");

  // 一般不需要在自己的代码中显式调用
  // 但是若想在创建任何NodeHandle实例之前启动ROS相关的线程, 网络等, 可以显式调用该函数.
  ::ros::start();

  // 使用ROS_INFO进行glog消息的输出
  cartographer_ros::ScopedRosLogSink ros_log_sink;

  // 开始运行cartographer_ros
  cartographer_ros::Run();

  // 结束ROS相关的线程, 网络等
  ::ros::shutdown();
}
