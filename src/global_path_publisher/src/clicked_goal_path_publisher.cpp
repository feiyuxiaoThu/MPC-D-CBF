#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;

class ClickedGoalPathPublisher {
private:
  ros::NodeHandle nh_;
  ros::Publisher global_path_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber curr_state_sub_;
  ros::Timer publish_timer_;
  
  Vector3d curr_state_; // 当前车辆状态 [x, y, theta]
  bool curr_state_received_;
  
  double step_; // 路径点之间的距离
  Vector2d target_pos_; // 目标位置
  Vector2d fixed_start_pos_; // 固定的起点位置
  bool has_target_; // 是否有目标点
  
public:
  ClickedGoalPathPublisher() : nh_("~"), curr_state_received_(false), has_target_(false) {
    // 初始化发布者和订阅者
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    
    // 订阅RViz的导航目标点
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, 
                             &ClickedGoalPathPublisher::goalCallback, this);
    
    // 也可以订阅点击点
    // goal_sub_ = nh_.subscribe("/clicked_point", 1, 
    //                          &ClickedGoalPathPublisher::clickedPointCallback, this);
    
    // 订阅当前车辆状态
    curr_state_sub_ = nh_.subscribe("/curr_state", 10, 
                                  &ClickedGoalPathPublisher::currStateCallback, this);
    
    // 路径点之间的距离
    nh_.param<double>("step", step_, 0.1);
    
    // 设置定时器，以10Hz的频率持续发布路径
    publish_timer_ = nh_.createTimer(ros::Duration(0.1), &ClickedGoalPathPublisher::timerCallback, this);
    
    ROS_INFO("Clicked Goal Path Publisher initialized");
    ROS_INFO("Waiting for clicked goal from RViz...");
  }
  
  // 定时器回调函数，持续发布路径
  void timerCallback(const ros::TimerEvent& event) {
    if (!curr_state_received_) {
      return;
    }
    
    if (has_target_) {
      // 使用固定的起点位置而不是当前位置
      publishPath(fixed_start_pos_, target_pos_);
    }
  }
  
  // 处理RViz的导航目标点
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!curr_state_received_) {
      ROS_WARN("Current state not received yet, cannot generate path");
      return;
    }
    
    // 记录点击时的车辆位置作为固定起点
    fixed_start_pos_ = Vector2d(curr_state_[0], curr_state_[1]);
    target_pos_ = Vector2d(msg->pose.position.x, msg->pose.position.y);
    has_target_ = true;
    
    ROS_INFO("New goal clicked at: (%.2f, %.2f)", target_pos_.x(), target_pos_.y());
    ROS_INFO("Fixed start position: (%.2f, %.2f)", fixed_start_pos_.x(), fixed_start_pos_.y());
  }
  
  // 处理RViz的点击点
  void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (!curr_state_received_) {
      ROS_WARN("Current state not received yet, cannot generate path");
      return;
    }
    
    // 记录点击时的车辆位置作为固定起点
    fixed_start_pos_ = Vector2d(curr_state_[0], curr_state_[1]);
    target_pos_ = Vector2d(msg->point.x, msg->point.y);
    has_target_ = true;
    
    ROS_INFO("New point clicked at: (%.2f, %.2f)", target_pos_.x(), target_pos_.y());
    ROS_INFO("Fixed start position: (%.2f, %.2f)", fixed_start_pos_.x(), fixed_start_pos_.y());
  }
  
  // 处理当前车辆状态
  void currStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 3) {
      curr_state_[0] = msg->data[0]; // x
      curr_state_[1] = msg->data[1]; // y
      curr_state_[2] = msg->data[2]; // theta
      curr_state_received_ = true;
    }
  }
  
  // 发布路径
  void publishPath(const Vector2d& start_pos, const Vector2d& target_pos) {
    double dist = (target_pos - start_pos).norm();
    Vector2d diff = (target_pos - start_pos) / dist;
    
    nav_msgs::Path global_path;
    global_path.header.stamp = ros::Time::now();
    global_path.header.frame_id = "world";
    
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;
    
    int idx = 0;
    for (double i = 0.0; i < dist; i += step_) {
      pose_stamped.header.seq = idx++;
      
      Vector2d waypoint = start_pos + i * diff;
      pose_stamped.pose.position.x = waypoint.x();
      pose_stamped.pose.position.y = waypoint.y();
      pose_stamped.pose.position.z = 0;
      
      global_path.poses.push_back(pose_stamped);
    }
    
    // 添加终点
    pose_stamped.header.seq = idx;
    pose_stamped.pose.position.x = target_pos.x();
    pose_stamped.pose.position.y = target_pos.y();
    pose_stamped.pose.position.z = 0;
    global_path.poses.push_back(pose_stamped);
    
    global_path_pub_.publish(global_path);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "clicked_goal_path_publisher");
  
  ClickedGoalPathPublisher path_publisher;
  
  ros::spin();
  
  return 0;
}