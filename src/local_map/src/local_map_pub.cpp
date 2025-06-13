#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "ellipse.hpp"
#include "DBSCAN.hpp"
#include "KM.hpp"

using namespace grid_map;
using namespace std;

GridMap map_({"elevation", "local_lidar", "gradient_map"});

pcl::PointCloud<pcl::PointXYZ> velodyne_cloud;
pcl::PointCloud<pcl::PointXYZ> velodyne_cloud_filter;
pcl::PointCloud<pcl::PointXYZ> velodyne_cloud_global;

pcl::PassThrough<pcl::PointXYZ> pass_x;
pcl::PassThrough<pcl::PointXYZ> pass_y;
pcl::VoxelGrid<pcl::PointXYZ> sor;

tf::TransformListener *listener_ptr = nullptr;
tf::StampedTransform base_link_transform;
tf::StampedTransform lidar_link_transform;
Eigen::Vector2d robot_position2d;

// ros sub & pub
ros::Subscriber velodyne_sub;

ros::Publisher gridmap_pub;
ros::Publisher ellipse_vis_pub;
ros::Publisher local_pcd_pub;
ros::Publisher for_obs_track_pub;

KMAlgorithm KM;

// DBSCAN param
float DBSCAN_R;
int DBSCAN_N;

float step_height;
float obs_height;

int block_size;
int block_num;

// robot pose obtain
void updateTF()
{
  while (true)
  {
    try
    {
      listener_ptr->waitForTransform("world", "base_link", ros::Time(0), ros::Duration(0.1));
      listener_ptr->lookupTransform("world", "base_link", ros::Time(0), base_link_transform);
      listener_ptr->waitForTransform("world", "velodyne", ros::Time(0), ros::Duration(0.1));
      listener_ptr->lookupTransform("world", "velodyne", ros::Time(0), lidar_link_transform);
      break;
    }
    catch (tf::TransformException &ex)
    {
      // ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
}

void pcd_transform()
{
  pass_x.setInputCloud(velodyne_cloud.makeShared());
  pass_x.filter(velodyne_cloud);

  pass_y.setInputCloud(velodyne_cloud.makeShared());
  pass_y.filter(velodyne_cloud);

  velodyne_cloud_filter.clear();
  sor.setInputCloud(velodyne_cloud.makeShared());
  sor.filter(velodyne_cloud_filter);

  Eigen::Affine3d affine_transform;
  tf::transformTFToEigen(lidar_link_transform, affine_transform);
  pcl::transformPointCloud(velodyne_cloud_filter, velodyne_cloud_global, affine_transform);
}

// 将激光雷达点云数据转换为网格地图
// 功能：遍历全局坐标系下的激光雷达点云，将每个点的Z值（高度）映射到对应的网格地图位置
// 对于每个网格位置，保存该位置的最高Z值作为地面高度
void lidar2gridmap(Eigen::MatrixXf &lidar_data_matrix)
{
  int col = lidar_data_matrix.cols();
  int row = lidar_data_matrix.rows();
  for (const auto &pt : velodyne_cloud_global)
  {
    int j = (pt.x - robot_position2d.x()) * _inv_resolution + col * 0.5;
    j = min(max(j, 0), row - 1);
    int k = (pt.y - robot_position2d.y()) * _inv_resolution + row * 0.5;
    k = min(max(k, 0), col - 1);

    if (std::isnan(lidar_data_matrix(row - 1 - j, col - 1 - k)))
      lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
    if (lidar_data_matrix(row - 1 - j, col - 1 - k) < pt.z)
      lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
  }
}


// 地图插值函数：填补地图中的无效像素点
// 功能：
// 1. 对地图中的NaN（无效）像素点进行插值处理
// 2. 使用3x3邻域内有效像素的平均值来填充无效像素
// 3. 提高地图的连续性和完整性，为后续处理提供更好的数据基础
// 4. 如果邻域内没有有效像素，则保持原始的NaN值
Eigen::MatrixXf map_interpolation(const Eigen::MatrixXf &map_data)
{
  int col = map_data.cols(), row = map_data.rows();
  Eigen::MatrixXf map_interpolation(map_data);  // 复制原始地图数据
  
  // 遍历地图的每个像素点（除了边界）
  for (int i = 1; i < row - 1; i++)
  {
    for (int j = 1; j < col - 1; j++)
    {
      // 如果当前像素点是NaN（无效值）
      if (isnan(map_data(i, j)))
      {
        int count = 0;
        float height = 0;
        
        // 检查3x3邻域内的所有像素点
        for (int k = 0; k <= 2; k++)
        {
          for (int q = 0; q <= 2; q++)
          {
            // 如果邻域内的像素点有效（不是NaN）
            if (!isnan(map_data(i - 1 + k, j - 1 + q)))
            {
              count++;  // 计数有效邻居
              height += map_data(i - 1 + k, j - 1 + q);  // 累加高度值
            }
          }
        }
        
        // 用邻域有效像素的平均值填充当前NaN像素，如果没有有效邻居则保持NaN
        map_interpolation(i, j) = (count > 0) ? height / count : NAN;
      }
    }
  }
  return map_interpolation;
}
// 地图膨胀块处理函数：对指定区域进行局部膨胀操作
// 功能：
// 1. 以给定的中心点(startRow, startCol)为中心，在指定半径范围内进行膨胀
// 2. 遍历以中心点为核心的(2*radius+1)x(2*radius+1)邻域
// 3. 将邻域内所有NaN（无效）像素点的值替换为中心点的高度值
// 4. 实现障碍物区域的扩展，增强障碍物的检测和避障效果
// 参数：
// - dst: 目标地图矩阵（膨胀结果写入此矩阵）
// - src: 源地图矩阵（读取数据的原始地图）
// - startRow, startCol: 膨胀中心点的行列坐标
// - radius: 膨胀半径
void map_inflate_block(Eigen::MatrixXf &dst, const Eigen::MatrixXf &src, int startRow, int startCol, int radius)
{
  for (int k = 0; k <= 2 * radius; k++)
  {
    for (int q = 0; q <= 2 * radius; q++)
    {
      if (isnan(src(startRow - radius + k, startCol - radius + q)))
      {
        dst(startRow - radius + k, startCol - radius + q) = src(startRow, startCol);
      }
    }
  }
}

// 地图膨胀处理函数：对输入地图进行膨胀操作以增强障碍物区域
// 功能：
// 1. 遍历地图中的每个有效像素点（排除边界和NaN值）
// 2. 根据像素点到地图中心的距离动态确定膨胀半径
//    - 中心区域（距离 < col/3）：膨胀半径为1
//    - 中等距离区域（距离 < col*0.45）：膨胀半径为2  
//    - 远离中心区域：膨胀半径为3
// 3. 对每个有效像素点调用map_inflate_block进行局部膨胀处理
// 4. 返回膨胀后的地图，用于后续的路径规划和障碍物检测
Eigen::MatrixXf map_inflate(const Eigen::MatrixXf &map_data)
{
  int col = map_data.cols(), row = map_data.rows();
  Eigen::MatrixXf map_inflated(map_data);
  for (int i = 3; i < row - 3; i++)
  {
    for (int j = 3; j < col - 3; j++)
    {
      if (isnan(map_data(i, j)))
        continue;

      double dis = sqrt((i - col / 2) * (i - col / 2) + (j - col / 2) * (j - col / 2));
      int radius;
      if (dis < col / 3)
        radius = 1;
      else if (dis < col * 0.45)
        radius = 2;
      else
        radius = 3;
      map_inflate_block(map_inflated, map_data, i, j, radius);
    }
  }
  return map_inflated;
}

// 梯度地图处理函数：计算地图的梯度信息并检测障碍物
// 功能：
// 1. 使用Sobel算子计算每个像素点的梯度幅值
// 2. 根据梯度大小和高度差检测障碍物点
// 3. 将检测到的障碍物点添加到数据集中用于后续聚类分析
// 4. 返回处理后的梯度地图
Eigen::MatrixXf gradient_map_processing(Eigen::MatrixXf &map_data, vector<DBSCAN::Point> &dataset)
{
  const float threshold = -1.25;
  int col = map_data.cols(), row = map_data.rows();
  Eigen::MatrixXf gradient_map(row, col);
  gradient_map.setOnes();
  gradient_map *= threshold;
  DBSCAN::Point obs;

  for (int i = 1; i < row - 1; i++)
  {
    for (int j = 1; j < col - 1; j++)
    {
      bool has_nan_value = false;
      for (int p = -1; p <= 1; p++)
      {
        for (int q = -1; q <= 1; q++)
        {
          if (isnan(map_data(i + p, j + q)))
          {
            gradient_map(i, j) = threshold;
            has_nan_value = true;
          }
        }
      }
      if (!has_nan_value)
      {
        float sobel_x = map_data(i + 1, j + 1) - map_data(i - 1, j + 1) + map_data(i + 1, j) - map_data(i - 1, j) +
                        map_data(i + 1, j - 1) - map_data(i - 1, j - 1);
        float sobel_y = map_data(i - 1, j + 1) - map_data(i - 1, j - 1) + map_data(i, j + 1) - map_data(i, j - 1) +
                        map_data(i + 1, j + 1) - map_data(i + 1, j - 1);
        gradient_map(i, j) = sqrt(sobel_x * sobel_x + sobel_y * sobel_y) + threshold;
        if (gradient_map(i, j) > obs_height + threshold)
        {
          obs.x = i;
          obs.y = j;
          dataset.push_back(obs);
        }
        else
        {
          Eigen::MatrixXf::Index minRow, minCol;
          Eigen::MatrixXf block = map_data.block(i / block_size, j / block_size, block_size, block_size);
          float min = block.minCoeff(&minRow, &minCol);
          if (map_data(i, j) - min > step_height)
          {
            obs.x = i;
            obs.y = j;
            dataset.push_back(obs);
          }
        }
      }
    }
  }

  return gradient_map;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_map_pub");
  ros::NodeHandle nh("~");

  gridmap_pub = nh.advertise<grid_map_msgs::GridMap>("gridmap", 1, true);
  local_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("local_pcd", 1);
  ellipse_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("ellipse_vis", 1);
  for_obs_track_pub = nh.advertise<std_msgs::Float32MultiArray>("for_obs_track", 1);
  velodyne_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, [&](sensor_msgs::PointCloud2::ConstPtr msg)
                                                        { 
  pcl::fromROSMsg(*msg, velodyne_cloud);    
  updateTF(); });

  // 参数配置
  // 局部地图尺寸参数
  nh.param<float>("localmap_x_size", localmap_x_size, 10);  // 局部地图X方向尺寸，单位：米，默认10米
  nh.param<float>("localmap_y_size", localmap_y_size, 10);  // 局部地图Y方向尺寸，单位：米，默认10米
  nh.param<float>("resolution", resolution, 0.1);          // 地图分辨率，单位：米/像素，默认0.1米

  // 障碍物检测参数
  nh.param<float>("obs_height", obs_height, 0.4);          // 障碍物高度阈值，单位：米，默认0.4米
  nh.param<float>("step_height", step_height, 0.5);        // 台阶高度阈值，单位：米，默认0.5米
  nh.param<float>("DBSCAN_R", DBSCAN_R, 5.0);             // DBSCAN聚类算法的邻域半径，单位：像素，默认5.0
  nh.param<int>("DBSCAN_N", DBSCAN_N, 5);                 // DBSCAN聚类算法的最小邻居点数，默认5个

  // 分块处理参数
  nh.param<int>("block_size", block_size, localmap_x_size * _inv_resolution * 0.2);  // 分块大小，默认为地图宽度的20%
  nh.param<int>("block_num", block_num, 5);                // 分块数量，默认5个

  // 计算辅助变量
  _inv_resolution = 1 / resolution;                        // 分辨率的倒数，用于坐标转换
  int map_index_len = localmap_x_size * _inv_resolution;   // 地图索引长度，即地图像素尺寸

  tf::TransformListener listener;
  listener_ptr = &listener;

  // initial
  map_.setFrameId("world");
  map_.setGeometry(grid_map::Length(localmap_x_size, localmap_y_size), resolution);
  Eigen::MatrixXf lidar_pcd_matrix(map_index_len, map_index_len);
  Eigen::MatrixXf map_interpolate(map_index_len, map_index_len);
  Eigen::MatrixXf gradient_map(map_index_len, map_index_len);

  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-localmap_x_size / 2, localmap_x_size / 2);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-localmap_y_size / 2, localmap_y_size / 2);

  sor.setLeafSize(0.05f, 0.05f, 0.05f);

  // main loop
  ros::Rate rate(11);
  while (ros::ok())
  {
    robot_position2d << base_link_transform.getOrigin().x(), base_link_transform.getOrigin().y();
    pcd_transform();
    map_.setPosition(robot_position2d);
    map_.clear("local_lidar");
    lidar_pcd_matrix = map_.get("local_lidar");
    lidar2gridmap(lidar_pcd_matrix);
    map_.add("local_lidar", lidar_pcd_matrix);

    map_interpolate = map_interpolation(lidar_pcd_matrix);
    map_interpolate = map_inflate(map_interpolate);
    map_.add("elevation", map_interpolate);

    // obs map
    vector<DBSCAN::Point> non_clustered_obs;
    gradient_map = gradient_map_processing(map_interpolate, non_clustered_obs);

    // DBSCAN
    DBSCAN DS(DBSCAN_R, DBSCAN_N, non_clustered_obs);
    vector<Obstacle> clustered_obs(DS.cluster_num);
    for (const auto &obs : non_clustered_obs)
    {
      if (obs.obsID > 0)
      {
        gradient_map(obs.x, obs.y) = -0.3;
        clustered_obs[obs.obsID - 1].emplace_back(obs.x, obs.y);
      }
    }    

    // get MSE
    vector<Ellipse> ellipses_array = get_ellipse_array(clustered_obs, map_);
    KM.tracking(ellipses_array);
    ab_variance_calculation(ellipses_array);

    // publish
    std_msgs::Float32MultiArray for_obs_track;
    if(ellipses_array.size()== 0)
    {
      ROS_INFO("ellipses_array.size() is 0");
    }
    for (const auto &ellipse : ellipses_array)
    {
      if (ellipse.label == 0)
        continue;

      for_obs_track.data.push_back(ellipse.cx);
      for_obs_track.data.push_back(ellipse.cy);
      for_obs_track.data.push_back(ellipse.semimajor);
      for_obs_track.data.push_back(ellipse.semiminor);
      for_obs_track.data.push_back(ellipse.theta);
      for_obs_track.data.push_back(ellipse.label);
      for_obs_track.data.push_back(ellipse.variance);
    }
    ROS_INFO("for_obs_track.data.size(): %d", for_obs_track.data.size());
    for_obs_track_pub.publish(for_obs_track);

    // publish pcd data
    sensor_msgs::PointCloud2 local_velodyne_msg;
    pcl::toROSMsg(velodyne_cloud_global, local_velodyne_msg);
    local_velodyne_msg.header.stamp = ros::Time::now();
    local_velodyne_msg.header.frame_id = "world";
    local_pcd_pub.publish(local_velodyne_msg);

    grid_map_msgs::GridMap gridMapMessage;
    grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
    gridmap_pub.publish(gridMapMessage);
    // publish ellipse data
    visEllipse(ellipses_array, ellipse_vis_pub);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
