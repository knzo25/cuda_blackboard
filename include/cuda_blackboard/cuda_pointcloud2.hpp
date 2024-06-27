
#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace cuda_blackboard
{

class CudaPointCloud2 : public sensor_msgs::msg::PointCloud2
{
public:
  using ros_type = sensor_msgs::msg::PointCloud2;

  CudaPointCloud2();
  CudaPointCloud2(const CudaPointCloud2 & pointcloud);  // This is needed for ROS compliance
  CudaPointCloud2 & operator=(const CudaPointCloud2 &) = delete;

  CudaPointCloud2(CudaPointCloud2 && pointcloud);
  CudaPointCloud2 & operator=(CudaPointCloud2 &&);
  CudaPointCloud2(const sensor_msgs::msg::PointCloud2 & source);

  ~CudaPointCloud2();

  uint8_t * data{nullptr};
};

}  // namespace cuda_blackboard
