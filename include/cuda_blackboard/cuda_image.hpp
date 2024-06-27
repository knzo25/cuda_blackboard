

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/type_adapter.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cuda_runtime_api.h>

#include <memory>

namespace cuda_blackboard
{

class CudaImage : public sensor_msgs::msg::Image
{
public:
  using ros_type = sensor_msgs::msg::Image;

  CudaImage() = default;
  CudaImage(const CudaImage & image);  // This is needed for ROS compliance
  CudaImage & operator=(const CudaImage &) = delete;
  CudaImage(CudaImage && image) = default;
  CudaImage & operator=(CudaImage &&) = default;
  CudaImage(const sensor_msgs::msg::Image & source);

  ~CudaImage();

  uint8_t * data;
};

}  // namespace cuda_blackboard
