
#pragma once

#include "cuda_blackboard/cuda_adaptation.hpp"
#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"

#include <std_msgs/msg/u_int64.hpp>

#include <string>

namespace cuda_blackboard
{

template <typename T>
struct NegotiationStruct
{
  using MsgT = T;

  static_assert(
    std::is_same_v<T, int> || std::is_same_v<T, double>,
    "NegotiationStruct can only be instantiated with CudaImage or CudaPointcloud2.");

  static const inline std::string supported_type_name = "";
};

template <>
struct NegotiationStruct<CudaImage>
{
  using MsgT = std_msgs::msg::UInt64;
  static const inline std::string supported_type_name =
    "_cuda_image";  // Hide the topic from the user
};

template <>
struct NegotiationStruct<CudaPointCloud2>
{
  using MsgT = std_msgs::msg::UInt64;
  static const inline std::string supported_type_name =
    "_cuda_pointcloud";  // Hide the topic from the user
};

template <>
struct NegotiationStruct<sensor_msgs::msg::Image>
{
  // using MsgT = sensor_msgs::msg::Image;
  using MsgT = CudaImage;
  static const inline std::string supported_type_name = "ros_image";
};

template <>
struct NegotiationStruct<sensor_msgs::msg::PointCloud2>
{
  // using MsgT = sensor_msgs::msg::PointCloud2;
  using MsgT = CudaPointCloud2;
  static const inline std::string supported_type_name = "ros_pointcloud";
};

}  // namespace cuda_blackboard
