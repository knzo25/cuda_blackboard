
#include "cuda_blackboard/cuda_image.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime_api.h>

namespace cuda_blackboard
{

CudaImage::CudaImage(const CudaImage & image) : sensor_msgs::msg::Image(image)
{
  RCLCPP_WARN(
    rclcpp::get_logger("CudaImage"),
    "CudaImage copy constructor called. This should be avoided and is most likely a design error.");

  cudaMalloc(reinterpret_cast<void **>(&data), image.height * image.step * sizeof(uint8_t));
  cudaMemcpy(
    data, image.data, image.height * image.step * sizeof(uint8_t), cudaMemcpyDeviceToDevice);
}

CudaImage::CudaImage(const sensor_msgs::msg::Image & source)
{
  header = source.header;
  encoding = source.encoding;
  height = source.height;
  width = source.width;
  step = source.step;
  is_bigendian = source.is_bigendian;

  cudaMalloc(reinterpret_cast<void **>(&data), source.height * source.step * sizeof(uint8_t));
  cudaMemcpy(
    data, source.data.data(), source.height * source.step * sizeof(uint8_t),
    cudaMemcpyHostToDevice);
}

CudaImage::~CudaImage()
{
  cudaFree(data);
}

}  // namespace cuda_blackboard
