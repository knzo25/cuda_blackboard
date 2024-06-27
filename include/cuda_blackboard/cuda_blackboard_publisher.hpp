
#pragma once

#include "cuda_blackboard/negotiated_types.hpp"

#include <negotiated/negotiated_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace cuda_blackboard
{

template <typename T>
class CudaBlackboardPublisher
{
public:
  CudaBlackboardPublisher(rclcpp::Node & node, const std::string & topic_name);
  void publish(std::unique_ptr<const T> msg);

  std::size_t get_subscription_count() const;
  std::size_t get_intra_process_subscription_count() const;

private:
  rclcpp::Node & node_;
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;
  typename rclcpp::Publisher<typename T::ros_type>::SharedPtr compatible_pub_;
};

}  // namespace cuda_blackboard
