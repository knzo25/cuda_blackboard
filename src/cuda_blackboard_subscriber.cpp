
#include "cuda_blackboard/cuda_blackboard_subscriber.hpp"

#include "cuda_blackboard/cuda_blackboard.hpp"
#include "cuda_blackboard/negotiated_types.hpp"

#include <functional>

namespace cuda_blackboard
{

template <typename T>
CudaBlackboardSubscriber<T>::CudaBlackboardSubscriber(
  rclcpp::Node & node, const std::string & topic_name, bool add_compatible_sub,
  std::function<void(std::shared_ptr<const T>)> callback)
: node_(node)
{
  using std::placeholders::_1;

  negotiated::NegotiatedSubscriptionOptions negotiation_options;
  negotiation_options.disconnect_on_negotiation_failure = false;

  callback_ = callback;
  negotiated_sub_ = std::make_shared<negotiated::NegotiatedSubscription>(
    node, topic_name + "/cuda", negotiation_options);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  negotiated_sub_->add_supported_callback<NegotiationStruct<T>>(
    1.0, rclcpp::QoS(1), std::bind(&CudaBlackboardSubscriber<T>::instanceIdCallback, this, _1),
    sub_options);

  if (add_compatible_sub) {
    compatible_sub_ =
      node.create_subscription<T>(topic_name, rclcpp::SensorDataQoS(), callback_, sub_options);
  }

  negotiated_sub_->start();
}

template <typename T>
void CudaBlackboardSubscriber<T>::instanceIdCallback(const std_msgs::msg::UInt64 & instance_id_msg)
{
  auto & blackboard = CudaBlackboard<T>::getInstance();
  auto data = blackboard.queryData(instance_id_msg.data);
  if (data) {
    callback_(data);
  } else {
    RCLCPP_ERROR_STREAM(
      node_.get_logger(), "There was not data with the requested instance id= "
                            << instance_id_msg.data << " in the blackboard.");
  }
}

}  // namespace cuda_blackboard

template class cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>;
template class cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaImage>;
