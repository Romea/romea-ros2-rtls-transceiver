// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>
#include <utility>

// romea ros
#include "romea_common_utils/qos.hpp"
// romea
#include "romea_common_utils/params/node_parameters.hpp"

// local
#include "romea_rtls_transceiver_hub/rtls_transceiver_hub.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
RTLSTransceiverHub::RTLSTransceiverHub(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("cmd_mux", options)),
  range_pub_(nullptr),
  poll_sub_(nullptr),
  payload_sub_(nullptr),
  set_external_transceivers_configuration_service_client_(nullptr),
  embedded_transceivers_(),
  external_transceivers_ids_()
{
  init_range_pub_();
  init_poll_sub_();
  init_payload_sub_();
  init_embedded_transceivers_inferfaces_();
  init_external_transceivers_configuration_service_client_();
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
RTLSTransceiverHub::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}


//-----------------------------------------------------------------------------
void RTLSTransceiverHub::init_poll_sub_()
{
  using namespace std::placeholders;
  auto callback = std::bind(&RTLSTransceiverHub::poll_callback_, this, _1);

  rclcpp::SubscriptionOptions options;
  options.callback_group = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  poll_sub_ = node_->create_subscription<Poll>("poll", best_effort(1), callback, options);
}

//-----------------------------------------------------------------------------
void RTLSTransceiverHub::init_payload_sub_()
{
  using namespace std::placeholders;
  auto callback = std::bind(&RTLSTransceiverHub::payload_callback_, this, _1);

  rclcpp::SubscriptionOptions options;
  options.callback_group = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  payload_sub_ = node_->create_subscription<Payload>("payload", best_effort(1), callback, options);
}

//-----------------------------------------------------------------------------
void RTLSTransceiverHub::init_external_transceivers_configuration_service_client_()
{
  using namespace std::placeholders;

  auto callback = std::bind(
    &RTLSTransceiverHub::set_external_transceivers_configuration_callback_, this, _1, _2);

  set_external_transceivers_configuration_service_client_ =
    node_->create_service<SetTranceiversConfiguration>(
    "set_external_transceivers_configuration", callback);
}

//-----------------------------------------------------------------------------
void RTLSTransceiverHub::init_embedded_transceivers_inferfaces_()
{
  declare_vector_parameter<std::string>(node_, "embedded_transceivers_names");
  // declare_vector_parameter<int>(node_, "embedded_transceivers_ids");

  auto embedded_transceivers_names = get_vector_parameter<std::string>(
    node_, "embedded_transceivers_names");

  // auto embedded_transceivers_ids = get_vector_parameter<int>(
  //   node_, "connected_transceivers_ids");

  // assert(embedded_transceivers_names.size() == embedded_transceivers_ids.size());

  for (const auto & transceiver_name : embedded_transceivers_names) {
    embedded_transceivers_[transceiver_name] =
      std::make_unique<TransceiverInterfaceClient>(node_, transceiver_name);
  }
}

//-----------------------------------------------------------------------------
void RTLSTransceiverHub::init_range_pub_()
{
  range_pub_ = node_->create_publisher<Range>("range", sensor_data_qos());
}

//-----------------------------------------------------------------------------
void RTLSTransceiverHub::poll_callback_(Poll::SharedPtr poll_msg)
{
  if (!external_transceivers_ids_.empty()) {
    auto & initiator = embedded_transceivers_[poll_msg->transceivers.initiator_name];
    auto & responder_id = external_transceivers_ids_[poll_msg->transceivers.responder_name];

    if (!poll_msg->payload.data.empty()) {
      initiator->set_payload(poll_msg->payload);
    }

    auto range_msg = std::make_unique<Range>();
    range_msg->transceivers = poll_msg->transceivers;
    if (initiator->ranging(responder_id, range_msg->range, std::chrono::seconds(1))) {
      initiator->get_payload(range_msg->payload);
    }
    range_pub_->publish(std::move(range_msg));
  }
}

//-----------------------------------------------------------------------------
void RTLSTransceiverHub::payload_callback_(Payload::SharedPtr payload_msg)
{
  for (auto & [name, transceiver] : embedded_transceivers_) {
    transceiver->set_payload(*payload_msg);
  }
}

//-----------------------------------------------------------------------------
void RTLSTransceiverHub::set_external_transceivers_configuration_callback_(
  const std::shared_ptr<SetTranceiversConfiguration::Request> request,
  std::shared_ptr<SetTranceiversConfiguration::Response>/*response*/)
{
  assert(request->transceivers_names.size() == request->transceivers_ids.size());

  for (size_t n = 0; n < request->transceivers_names.size(); ++n) {
    external_transceivers_ids_[request->transceivers_names[n]] =
      request->transceivers_ids[n];
  }
}

}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::RTLSTransceiverHub)
