// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <memory>
#include <string>
#include <vector>

// gazebo
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Collision.hh"

// ros
#include "gazebo_ros/node.hpp"


// romea
#include "romea_core_rtls_transceiver/RTLSRangeRandomNoise.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface_server.hpp"

// local
#include "romea_rtls_transceiver_gazebo/gazebo_ros_rtls_transceiver.hpp"

namespace gazebo
{

class GazeboRosRTLSTransceiverPrivate
{
public:
  /// A pointer to the Link, where force is applied
  gazebo::physics::LinkPtr link;

  /// ros node
  std::shared_ptr<rclcpp::Node> node;

  /// ros interface
  // std::shared_ptr<GazeboRosRTLSTransceiverInterface> ros_interface;

  /// ros interface server
  std::shared_ptr<romea::RTLSTransceiverInterfaceServer> ros_interface_server;

  /// body position according parent link
  ignition::math::Vector3d position;

  ///  transceiver idientifier
  romea::RTLSTransceiverEUID euid;

  /// ranging noise characteristics
  romea::RTLSRangeRandomNoise range_noise;

  /// minimal distance for range
  double minimal_range;

  /// maxiimal distance for range
  double maximal_range;

  /// payload send by user
  std::vector<uint8_t> payload;


  ignition::math::Vector3d compute_world_position() const;
  bool is_available_range(const double & range)const;
  double compute_noised_range(const double & range);

  void process_request(romea_rtls_transceiver_msgs::msg::RangingRequest::ConstSharedPtr msg);
  void send_result(const uint16_t & responder_id, const double & range);
  void send_payload(const std::vector<uint8_t> payload);
};

class GazeboRosRTLSNetwork
{
public:
  // delete copy and move constructors and assign operators
  GazeboRosRTLSNetwork(GazeboRosRTLSNetwork const &) = delete;             // Copy construct
  GazeboRosRTLSNetwork(GazeboRosRTLSNetwork &&) = delete;                  // Move construct
  GazeboRosRTLSNetwork & operator=(GazeboRosRTLSNetwork const &) = delete;  // Copy assign
  GazeboRosRTLSNetwork & operator=(GazeboRosRTLSNetwork &&) = delete;      // Move assign

protected:
  GazeboRosRTLSNetwork();

public:
  static GazeboRosRTLSNetwork & Instance();

  void add_transceiver(GazeboRosRTLSTransceiverPrivate * transceiver);

  void ranging(
    romea::RTLSTransceiverEUID initiator_euid,
    romea::RTLSTransceiverEUID responder_euid);

private:
  std::map<romea::RTLSTransceiverEUID, GazeboRosRTLSTransceiverPrivate *> transceivers_;
};


//-----------------------------------------------------------------------------
GazeboRosRTLSTransceiver::GazeboRosRTLSTransceiver()
: impl_(std::make_unique<GazeboRosRTLSTransceiverPrivate>())
{
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSTransceiver::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // std::cout << " model name " << _model->GetName() << std::endl;

  std::string transceiver_link_name;
  if (_sdf->HasElement("link_name")) {
    _sdf->GetElement("link_name")->GetValue()->Get(transceiver_link_name);
  } else {
    gzthrow("RTLS transceiver link_name not found in sdf");
    return;
  }

  // std::cout << "transceiver_link_name" << transceiver_link_name << std::endl;
  //impl->link = _model->GetLink(transceiver_link_name);
  // get canonical link because transceiver link is not defined in sdf file due to lumb
  impl_->link = _model->GetLink();

  std::string transceiver_name;
  if (_sdf->HasElement("transceiver_name")) {
    _sdf->GetElement("transceiver_name")->GetValue()->Get(transceiver_name);
  } else {
    gzthrow("RTLS transceiver name not found in sdf");
    return;
  }

  // std::cout << "transceiver_name" << transceiver_name << std::endl;

  unsigned int transceiver_pan_id;
  if (_sdf->HasElement("transceiver_pan_id")) {
    _sdf->GetElement("transceiver_pan_id")->GetValue()->Get(transceiver_pan_id);
  } else {
    gzthrow("RTLS transceiver pan_id not found in sdf");
    return;
  }
  // std::cout << "transceiver_pan_id" << transceiver_pan_id << std::endl;

  unsigned int transceiver_id;
  if (_sdf->HasElement("transceiver_id")) {
    _sdf->GetElement("transceiver_id")->GetValue()->Get(transceiver_id);
  } else {
    gzthrow("RTLS transceiver id not found in sdf");
    return;
  }

  // std::cout << "transceiver_id" << transceiver_id << std::endl;

  std::string mode;
  if (_sdf->HasElement("mode")) {
    _sdf->GetElement("mode")->GetValue()->Get(mode);
  } else {
    gzthrow("RTLS transceiver mode not found in sdf");
    return;
  }

  // Collision position is the same of transceiver position
  auto collisions = impl_->link->GetCollisions();
  auto it = std::find_if(
    collisions.begin(),
    collisions.end(),
    [&transceiver_link_name](const gazebo::physics::CollisionPtr & collision)
    {
      return collision->GetName().find(transceiver_link_name) != std::string::npos;
    });

  assert(it != impl_->link->GetCollisions().end());
  impl_->position = (*it)->RelativePose().Pos();

  // if (_sdf->HasElement("xyz")) {
  //   std::istringstream iss(_sdf->GetElement("xyz")->GetValue()->GetAsString());
  //   iss >> impl_->position.X();
  //   iss >> impl_->position.Y();
  //   iss >> impl_->position.Z();
  // } else {
  //   gzthrow("Rtls transceiver xyz not found in sdf");
  // }

  if (_sdf->HasElement("minimal_range")) {
    _sdf->GetElement("minimal_range")->GetValue()->Get(impl_->minimal_range);
    // std::cout << transceiver_name << " minimal_range " << impl_->minimal_range << std::endl;
  } else {
    gzthrow("Rtls transceiver minimal range not found in sdf");
  }

  if (_sdf->HasElement("maximal_range")) {
    _sdf->GetElement("maximal_range")->GetValue()->Get(impl_->maximal_range);
    // std::cout << transceiver_name << " maximal_range " << impl_->maximal_range << std::endl;
  } else {
    gzthrow("Rtls transceiver maximal range not found in sdf");
  }

  double noise_a;
  if (_sdf->HasElement("noise_a")) {
    _sdf->GetElement("noise_a")->GetValue()->Get(noise_a);
    // std::cout << transceiver_name << " noise_a " << noise_a << std::endl;
  } else {
    gzthrow("Rtls transceiver noise_a not found in sdf");
  }

  double noise_b;
  if (_sdf->HasElement("noise_b")) {
    _sdf->GetElement("noise_b")->GetValue()->Get(noise_b);
    // std::cout << transceiver_name << " noise_b " << noise_b << std::endl;
  } else {
    gzthrow("Rtls transceiver noise_a not found in sdf");
  }

  impl_->range_noise = romea::RTLSRangeRandomNoise(noise_a, noise_b);
  impl_->euid.pan_id = static_cast<uint16_t>(transceiver_pan_id);
  impl_->euid.id = static_cast<uint16_t>(transceiver_id);

  // std::cout << " transceiver_name " << transceiver_name << std::endl;
  // std::cout << " transceiver_pand_id " << transceiver_pan_id << std::endl;
  // std::cout << " transceiver_id " << transceiver_id << std::endl;
  // std::cout << " transceiver_position " << impl_->position[0] << " " << impl_->position[1] << " " <<
  //   impl_->position[2] << std::endl;

  if (mode != "standalone") {
    impl_->node = gazebo_ros::Node::Get(_sdf);

    romea::RTLSTransceiverInterfaceServer::RangingRequestCallback request_callback = std::bind(
      &GazeboRosRTLSTransceiverPrivate::process_request, impl_.get(), std::placeholders::_1);

    using ROSInterfaceServer = romea::RTLSTransceiverInterfaceServer;
    impl_->ros_interface_server = std::make_shared<ROSInterfaceServer>(
      impl_->node, request_callback);
  }

  GazeboRosRTLSNetwork::Instance().add_transceiver(impl_.get());
}

//-----------------------------------------------------------------------------
ignition::math::Vector3d GazeboRosRTLSTransceiverPrivate::compute_world_position() const
{
#if GAZEBO_MAJOR_VERSION < 8
  ignition::math::Pose3d pose = link->GetWorldPose().Ign();
#else
  ignition::math::Pose3d pose = link->WorldPose();
#endif
  return pose.Pos() + pose.Rot() * position;
}

//-----------------------------------------------------------------------------
double GazeboRosRTLSTransceiverPrivate::compute_noised_range(const double & range)
{
  return range + range_noise.draw(range);
}

//-----------------------------------------------------------------------------
bool GazeboRosRTLSTransceiverPrivate::is_available_range(const double & range)const
{
  return range >= minimal_range && range <= maximal_range;
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSTransceiverPrivate::send_result(
  const uint16_t & responder_id,
  const double & range)
{
  romea_rtls_transceiver_msgs::msg::RangingResult result_msg;
  result_msg.initiator_id = euid.id;
  result_msg.responder_id = responder_id;
  result_msg.range.stamp = node->get_clock()->now();
  result_msg.range.range = range;
  result_msg.range.total_rx_power_level = 255;
  result_msg.range.first_path_rx_power_level = 255;
  ros_interface_server->send_ranging_result(result_msg);
  // TODO(Jean) simulate channel transmission
  // TODO(Jean) simulate power loss
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSTransceiverPrivate::send_payload(const std::vector<uint8_t> payload)
{
  romea_rtls_transceiver_msgs::msg::Payload payload_msg;
  payload_msg.data = payload;
  ros_interface_server->send_payload(payload_msg);
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSTransceiverPrivate::process_request(
  romea_rtls_transceiver_msgs::msg::RangingRequest::ConstSharedPtr msg)
{
  payload = msg->payload.data;
  if (msg->responder_id != std::numeric_limits<uint16_t>::max()) {
    GazeboRosRTLSNetwork::Instance().ranging(euid, {euid.pan_id, msg->responder_id});
    payload.clear();
  }
}

//-----------------------------------------------------------------------------
GazeboRosRTLSNetwork & GazeboRosRTLSNetwork::Instance()
{
  static GazeboRosRTLSNetwork myInstance;
  return myInstance;
}

//-----------------------------------------------------------------------------
GazeboRosRTLSNetwork::GazeboRosRTLSNetwork()
{
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSNetwork::add_transceiver(GazeboRosRTLSTransceiverPrivate * transceiver)
{
  transceivers_[transceiver->euid] = transceiver;
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSNetwork::ranging(
  romea::RTLSTransceiverEUID initiator_euid,
  romea::RTLSTransceiverEUID responder_euid)
{
  GazeboRosRTLSTransceiverPrivate * initiator = transceivers_[initiator_euid];
  GazeboRosRTLSTransceiverPrivate * responder = transceivers_[responder_euid];


  if (responder != nullptr && initiator != nullptr) {
    auto initiator_position = initiator->compute_world_position();
    auto responder_position = responder->compute_world_position();
    double distance = initiator_position.Distance(responder_position);
    double range = initiator->compute_noised_range(distance);

    if (initiator->is_available_range(range)) {
      initiator->send_result(responder_euid.id, range);

      if (!responder->payload.empty()) {
        initiator->send_payload(responder->payload);
        responder->payload.clear();
      }

      if (!initiator->payload.empty()) {
        responder->send_payload(initiator->payload);
        initiator->payload.clear();
      }
    }
  }
}


// Register this sensor with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRTLSTransceiver)

}  // namespace gazebo
