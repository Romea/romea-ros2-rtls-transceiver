// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>
#include <vector>

// gazebo
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"

// ros
#include "gazebo_ros/node.hpp"


// romea
#include "romea_core_rtls/RTLSRangeRandomNoise.hpp"

// local
#include "romea_rtls_transceiver_gazebo/gazebo_ros_rtls_network.hpp"
#include "romea_rtls_transceiver_gazebo/gazebo_ros_rtls_transceiver.hpp"
#include "romea_rtls_transceiver_gazebo/gazebo_ros_rtls_transceiver_interface.hpp"


namespace gazebo
{

class GazeboRosRTLSTransceiverPrivate
{
public:
  /// A pointer to the Link, where force is applied
  gazebo::physics::LinkPtr link;

  /// ros action and service interfaces
  std::unique_ptr<GazeboRosRTLSTransceiverInterface> ros_interface;

  /// position according link
  ignition::math::Vector3d position;

  /// ranging noise characteristics
  romea::RTLSRangeRandomNoise range_noise;

  /// minimal distance for range
  double minimal_range;

  /// maxiimal distance for range
  double maximal_range;

  ignition::math::Vector3d compute_world_position() const;
  bool is_available_range(const double & range)const;
  double compute_noised_range(const double & range);
};

//-----------------------------------------------------------------------------
GazeboRosRTLSTransceiver::GazeboRosRTLSTransceiver()
: impl_(std::make_unique<GazeboRosRTLSTransceiverPrivate>())
{
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSTransceiver::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::string transceiver_link_name;
  if (_sdf->HasElement("link_name")) {
    _sdf->GetElement("link_name")->GetValue()->Get(transceiver_link_name);
  } else {
    gzthrow("RTLS transceiver link_name not found in sdf");
    return;
  }
  // std::cout << "transceiver_link_name " << transceiver_link_name << std::endl;

  impl_->link = _model->GetLink(transceiver_link_name);

  std::string transceiver_name;
  if (_sdf->HasElement("name")) {
    _sdf->GetElement("name")->GetValue()->Get(transceiver_name);
  } else {
    gzthrow("RTLS transceiver name not found in sdf");
    return;
  }

  // std::cout << "transceiver_name" << transceiver_name << std::endl;

  unsigned int transceiver_pan_id;
  if (_sdf->HasElement("pan_id")) {
    _sdf->GetElement("pan_id")->GetValue()->Get(transceiver_pan_id);
  } else {
    gzthrow("RTLS transceiver pan_id not found in sdf");
    return;
  }
  // std::cout << "transceiver_pan_id" << transceiver_pan_id << std::endl;

  unsigned int transceiver_id;
  if (_sdf->HasElement("id")) {
    _sdf->GetElement("id")->GetValue()->Get(transceiver_id);
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

  if (_sdf->HasElement("xyz")) {
    std::istringstream iss(_sdf->GetElement("xyz")->GetValue()->GetAsString());
    iss >> impl_->position.X();
    iss >> impl_->position.Y();
    iss >> impl_->position.Z();
  } else {
    gzthrow("Rtls transceiver xyz not found in sdf");
  }

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

  romea::RTLSTransceiverEUID transceiver_euid;
  transceiver_euid.pan_id = static_cast<uint16_t>(transceiver_pan_id);
  transceiver_euid.id = static_cast<uint16_t>(transceiver_id);

  impl_->range_noise = romea::RTLSRangeRandomNoise(noise_a, noise_b);

  // std::cout << " transceiver_name " << transceiver_name << std::endl;
  // std::cout << " transceiver_pand_id " << transceiver_pan_id << std::endl;
  // std::cout << " transceiver_id " << transceiver_id << std::endl;

  if (mode != "standalone") {
    impl_->ros_interface = std::make_unique<GazeboRosRTLSTransceiverInterface>(
      gazebo_ros::Node::Get(_sdf), transceiver_name, transceiver_euid);
  }

  GazeboRosRTLSNetwork::Instance().add_transceiver(transceiver_euid, this);
}

//-----------------------------------------------------------------------------
std::optional<double> GazeboRosRTLSTransceiver::ComputeRange(
  const GazeboRosRTLSTransceiver * responder)
{
  ignition::math::Vector3d initiator_position =
    this->impl_->compute_world_position();
  ignition::math::Vector3d responder_position =
    responder->impl_->compute_world_position();
  double distance = initiator_position.Distance(responder_position);

  if (this->impl_->is_available_range(distance) &&
    responder->impl_->is_available_range(distance))
  {
    if (responder->impl_->ros_interface != nullptr) {
      this->impl_->ros_interface->exchange_payloads(
        *responder->impl_->ros_interface);
    }
    return this->impl_->compute_noised_range(distance);
  } else {
    return {};
  }
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

// Register this sensor with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRTLSTransceiver)

}  // namespace gazebo
