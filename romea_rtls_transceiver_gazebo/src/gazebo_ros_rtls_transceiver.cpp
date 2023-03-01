// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>
#include <vector>

// gazebo
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
{
}

//-----------------------------------------------------------------------------
void GazeboRosRTLSTransceiver::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
//   world_ = _model->GetWorld();

  std::string transceiver_name;
  if (_sdf->HasElement("name")) {
    _sdf->GetElement("name")->GetValue()->Get(transceiver_name);
  } else {
    gzthrow("RTLS transceiver name not found in sdf");
    return;
  }

  unsigned int transceiver_pan_id;
  if (_sdf->HasElement("pan_id")) {
    _sdf->GetElement("pan_id")->GetValue()->Get(transceiver_pan_id);
  } else {
    gzthrow("RTLS transceiver pan_id not found in sdf");
    return;
  }

  unsigned int transceiver_id;
  if (_sdf->HasElement("id")) {
    _sdf->GetElement("id")->GetValue()->Get(transceiver_id);
  } else {
    gzthrow("RTLS transceiver id not found in sdf");
    return;
  }

  std::string mode;
  if (_sdf->HasElement("mode")) {
    _sdf->GetElement("mode")->GetValue()->Get(mode);
  } else {
    gzthrow("RTLS transceiver mode not found in sdf");
    return;
  }


  if (_sdf->HasElement("xyz")) {
    std::istringstream iss(_sdf->GetElement("xyz")->GetValue()->GetAsString());
    std::cout << " read xyz " << iss.str() << std::endl;
    iss >> impl_->position.X();
    iss >> impl_->position.Y();
    iss >> impl_->position.Z();
  } else {
    gzthrow("Rtls transceiver xyz not found in sdf");
  }

  if (_sdf->HasElement("minimal_range")) {
    _sdf->GetElement("minimal_range")->GetValue()->Get(impl_->minimal_range);
    std::cout << transceiver_name << " minimal_range " << impl_->minimal_range << std::endl;
  } else {
    gzthrow("Rtls transceiver minimal range not found in sdf");
  }

  if (_sdf->HasElement("maximal_range")) {
    _sdf->GetElement("maximal_range")->GetValue()->Get(impl_->maximal_range);
    std::cout << transceiver_name << " maximal_range " << impl_->maximal_range << std::endl;
  } else {
    gzthrow("Rtls transceiver maximal range not found in sdf");
  }

  double noise_a;
  if (_sdf->HasElement("noise_a")) {
    _sdf->GetElement("noise_a")->GetValue()->Get(noise_a);
    std::cout << transceiver_name << " noise_a " << noise_a << std::endl;
  } else {
    gzthrow("Rtls transceiver noise_a not found in sdf");
  }

  double noise_b;
  if (_sdf->HasElement("noise_b")) {
    _sdf->GetElement("noise_b")->GetValue()->Get(noise_b);
    std::cout << transceiver_name << " noise_b " << noise_b << std::endl;
  } else {
    gzthrow("Rtls transceiver noise_a not found in sdf");
  }

  romea::RTLSTransceiverEUID transceiver_euid;
  impl_->range_noise = romea::RTLSRangeRandomNoise(noise_a, noise_b);

  std::cout << " transceiver_name " << transceiver_name << std::endl;
  std::cout << " transceiver_pand_id " << transceiver_pan_id << std::endl;
  std::cout << " transceiver_id " << transceiver_id << std::endl;

  if (mode != "standalone") {
    transceiver_euid.id = static_cast<uint16_t>(transceiver_id);
    transceiver_euid.pan_id = static_cast<uint16_t>(transceiver_pan_id);

    impl_->ros_interface = std::make_unique<GazeboRosRTLSTransceiverInterface>(
      gazebo_ros::Node::Get(_sdf), transceiver_name, transceiver_euid);
  }

  GazeboRosRTLSNetwork::Instance().add_transceiver(transceiver_euid, this);


//   std::string body_name;
//   if (_sdf->HasElement("bodyName")) {
//     body_name = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
//     std::cout << transceiver_name << " bodyName  " << body_name << std::endl;
//     link_ = _model->GetLink(body_name);
//     std::cout << " get links" << std::endl;
//     for (const physics::LinkPtr & linkptr : _model->GetLinks()) {
//       std::cout << linkptr->GetName() << std::endl;
//     }
//   } else {
//     gzthrow("Rtls transceiver bodyName not found in sdf");
//   }

//   if (_sdf->HasElement("linkName")) {
//     std::string link_name = _sdf->GetElement("linkName")->GetValue()->GetAsString();
//     std::cout << transceiver_name << " link_name " << link_name << std::endl;
//   } else {
//     gzthrow("Rtls transceiver linkName not found in sdf");
//   }


//   if (body_name.find("base_footprint") != std::string::npos) {
//     double z_offset = 0;
//     if (_sdf->HasElement("z_offset")) {
//       _sdf->GetElement("z_offset")->GetValue()->Get(z_offset);
//     }
//     position_.Z() += z_offset;
//   }

//   std::cout << transceiver_name << " position " << position_ << std::endl;

//   if (!link_) {
//     ROS_FATAL(
//       "GazeboRTLSTransceiverPlugin plugin error: bodyName: %s does not exist\n",
//       body_name.c_str());
//     return;
//   }


//   //  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
//   //                         boost::bind(&RTLSTransceiverPlugin::Update, this, _1));
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
    this->impl_->ros_interface->exchange_payloads(*responder->impl_->ros_interface);
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
