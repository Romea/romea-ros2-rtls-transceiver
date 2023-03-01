// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_TRANSCEIVER_HPP_
#define ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_TRANSCEIVER_HPP_

// std
#include <memory>
#include <vector>

// gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/GpsSensor.hh"
#include "gazebo/common/Events.hh"


namespace gazebo
{

class GazeboRosRTLSTransceiverPrivate;

class GazeboRosRTLSTransceiver : public gazebo::ModelPlugin
{
public:
  /// Constructor.
  GazeboRosRTLSTransceiver();

  /// Destructor.
  virtual ~GazeboRosRTLSTransceiver();

  /// Inherited
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  std::optional<double> ComputeRange(const GazeboRosRTLSTransceiver * responder);

private:
  std::unique_ptr<GazeboRosRTLSTransceiverPrivate> impl_;
};

}  // namespace gazebo

#endif  // ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_TRANSCEIVER_HPP_
