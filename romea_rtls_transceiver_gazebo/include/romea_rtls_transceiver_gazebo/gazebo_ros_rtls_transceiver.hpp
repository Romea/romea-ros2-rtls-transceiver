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
  virtual ~GazeboRosRTLSTransceiver() = default;

  /// Inherited
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // std::optional<double> ComputeRange(const GazeboRosRTLSTransceiver * responder);

private:
  std::unique_ptr<GazeboRosRTLSTransceiverPrivate> impl_;
};

}  // namespace gazebo

#endif  // ROMEA_RTLS_TRANSCEIVER_GAZEBO__GAZEBO_ROS_RTLS_TRANSCEIVER_HPP_
