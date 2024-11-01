/**
 * @file DVLRosConverter.cc
 *
 * @brief This plugin converts Doppler Velocity Log (DVL) messages from Gazebo format to ROS-compatible format.
 *
 * The DVLRosConverter is designed to bridge the gap between Gazebo's native DVL message format and the format
 * expected by ROS-based systems. This is particularly useful in underwater vehicle simulations where DVL data
 * needs to be integrated with ROS-based control and navigation systems.
 *
 * Key Features:
 * - **Message Conversion**: Transforms Gazebo's DVLVelocityTracking messages into ROS-compatible TwistWithCovariance and Float messages.
 * - **Velocity Processing**: Extracts linear velocity from the DVL message and sets it in the TwistWithCovariance message.
 * - **Covariance Handling**: Transfers the covariance matrix from the DVL message to the TwistWithCovariance message.
 * - **Range Data**: Extracts the range data from the DVL message and publishes it as a separate Float message.
 * - **Configurable Topics**: Allows customization of input and output topic names through SDF parameters.
 *
 * The plugin subscribes to a Gazebo DVL topic, processes incoming messages, and publishes the converted data
 * on ROS-compatible topics. This enables seamless integration of simulated DVL data with ROS-based systems.
 *
 * Configuration Parameters (SDF file):
 * - `input_topic`: The Gazebo topic to subscribe to for DVL messages.
 * - `velocity_output_topic`: The topic to publish converted velocity messages.
 * - `range_output_topic`: The topic to publish converted range messages.
 *
 * The conversion process occurs in the `OnDVLMsg` callback function, which is triggered whenever a new DVL
 * message is received. The function performs the following steps:
 * 1. Extracts linear velocity from the DVL message.
 * 2. Sets the linear velocity in a TwistWithCovariance message.
 * 3. Transfers the covariance matrix from the DVL message to the TwistWithCovariance message.
 * 4. Publishes the TwistWithCovariance message on the velocity output topic.
 * 5. Extracts the range data and publishes it as a Float message on the range output topic.
 *
 * This plugin is essential for applications requiring accurate simulation of underwater vehicles equipped
 * with DVL sensors, especially when integrating with ROS-based control systems or when testing navigation
 * algorithms that rely on DVL data.

 * Example usage in an SDF file:
    <plugin
      filename="libDVLRosConverter.so"
      name="marsim_gazebo::DVLRosConverter">
      <input_topic>/model/bluerov2/dvl/velocity</input_topic>
      <velocity_output_topic>/dvl_ros_compatible/velocity</velocity_output_topic>
      <range_output_topic>/dvl_ros_compatible/range</range_output_topic>
    </plugin>

 */

#include "DVLRosConverter.hh"

#include <gz/plugin/Register.hh>
#include <gz/msgs/twist_with_covariance.pb.h>
#include <gz/msgs/float.pb.h>

namespace marsim_gazebo
{

DVLRosConverter::DVLRosConverter() = default;

void DVLRosConverter::Configure(const ::gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                ::gz::sim::EntityComponentManager &_ecm,
                                ::gz::sim::EventManager &_eventMgr)
{
  // Read topic names from SDF
  this->inputTopic = _sdf->Get<std::string>("input_topic", "/model/bluerov2/dvl/velocity").first;
  this->velocityOutputTopic = _sdf->Get<std::string>("velocity_output_topic", "/dvl_ros_compatible/velocity").first;
  this->rangeOutputTopic = _sdf->Get<std::string>("range_output_topic", "/dvl_ros_compatible/range").first;

  this->node = std::make_unique<gz::transport::Node>();
  
  // Subscribe to the input topic
  if (!this->node->Subscribe(this->inputTopic, &DVLRosConverter::OnDVLMsg, this))
  {
    gzerr << "Error subscribing to topic [" << this->inputTopic << "]" << std::endl;
  }

  // Advertise the output topics
  this->velocityPub = this->node->Advertise<::gz::msgs::TwistWithCovariance>(this->velocityOutputTopic);
  this->rangePub = this->node->Advertise<::gz::msgs::Float>(this->rangeOutputTopic);

  gzdbg << "DVLRosConverter initialized with topics:" << std::endl
        << "  Input: " << this->inputTopic << std::endl
        << "  Velocity Output: " << this->velocityOutputTopic << std::endl
        << "  Range Output: " << this->rangeOutputTopic << std::endl;
}

void DVLRosConverter::PreUpdate(const ::gz::sim::UpdateInfo &_info,
                                ::gz::sim::EntityComponentManager &_ecm)
{
  // This method is called on every simulation step
  // We don't need to do anything here as we're using a callback-based approach
}

void DVLRosConverter::OnDVLMsg(const ::gz::msgs::DVLVelocityTracking &msg)
{
  // Convert velocity and covariance to ROS-compatible TwistWithCovariance format
  ::gz::msgs::TwistWithCovariance twistMsg;
  
  // Store timestamp locally
  int64_t sec = msg.header().stamp().sec();
  int64_t nsec = msg.header().stamp().nsec();

  // Set linear velocity
  auto linear = twistMsg.mutable_twist()->mutable_linear();
  linear->set_x(msg.velocity().mean().x());
  linear->set_y(msg.velocity().mean().y());
  linear->set_z(msg.velocity().mean().z());
  
  // Set angular velocity to zero (DVL typically doesn't measure angular velocity)
  auto angular = twistMsg.mutable_twist()->mutable_angular();
  angular->set_x(0);
  angular->set_y(0);
  angular->set_z(0);
  
  // Set covariance
  auto covariance = twistMsg.mutable_covariance();
  covariance->Clear();
  for (int i = 0; i < 36; ++i) {
    if (i < 9) {
      covariance->add_data(msg.velocity().covariance(i));
    } else {
      covariance->add_data(0); // Fill the rest with zeros
    }
  }

  // Publish velocity message
  this->velocityPub.Publish(twistMsg);

  // Convert range to ROS-compatible Float format
  ::gz::msgs::Float rangeMsg;
  rangeMsg.set_data(msg.target().range().mean());

  // Publish range message
  this->rangePub.Publish(rangeMsg);
}

} // namespace marsim_gazebo

GZ_ADD_PLUGIN(marsim_gazebo::DVLRosConverter,
              ::gz::sim::System,
              marsim_gazebo::DVLRosConverter::ISystemConfigure,
              marsim_gazebo::DVLRosConverter::ISystemPreUpdate)