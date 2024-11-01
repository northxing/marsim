#ifndef MARSIM_GAZEBO_DVL_ROS_CONVERTER_HH_
#define MARSIM_GAZEBO_DVL_ROS_CONVERTER_HH_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/dvl_velocity_tracking.pb.h>

namespace marsim_gazebo
{
class DVLRosConverter : public ::gz::sim::System,
                        public ::gz::sim::ISystemConfigure,
                        public ::gz::sim::ISystemPreUpdate
{
  public: DVLRosConverter();

  public: void Configure(const ::gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ::gz::sim::EntityComponentManager &_ecm,
                         ::gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(const ::gz::sim::UpdateInfo &_info,
                         ::gz::sim::EntityComponentManager &_ecm) override;

  private: void OnDVLMsg(const gz::msgs::DVLVelocityTracking &msg);

  private: 
    std::unique_ptr<gz::transport::Node> node;
    gz::transport::Node::Publisher velocityPub;
    gz::transport::Node::Publisher rangePub;
    std::string inputTopic;
    std::string velocityOutputTopic;
    std::string rangeOutputTopic;
};
}

#endif  // MARSIM_GAZEBO_DVL_ROS_CONVERTER_HH_