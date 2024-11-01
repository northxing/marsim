#ifndef MARSIM_GAZEBO_GAUSS_MARKOV_OCEAN_CURRENT_HH_
#define MARSIM_GAZEBO_GAUSS_MARKOV_OCEAN_CURRENT_HH_

#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>
#include <random>
#include <memory>

namespace marsim_gazebo
{
  class GaussMarkovOceanCurrent:
    public ::gz::sim::System,
    public ::gz::sim::ISystemConfigure,
    public ::gz::sim::ISystemPreUpdate
  {
    public: GaussMarkovOceanCurrent();

    public: void Configure(const ::gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ::gz::sim::EntityComponentManager &_ecm,
                           ::gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(const ::gz::sim::UpdateInfo &_info,
                           ::gz::sim::EntityComponentManager &_ecm) override;

    private: ::gz::math::Vector3d currentVelocity;
    private: ::gz::math::Vector3d mu;
    private: ::gz::math::Vector3d sigma;
    private: double beta;
    private: std::default_random_engine generator;
    private: std::normal_distribution<double> normalDistribution;

    private: std::unique_ptr<::gz::transport::Node> node;
    private: std::string currentTopic;
    private: ::gz::transport::Node::Publisher publisher;
  };
}

#endif