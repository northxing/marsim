/**
 * @file GaussMarkovOceanCurrent.cc
 *
 * @brief This plugin simulates ocean currents using a Gauss-Markov process within the Gazebo simulation environment.
 *
 * The Gauss-Markov process is a stochastic model used to simulate time-varying ocean currents with a controlled 
 * level of randomness (noise). It allows the simulation of realistic ocean current behavior that converges 
 * towards a mean value while maintaining a certain level of unpredictability. This is useful in scenarios 
 * like marine robotics or maritime simulations where environmental factors need to be considered.
 *
 * Key Features:
 * - **mu (drift)**: Represents the mean or target velocity that the ocean current converges towards over time.
 * - **sigma (volatility)**: Controls the magnitude of the random fluctuations (noise) in each direction.
 * - **beta (rate of decay)**: Controls how quickly the current velocity reverts to the mean velocity.
 * - **Gaussian noise**: The random component is drawn from a normal (Gaussian) distribution to add stochastic 
 *   variations to the velocity.
 *
 * The plugin utilizes the Gazebo transport system to publish the simulated current velocity to a specific 
 * topic, which can be subscribed to by other systems in the simulation.
 *
 * Configuration Parameters (SDF file):
 * - `mu`: Vector3 (x, y, z) mean velocity of the current.
 * - `sigma`: Vector3 (x, y, z) standard deviation of the noise.
 * - `beta`: Scalar value controlling the rate of decay towards the mean velocity.
 * - `seed`: Optional random seed to initialize the random number generator for reproducibility.
 * - `namespace`: Optional namespace to customize the topic name for publishing the current velocity.
 *
 * The velocity is updated on each simulation step in the `PreUpdate` function, which calculates the 
 * next value of the current using the following formula:
 * 
 *     currentVelocity = currentVelocity * exp(-beta * dt)
 *                       + mu * (1 - exp(-beta * dt))
 *                       + sigma * sqrt(1 - exp(-2 * beta * dt)) * noise
 *
 * This ensures that the velocity smoothly evolves over time with random noise, creating realistic ocean-like dynamics.
 *
 * The plugin is useful for applications requiring simulated environmental dynamics such as underwater vehicles, 
 * marine navigation systems, and environmental testing of ocean-based robotics.

 * Example usage in a sdf file
    <plugin
      filename="libGaussMarkovOceanCurrent.so"
      name="marsim_gazebo::GaussMarkovOceanCurrent">
      <mu>0.0</mu>
      <sigma>0.1 0.1 0.1</sigma>
      <beta>0.1</beta>
      <namespace>bluerov2</namespace>
      <seed>12345</seed>
    </plugin>

*/

#include "GaussMarkovOceanCurrent.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <iostream>
#include <chrono>
#include <sstream>

namespace marsim_gazebo
{
  GZ_ADD_PLUGIN(
    GaussMarkovOceanCurrent,
    ::gz::sim::System,
    GaussMarkovOceanCurrent::ISystemConfigure,
    GaussMarkovOceanCurrent::ISystemPreUpdate
  )

  GaussMarkovOceanCurrent::GaussMarkovOceanCurrent()
    : mu(::gz::math::Vector3d::Zero), 
      sigma(::gz::math::Vector3d(0.1, 0.1, 0.1)), 
      beta(0.1), 
      normalDistribution(0.0, 1.0)
  {
  }

  void GaussMarkovOceanCurrent::Configure(const ::gz::sim::Entity &_entity,
                                          const std::shared_ptr<const sdf::Element> &_sdf,
                                          ::gz::sim::EntityComponentManager &_ecm,
                                          ::gz::sim::EventManager &_eventMgr)
  {
    if (_sdf->HasElement("mu"))
    {
      std::string muStr = _sdf->Get<std::string>("mu");
      std::istringstream iss(muStr);
      iss >> this->mu.X() >> this->mu.Y() >> this->mu.Z();
    }
    
    if (_sdf->HasElement("sigma"))
    {
      std::string sigmaStr = _sdf->Get<std::string>("sigma");
      std::istringstream iss(sigmaStr);
      iss >> this->sigma.X() >> this->sigma.Y() >> this->sigma.Z();
    }
    
    if (_sdf->HasElement("beta"))
      this->beta = _sdf->Get<double>("beta");

    // Set up the random number generator
    unsigned seed;
    if (_sdf->HasElement("seed"))
    {
      seed = _sdf->Get<unsigned>("seed");
    }
    else
    {
      seed = std::chrono::system_clock::now().time_since_epoch().count();
    }
    this->generator.seed(seed);

    this->currentVelocity = ::gz::math::Vector3d::Zero;

    // Initialize the transport node
    this->node = std::make_unique<::gz::transport::Node>();
    
    // Check if a namespace is provided
    if (_sdf->HasElement("namespace"))
    {
      std::string ns = _sdf->Get<std::string>("namespace");
      this->currentTopic = "/model/" + ns + "/ocean_current";
    }
    else
    {
      // Use the default topic if no namespace is provided
      this->currentTopic = "/ocean_current";
    }
    
    // Advertise the topic
    this->publisher = this->node->Advertise<::gz::msgs::Vector3d>(this->currentTopic);

    std::cout << "GaussMarkovOceanCurrent plugin publishing to topic: " 
              << this->currentTopic << " with seed: " << seed << std::endl;
    std::cout << "Parameters: mu = " << this->mu << ", sigma = " << this->sigma 
              << ", beta = " << this->beta << std::endl;
  }

  void GaussMarkovOceanCurrent::PreUpdate(const ::gz::sim::UpdateInfo &_info,
                                          ::gz::sim::EntityComponentManager &_ecm)
  {
    double dt = _info.dt.count();

    ::gz::math::Vector3d noise(
      this->normalDistribution(this->generator),
      this->normalDistribution(this->generator),
      this->normalDistribution(this->generator));

    this->currentVelocity = this->currentVelocity * std::exp(-this->beta * dt) +
                            this->mu * (1 - std::exp(-this->beta * dt)) +
                            this->sigma * std::sqrt(1 - std::exp(-2 * this->beta * dt)) * noise;

    // Publish the current velocity to the Gazebo topic
    ::gz::msgs::Vector3d msg;
    msg.set_x(this->currentVelocity.X());
    msg.set_y(this->currentVelocity.Y());
    msg.set_z(this->currentVelocity.Z());
    this->publisher.Publish(msg);
  }
}

