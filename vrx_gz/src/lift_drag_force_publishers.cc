#include <iostream>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

#include <gz/math/Vector3.hh>

#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>

#include "lift_drag_force_publishers.hh"

using namespace gz;
using namespace gz::sim;
// Don't use 'using namespace' for math or msgs to avoid conflicts

//////////////////////////////////////////////////
void LiftDragForcePublisher::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
{
  // Attach to model
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    std::cerr << "[LiftDragForcePublisher] Model is not valid\n";
    return;
  }

  // Read <link_name> from SDF (optional)
  if (_sdf && _sdf->HasElement("link_name"))
  {
    this->linkName = _sdf->Get<std::string>("link_name");
  }
  else
  {
    std::cout << "[LiftDragForcePublisher] <link_name> not specified, "
              << "using default: " << this->linkName << "\n";
  }

  // Read <topic> from SDF (optional)
  if (_sdf && _sdf->HasElement("topic"))
  {
    this->topic = _sdf->Get<std::string>("topic");
  }
  else
  {
    std::cout << "[LiftDragForcePublisher] <topic> not specified, "
              << "using default: " << this->topic << "\n";
  }

  // Find the link entity by name
  this->linkEntity = this->model.LinkByName(_ecm, this->linkName);
  if (this->linkEntity == kNullEntity)
  {
    std::cerr << "[LiftDragForcePublisher] Could not find link ["
              << this->linkName << "] in model ["
              << this->model.Name(_ecm) << "]\n";
    return;
  }
  else
  {
    std::cout << "[LiftDragForcePublisher] Found link ["
              << this->linkName << "], entity id = "
              << this->linkEntity << "\n";
  }

  // Enable velocity checks so we can read WorldLinearVelocity
  gz::sim::Link link(this->linkEntity);
  link.EnableVelocityChecks(_ecm);
  std::cout << "[LiftDragForcePublisher] Enabled velocity checks for link ["
            << this->linkName << "]\n";

  // Create publisher
  this->velocityPub = this->node.Advertise<gz::msgs::Vector3d>(this->topic);
  if (!this->velocityPub.Valid())
  {
    std::cerr << "[LiftDragForcePublisher] Failed to advertise velocity on ["
              << this->topic << "]\n";
  }
  else
  {
    std::cout << "[LiftDragForcePublisher] Publishing link velocity on ["
              << this->topic << "]\n";
  }
}

//////////////////////////////////////////////////
void LiftDragForcePublisher::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
if (_info.paused)
  return;

  if (this->linkEntity == gz::sim::kNullEntity)
    return;

  // Use Link API to get world linear velocity (handles component access automatically)
  gz::sim::Link link(this->linkEntity);
auto worldVelOpt = link.WorldLinearVelocity(_ecm);

if (!worldVelOpt)
{
  static bool warned = false;
  if (!warned)
  {
    std::cout << "[LiftDragForcePublisher] WorldLinearVelocity not available yet "
              << "for link [" << this->linkName << "]. Waiting...\n";
    std::cout.flush();
    warned = true;
  }
  // Don't publish until component is available
  return;
}

// Get velocity (in world frame) - use math::Vector3d for velocity data
gz::math::Vector3d vel = *worldVelOpt;

// Build and publish velocity message - use msgs::Vector3d for message
gz::msgs::Vector3d msg;
gz::msgs::Set(&msg, vel);

if (this->velocityPub.Valid())
{
  bool ok = this->velocityPub.Publish(msg);

  static int count = 0;
  if (++count <= 10 || count % 1000 == 0)
  {
    std::cout << "[LiftDragForcePublisher] Publish #" << count
              << " vel = [" << vel.X() << ", " << vel.Y() << ", " << vel.Z()
              << "], ok=" << (ok ? "true" : "false") << "\n";
    std::cout.flush();
  }
}
else
{
  static bool warned = false;
  if (!warned)
  {
    std::cerr << "[LiftDragForcePublisher] ERROR: Publisher is not valid! Topic: ["
              << this->topic << "]\n";
    std::cerr.flush();
    warned = true;
  }
}
}

// Register plugin with Gazebo
GZ_ADD_PLUGIN(LiftDragForcePublisher,
              System,
              ISystemConfigure,
              ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LiftDragForcePublisher,
                    "lift_drag_force_publisher")
