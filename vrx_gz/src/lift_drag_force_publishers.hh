#pragma once

#include <string>
#include <memory>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

/// \brief Minimal debug version of LiftDragForcePublisher.
/// This version ONLY:
///  - finds a link by name
///  - reads its LinearVelocity
///  - publishes that velocity on the given topic using gz::msgs::Vector3d
/// It DOES NOT affect physics in any way.
class LiftDragForcePublisher
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Called once when the plugin is loaded.
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  /// \brief Called every simulation iteration before physics update.
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  /// \brief Model this system is attached to.
  gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Link entity we are monitoring (e.g. sail_shaft_link).
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};

  /// \brief Name of the link to monitor. Read from SDF <link_name>.
  std::string linkName{"sail_shaft_link"};



  /// \brief Topic to publish the velocity on. Read from SDF <topic>.
  std::string topic{"/sailboat/sail/lift_drag_force"};

  /// \brief Transport node for publishing messages.
  gz::transport::Node node;

  /// \brief Publisher for gz::msgs::Vector3d messages (velocity).
  gz::transport::Node::Publisher velocityPub;


  // ... existing members ...

  // ========== Lift-Drag Parameters ==========
  /// \brief Fluid density (air or water). Read from SDF <air_density>.
  double airDensity{0.0};  // Must be specified in SDF

  /// \brief Surface area. Read from SDF <area>.
  double area{0.0};  // Must be specified in SDF

  /// \brief Lift coefficient slope. Read from SDF <cla>.
  double cla{0.0};  // Must be specified in SDF

  /// \brief Drag coefficient slope. Read from SDF <cda>.
  double cda{0.0};  // Must be specified in SDF

  /// \brief Stall angle of attack. Read from SDF <alpha_stall>.
  double alphaStall{0.0};  // Must be specified in SDF

  /// \brief Post-stall lift coefficient. Read from SDF <cla_stall>.
  double claStall{0.0};  // Must be specified in SDF

  /// \brief Post-stall drag coefficient. Read from SDF <cda_stall>.
  double cdaStall{0.0};  // Must be specified in SDF

  /// \brief Forward direction vector in link frame. Read from SDF <forward>.
  gz::math::Vector3d forward{0, 0, 0};  // Must be specified in SDF

  /// \brief Upward direction vector in link frame. Read from SDF <upward>.
  gz::math::Vector3d upward{0, 0, 0};  // Must be specified in SDF

  /// \brief Center of pressure in link frame. Read from SDF <cp>.
  gz::math::Vector3d cp{0, 0, 0};  // Must be specified in SDF
};
