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

  // Normalization parameters
  /// \brief Maximum Cl for normalization. Read from SDF <cl_max>.
  double cl_max{1.0};

  /// \brief Maximum Cd for normalization. Read from SDF <cd_max>.
  double cd_max{1.0};

  // Normalized linear regression for Cl: Cl = (cla_norm * alpha_norm + cl0_norm) * cl_max
  /// \brief Normalized lift curve slope. Read from SDF <cla_norm>.
  double cla_norm{0.0};

  /// \brief Normalized lift at alpha=0. Read from SDF <cl0_norm>.
  double cl0_norm{0.0};

  // Normalized quartic regression for Cd: Cd = (cd_a_norm*α⁴ + cd_b_norm*α³ + cd_c_norm*α² + cd_d_norm*α + cd_e_norm) * cd_max
  /// \brief Normalized quartic term for drag. Read from SDF <cd_a_norm>.
  double cd_a_norm{0.0};

  /// \brief Normalized cubic term for drag. Read from SDF <cd_b_norm>.
  double cd_b_norm{0.0};

  /// \brief Normalized quadratic term for drag. Read from SDF <cd_c_norm>.
  double cd_c_norm{0.0};

  /// \brief Normalized linear term for drag. Read from SDF <cd_d_norm>.
  double cd_d_norm{0.0};

  /// \brief Normalized constant term for drag. Read from SDF <cd_e_norm>.
  double cd_e_norm{0.0};

  /// \brief Stall angle of attack (degrees, also used for alpha normalization). Read from SDF <alpha_stall>.
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
