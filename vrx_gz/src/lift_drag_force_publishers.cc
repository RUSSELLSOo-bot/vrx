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
  
    // ========== Read Lift-Drag Parameters from SDF ==========
    // All parameters must be specified in SDF - no hardcoded defaults
    
    // Fluid properties
    
    if (_sdf && _sdf->HasElement("air_density"))
    {
      this->airDensity = _sdf->Get<double>("air_density");
      std::cout << "[LiftDragForcePublisher] air_density = " << this->airDensity << " kg/m³\n";
    }
    else  
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <air_density> not specified in SDF!\n";
    }

    // Geometry
    if (_sdf->HasElement("area"))
    {
      this->area = _sdf->Get<double>("area");
      std::cout << "[LiftDragForcePublisher] area = " << this->area << " m²\n";
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <area> not specified in SDF!\n";
    }
  
    // Normalization parameters
    if (_sdf->HasElement("cl_max"))
    {
      this->cl_max = _sdf->Get<double>("cl_max");
      std::cout << "[LiftDragForcePublisher] cl_max = " << this->cl_max << "\n";
    }
    
    if (_sdf->HasElement("cd_max"))
    {
      this->cd_max = _sdf->Get<double>("cd_max");
      std::cout << "[LiftDragForcePublisher] cd_max = " << this->cd_max << "\n";
    }

    // Normalized linear regression coefficients for Cl
    if (_sdf->HasElement("cla_norm"))
    {
      this->cla_norm = _sdf->Get<double>("cla_norm");
      std::cout << "[LiftDragForcePublisher] cla_norm = " << this->cla_norm << "\n";
    }

    if (_sdf->HasElement("cl0_norm"))
    {
      this->cl0_norm = _sdf->Get<double>("cl0_norm");
      std::cout << "[LiftDragForcePublisher] cl0_norm = " << this->cl0_norm << "\n";
    }

    // Normalized quartic regression coefficients for Cd
    if (_sdf->HasElement("cd_a_norm"))
    {
      this->cd_a_norm = _sdf->Get<double>("cd_a_norm");
      std::cout << "[LiftDragForcePublisher] cd_a_norm (quartic) = " << this->cd_a_norm << "\n";
    }

    if (_sdf->HasElement("cd_b_norm"))
    {
      this->cd_b_norm = _sdf->Get<double>("cd_b_norm");
      std::cout << "[LiftDragForcePublisher] cd_b_norm (cubic) = " << this->cd_b_norm << "\n";
    }

    if (_sdf->HasElement("cd_c_norm"))
    {
      this->cd_c_norm = _sdf->Get<double>("cd_c_norm");
      std::cout << "[LiftDragForcePublisher] cd_c_norm (quadratic) = " << this->cd_c_norm << "\n";
    }

    if (_sdf->HasElement("cd_d_norm"))
    {
      this->cd_d_norm = _sdf->Get<double>("cd_d_norm");
      std::cout << "[LiftDragForcePublisher] cd_d_norm (linear) = " << this->cd_d_norm << "\n";
    }

    if (_sdf->HasElement("cd_e_norm"))
    {
      this->cd_e_norm = _sdf->Get<double>("cd_e_norm");
      std::cout << "[LiftDragForcePublisher] cd_e_norm (constant) = " << this->cd_e_norm << "\n";
    }

    // Stall parameters
    if (_sdf->HasElement("alpha_stall"))
    {
      this->alphaStall = _sdf->Get<double>("alpha_stall");
      std::cout << "[LiftDragForcePublisher] alpha_stall = " << this->alphaStall << "°\n";
    }

    if (_sdf->HasElement("cla_stall"))
    {
      this->claStall = _sdf->Get<double>("cla_stall");
      std::cout << "[LiftDragForcePublisher] cla_stall = " << this->claStall << "\n";
    }

    if (_sdf->HasElement("cda_stall"))
    {
      this->cdaStall = _sdf->Get<double>("cda_stall");
      std::cout << "[LiftDragForcePublisher] cda_stall = " << this->cdaStall << "\n";
    }
  
    // Direction vectors
    if (_sdf->HasElement("forward"))
    {
      this->forward = _sdf->Get<gz::math::Vector3d>("forward");
      this->forward.Normalize();
      std::cout << "[LiftDragForcePublisher] forward = [" 
                << this->forward.X() << ", " << this->forward.Y() << ", " 
                << this->forward.Z() << "]\n";
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <forward> not specified in SDF!\n";
    }
  
    if (_sdf->HasElement("upward"))
    {
      this->upward = _sdf->Get<gz::math::Vector3d>("upward");
      this->upward.Normalize();
      std::cout << "[LiftDragForcePublisher] upward = [" 
                << this->upward.X() << ", " << this->upward.Y() << ", " 
                << this->upward.Z() << "]\n";
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <upward> not specified in SDF!\n";
    }
  
    if (_sdf->HasElement("cp"))
    {
      this->cp = _sdf->Get<gz::math::Vector3d>("cp");
      std::cout << "[LiftDragForcePublisher] cp = [" 
                << this->cp.X() << ", " << this->cp.Y() << ", " 
                << this->cp.Z() << "]\n";
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <cp> not specified in SDF!\n";
    }
  
    // Find the link entity by name
    this->linkEntity = this->model.LinkByName(_ecm, this->linkName);
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
gz::math::Vector3d velW = *worldVelOpt;

// Build and publish velocity message - use msgs::Vector3d for message
gz::msgs::Vector3d msg;
gz::msgs::Set(&msg, velW);

// Get link pose to transform directions from link frame to world frame
// auto poseComp = _ecm.Component<gz::sim::components::WorldPose>(this->linkEntity);
auto poseComp = link.WorldPose(_ecm);

if (!poseComp)
{
  static bool warned = false;
  if (!warned)
  {
    std::cerr << "[LiftDragForcePublisher] ERROR: WorldPose component not available!\n";
    warned = true;
  }
  return;
}

// gz::math::Pose3d linkPose = poseComp->Data();
gz::math::Pose3d linkPose = *poseComp;
gz::math::Quaterniond linkRot = linkPose.Rot();

// Transform forward and upward directions from link frame to world frame
gz::math::Vector3d forwardWorld = linkRot.RotateVector(this->forward);
gz::math::Vector3d upwardWorld = linkRot.RotateVector(this->upward);

// Ensure they're normalized
forwardWorld.Normalize();
upwardWorld.Normalize();

// Calculate the spanwise direction (perpendicular to both forward and upward)
gz::math::Vector3d spanwiseWorld = forwardWorld.Cross(upwardWorld);
spanwiseWorld.Normalize();

double velSpanwise = velW.Dot(spanwiseWorld);
gz::math::Vector3d velInPlane = velW - velSpanwise * spanwiseWorld;

double velMag = velInPlane.Length();
// If velocity is too small, no aerodynamic forces
if (velMag < 1e-6)
{
  return;
}

//normalized flow direction 
// Normalized flow direction
gz::math::Vector3d flowDir = velInPlane / velMag;

// Calculate signed angle of attack using atan2 (returns radians)
// Positive alpha: flow from below; Negative alpha: flow from above
double alphaRad = atan2(flowDir.Dot(upwardWorld), flowDir.Dot(forwardWorld));

// Convert to degrees
double alpha = alphaRad * 180.0 / M_PI;



// Lift direction is perpendicular to flow, in the chord-normal plane
// Cross product naturally gives correct sign based on flow direction
gz::math::Vector3d liftDir = flowDir.Cross(spanwiseWorld);
liftDir.Normalize();

    

// ========== Aerodynamic Coefficient Calculation ==========
double cl, cd;

if (std::abs(alpha) < this->alphaStall)
{
  // Pre-stall: use linear lift model (guarantees cl=0 when alpha=0)
  // Calculate effective lift slope from normalized parameters
  double cla = this->cla_norm * this->cl_max / this->alphaStall;
  cl = cla * alpha;  // Simple linear: zero at alpha=0
  
  // Drag: use quartic regression (normalized, using Horner's method)
  // Normalize alpha to [-1, 1] range for drag calculation
  double alpha_norm = alpha / this->alphaStall;
  
  // cd_norm = ((((cd_a*α + cd_b)*α + cd_c)*α + cd_d)*α + cd_e)
  double cd_norm = this->cd_a_norm;
  cd_norm = cd_norm * alpha_norm + this->cd_b_norm;
  cd_norm = cd_norm * alpha_norm + this->cd_c_norm;
  cd_norm = cd_norm * alpha_norm + this->cd_d_norm;
  cd_norm = cd_norm * alpha_norm + this->cd_e_norm;
  
  // Denormalize to get actual Cd
  cd = cd_norm * this->cd_max;
  
  // Clamp to physically reasonable bounds (with 20% margin for safety)
  cl = std::clamp(cl, -this->cl_max * 1.2, this->cl_max * 1.2);
  cd = std::clamp(cd, 0.0, this->cd_max * 2.0);
}
else
{
  // Post-stall: reduced lift, increased drag
  double sign = (alpha >= 0) ? 1.0 : -1.0;
  cl = this->claStall * sign;
  cd = this->cdaStall;
}

// ========== Force Calculation ==========
// Dynamic pressure: q = 0.5 * rho * v^2
double q = 0.5 * this->airDensity * velMag * velMag;

// Lift force magnitude: F_L = q * S * C_L
double liftMag = q * this->area * cl;

// Drag force magnitude: F_D = q * S * C_D
double dragMag = q * this->area * cd;

// Limit forces to prevent physics explosions
const double MAX_VELOCITY = 5.0;  // m/s
const double q_max = 0.5 * this->airDensity * MAX_VELOCITY * MAX_VELOCITY;

// Calculate max force using maximum coefficients from normalization
// Maximum total coefficient is the sum of max Cl and max Cd
const double MAX_FORCE = q_max * this->area * (this->cl_max + this->cd_max);

liftMag = std::clamp(liftMag, -MAX_FORCE, MAX_FORCE);
dragMag = std::clamp(dragMag, 0.0, MAX_FORCE);

gz::math::Vector3d dragForce = -dragMag * flowDir;
gz::math::Vector3d liftForce = liftMag * liftDir;

// Total aerodynamic force
gz::math::Vector3d totalForce = liftForce + dragForce;

// Final NaN safety check before applying force
if (!std::isfinite(totalForce.X()) || !std::isfinite(totalForce.Y()) || !std::isfinite(totalForce.Z()))
{
  std::cerr << "[LiftDragForcePublisher] WARNING: NaN detected in force, skipping this update\n";
  return;  // Don't apply invalid forces
}

// Apply force at center of pressure
// Note: AddWorldForce expects offset in link frame, not world frame
gz::sim::Link linkAPI(this->linkEntity);
linkAPI.AddWorldForce(_ecm, totalForce, this->cp);


gz::msgs::Vector3d Fmsg;
// totalForce.Set(1, 0, 0);  // Sets X, Y, Z to 0, 0, 0
gz::msgs::Set(&Fmsg, totalForce);

if (this->velocityPub.Valid())
{
  this->velocityPub.Publish(Fmsg);
  
  // Periodic detailed logging (every 100 iterations)
  static int counter = 0;
  if (counter % 10 == 0)
  {
    std::cout << "[LiftDragForcePublisher] "
              << "vel=" << velMag << " m/s, "
              << "alpha=" << alpha << "°, "
              << "total_force=[" << totalForce.X() << ", " 
              << totalForce.Y() << ", " << totalForce.Z() << "] N\n";
  }
  counter++;
}
else
{
  static bool warned = false;
  if (!warned)
  {
    std::cerr << "[LiftDragForcePublisher] ERROR: Publisher is not valid! Topic: ["
              << this->topic << "]\n";
    warned = true;
  }
}
}




// if (this->velocityPub.Valid())
// {
//   bool ok = this->velocityPub.Publish(msg);
// }
// else
// {
//   static bool warned = false;
//   if (!warned)
//   {
//     std::cerr << "[LiftDragForcePublisher] ERROR: Publisher is not valid! Topic: ["
//               << this->topic << "]\n";
//     std::cerr.flush();
//     warned = true;
//   }
// }
// }

// Register plugin with Gazebo
GZ_ADD_PLUGIN(LiftDragForcePublisher,
              System,
              ISystemConfigure,
              ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LiftDragForcePublisher,
                    "lift_drag_force_publisher")
