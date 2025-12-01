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
  
    // Aerodynamic coefficients
    if (_sdf->HasElement("cla"))
    {
      this->cla = _sdf->Get<double>("cla");
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <cla> not specified in SDF!\n";
    }
  
    if (_sdf->HasElement("cda"))
    {
      this->cda = _sdf->Get<double>("cda");
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <cda> not specified in SDF!\n";
    }
  
    if (_sdf->HasElement("alpha_stall"))
    {
      this->alphaStall = _sdf->Get<double>("alpha_stall");
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <alpha_stall> not specified in SDF!\n";
    }
  
    if (_sdf->HasElement("cla_stall"))
    {
      this->claStall = _sdf->Get<double>("cla_stall");
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <cla_stall> not specified in SDF!\n";
    }
  
    if (_sdf->HasElement("cda_stall"))
    {
      this->cdaStall = _sdf->Get<double>("cda_stall");
    }
    else
    {
      std::cerr << "[LiftDragForcePublisher] ERROR: <cda_stall> not specified in SDF!\n";
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
double alphaRad = atan2(flowDir.Dot(upwardWorld), flowDir.Dot(forwardWorld));

// Convert to degrees
double alpha = alphaRad * 180.0 / M_PI;

// Lift direction is perpendicular to flow, in the chord-normal plane
gz::math::Vector3d liftDir;
if (std::abs(alphaRad) > 1e-6)  // Avoid singularity at alpha=0
{
  liftDir = flowDir.Cross(spanwiseWorld);
  liftDir.Normalize();
}
else
{
  // At alpha≈0, no lift or use upward direction
  liftDir = upwardWorld;
}

    

double cl, cd;

if (std::abs(alpha*180.0/M_PI) < this->alphaStall)
{
  // Pre-stall: linear region
  cl = this->cla * alpha;
  cd = this->cda + this->cdaStall * alpha * alpha;
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
const double MAX_VELOCITY = 10.0;  // m/s
const double q_max = 0.5 * this->airDensity * MAX_VELOCITY * MAX_VELOCITY;
const double MAX_FORCE = q_max * this->area * (std::abs(this->cla) + this->cda);  

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


gz::math::Vector3d cpWorld = linkRot.RotateVector(this->cp);

gz::sim::Link linkAPI(this->linkEntity);
linkAPI.AddWorldForce(_ecm, totalForce, cpWorld);


gz::msgs::Vector3d Fmsg;
// totalForce.Set(1, 0, 0);  // Sets X, Y, Z to 0, 0, 0
gz::msgs::Set(&Fmsg, totalForce);

if (this->velocityPub.Valid())
{
  this->velocityPub.Publish(Fmsg);
  
  // Periodic detailed logging (every 100 iterations)
  static int counter = 0;
  if (counter % 100 == 0)
  {
    std::cout << "[LiftDragForcePublisher] "
              << "vel=" << velMag << " m/s, "
              << "alpha=" << alpha << "°, "
              << "cl=" << cl << ", cd=" << cd << ", "
              << "lift=" << liftMag << " N, drag=" << dragMag << " N, "
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
