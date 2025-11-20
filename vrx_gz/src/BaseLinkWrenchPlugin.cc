/*
 * Copyright (C) 2025
 * Licensed under the Apache License, Version 2.0
 */

 #include <memory>
 #include <iostream>
 
 #include <gz/sim/System.hh>
 #include <gz/sim/Model.hh>
 #include <gz/sim/Link.hh>
 #include <gz/sim/World.hh>
 #include <gz/sim/components/LinearAcceleration.hh>
 #include <gz/sim/components/AngularVelocity.hh>
 #include <gz/sim/components/AngularAcceleration.hh>
 #include <gz/sim/components/Inertial.hh>
 #include <gz/sim/components/ParentEntity.hh>
 
 #include <gz/math/Matrix3.hh>
 #include <gz/math/Vector3.hh>
 #include <gz/math/Pose3.hh>
 
 #include <gz/msgs/wrench.pb.h>
 #include <gz/transport/Node.hh>
 #include <gz/plugin/Register.hh>
 
 #include "BaseLinkWrenchPlugin.hh"
 
 using namespace gz;
 using namespace gz::sim;
 using namespace vrx;
 
 //////////////////////////////////////////////////
 void BaseLinkWrenchPlugin::Configure(
     const Entity &_entity,
     const std::shared_ptr<const sdf::Element> &_sdf,
     EntityComponentManager &_ecm,
     EventManager &)
 {
   // This system is attached at the model level
   this->model = Model(_entity);
   if (!this->model.Valid(_ecm))
   {
     std::cerr << "[BaseLinkWrenchPlugin] Model is not valid\n";
     return;
   }
 
   // Read parameters from SDF: link_name and topic names
   if (_sdf->HasElement("link_name"))
     this->linkName = _sdf->Get<std::string>("link_name");
   else
     this->linkName = "base_link";
 
   if (_sdf->HasElement("topic_gravity"))
     this->topicGravity = _sdf->Get<std::string>("topic_gravity");
   else
     this->topicGravity = "/sailboat/base_link/gravity_wrench";
 
   if (_sdf->HasElement("topic_non_gravity"))
     this->topicNonGravity = _sdf->Get<std::string>("topic_non_gravity");
   else
     this->topicNonGravity = "/sailboat/base_link/net_minus_gravity";
 
   // Get link entity
   auto linkEntity = this->model.LinkByName(_ecm, this->linkName);
   if (kNullEntity == linkEntity)
   {
     std::cerr << "[BaseLinkWrenchPlugin] Could not find link ["
               << this->linkName << "]\n";
     return;
   }
   this->linkEntity = linkEntity;
 
   // Find the world entity (parent of the model)
   auto parent = _ecm.Component<components::ParentEntity>(_entity);
   if (parent)
   {
     this->worldEntity = parent->Data();
   }
   else
   {
     std::cerr << "[BaseLinkWrenchPlugin] Could not find world entity\n";
   }
 
   // Publishers
   this->gravWrenchPub =
       this->node.Advertise<msgs::Wrench>(this->topicGravity);
   if (!this->gravWrenchPub)
   {
     std::cerr << "[BaseLinkWrenchPlugin] Failed to advertise gravity wrench on ["
               << this->topicGravity << "]\n";
   }
   else
   {
     std::cout << "[BaseLinkWrenchPlugin] Publishing GRAVITY wrench of link ["
               << this->linkName << "] on [" << this->topicGravity << "]\n";
   }
 
   this->nonGravWrenchPub =
       this->node.Advertise<msgs::Wrench>(this->topicNonGravity);
   if (!this->nonGravWrenchPub)
   {
     std::cerr << "[BaseLinkWrenchPlugin] Failed to advertise non-gravity wrench on ["
               << this->topicNonGravity << "]\n";
   }
   else
   {
     std::cout << "[BaseLinkWrenchPlugin] Publishing NET-MINUS-GRAVITY wrench of link ["
               << this->linkName << "] on [" << this->topicNonGravity << "]\n";
   }
 }
 
 //////////////////////////////////////////////////
 void BaseLinkWrenchPlugin::PostUpdate(
     const UpdateInfo &_info,
     const EntityComponentManager &_ecm)
 {
   if (_info.paused)
     return;
 
   if (this->linkEntity == kNullEntity)
     return;
 
   // Get link interface
   Link link(this->linkEntity);
 
   // Get components
   auto linAccComp =
       _ecm.Component<components::LinearAcceleration>(this->linkEntity);
   auto angVelComp =
       _ecm.Component<components::AngularVelocity>(this->linkEntity);
   auto angAccComp =
       _ecm.Component<components::AngularAcceleration>(this->linkEntity);
   auto inertialComp =
       _ecm.Component<components::Inertial>(this->linkEntity);
 
   if (!linAccComp || !angVelComp || !angAccComp || !inertialComp)
   {
     // Not all components available yet
     return;
   }
 
   // World pose of the link
   const auto worldPose = link.WorldPose(_ecm);
   if (!worldPose)
   {
     return;
   }
 
   // --- Data in world frame ---
   const auto &aWorld     = linAccComp->Data();
   const auto &wWorld     = angVelComp->Data();
   const auto &alphaWorld = angAccComp->Data();
   const auto &inertial   = inertialComp->Data();
 
   double mass = inertial.MassMatrix().Mass();
 
   // Rotation from world to body frame
   const auto &qWorldToBody = (*worldPose).Rot();
 
   // Transform linear acceleration to body frame: a_body = q^-1 * a_world
   math::Vector3d aBody = qWorldToBody.Inverse().RotateVector(aWorld);
 
   // Inertia in body frame
   const auto &Ibody = inertial.MassMatrix().Moi();
   math::Matrix3d I = Ibody;
 
   // Angular vel/accel in body frame
   math::Vector3d wBody     = qWorldToBody.Inverse().RotateVector(wWorld);
   math::Vector3d alphaBody = qWorldToBody.Inverse().RotateVector(alphaWorld);
 
   // --- TOTAL NET FORCE & TORQUE in BODY frame (from dynamics) ---
   math::Vector3d Fbody = mass * aBody;
   math::Vector3d Iw    = I * wBody;
   math::Vector3d tauBody = I * alphaBody + wBody.Cross(Iw);
 
   // --- GRAVITY / WEIGHT ---
 
   // Default gravity if we can't read from world
   math::Vector3d gWorld(0, 0, -9.81);
 
   if (this->worldEntity != kNullEntity)
   {
     World world(this->worldEntity);
     auto gOpt = world.Gravity(_ecm);
     if (gOpt)
       gWorld = *gOpt;
   }
 
   // Weight force in world and body frames
   math::Vector3d FgravWorld = mass * gWorld;
   math::Vector3d FgravBody =
       qWorldToBody.Inverse().RotateVector(FgravWorld);
 
   // Torque from gravity about link frame origin:
   // r_body = COM position relative to link frame (body frame)
   math::Vector3d rBody = inertial.Pose().Pos();  // COM offset in link frame
   math::Vector3d tauGravBody = rBody.Cross(FgravBody);
 
   // --- EVERYTHING-BUT-GRAVITY WRENCH ---
   math::Vector3d FnonGravBody  = Fbody - FgravBody;
   math::Vector3d taunonGravBody = tauBody - tauGravBody;
 
   // --- Publish GRAVITY wrench ---
   if (this->gravWrenchPub)
   {
     msgs::Wrench msg;
     msg.mutable_force()->set_x(FgravBody.X());
     msg.mutable_force()->set_y(FgravBody.Y());
     msg.mutable_force()->set_z(FgravBody.Z());
     msg.mutable_torque()->set_x(tauGravBody.X());
     msg.mutable_torque()->set_y(tauGravBody.Y());
     msg.mutable_torque()->set_z(tauGravBody.Z());
     this->gravWrenchPub.Publish(msg);
   }
 
   // --- Publish NET-MINUS-GRAVITY wrench ---
   if (this->nonGravWrenchPub)
   {
     msgs::Wrench msg;
     msg.mutable_force()->set_x(FnonGravBody.X());
     msg.mutable_force()->set_y(FnonGravBody.Y());
     msg.mutable_force()->set_z(FnonGravBody.Z());
     msg.mutable_torque()->set_x(taunonGravBody.X());
     msg.mutable_torque()->set_y(taunonGravBody.Y());
     msg.mutable_torque()->set_z(taunonGravBody.Z());
     this->nonGravWrenchPub.Publish(msg);
   }
 }
 
 // Register plugin
 GZ_ADD_PLUGIN(
   BaseLinkWrenchPlugin,
   System,
   ISystemConfigure,
   ISystemPostUpdate)
 
 GZ_ADD_PLUGIN_ALIAS(BaseLinkWrenchPlugin,
                     "vrx::BaseLinkWrenchPlugin")
 