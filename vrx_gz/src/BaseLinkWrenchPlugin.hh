#pragma once

#include <memory>
#include <string>

#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>

namespace vrx
{
  class BaseLinkWrenchPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    public:
      void Configure(const gz::sim::Entity &_entity,
                     const std::shared_ptr<const sdf::Element> &_sdf,
                     gz::sim::EntityComponentManager &_ecm,
                     gz::sim::EventManager &_eventMgr) override;

      void PostUpdate(const gz::sim::UpdateInfo &_info,
                      const gz::sim::EntityComponentManager &_ecm) override;

    private:
      // Model + entities
      gz::sim::Model model{gz::sim::kNullEntity};
      gz::sim::Entity linkEntity{gz::sim::kNullEntity};
      gz::sim::Entity worldEntity{gz::sim::kNullEntity};

      // Config
      std::string linkName{"base_link"};
      std::string topicGravity{"/sailboat/base_link/gravity_wrench"};
      std::string topicNonGravity{"/sailboat/base_link/net_minus_gravity"};

      // Transport
      gz::transport::Node node;
      gz::transport::Node::Publisher gravWrenchPub;    // gravity-only
      gz::transport::Node::Publisher nonGravWrenchPub; // total - gravity
  };
}
