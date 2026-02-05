#ifndef STRATEGY_NODES_H
#define STRATEGY_NODES_H

#include "behaviortree_cpp/bt_factory.h"
#include "brain.h"

// Include our new nodes
#include "../condition/check_ball_dist.h"
#include "../condition/is_lead.h"
#include "../action/action_wait.h"

inline void RegisterStrategyNodes(BT::BehaviorTreeFactory& factory, Brain* brain) {
    factory.registerBuilder<CheckBallDist>("CheckBallDist", 
        [brain](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<CheckBallDist>(name, config, brain);
        });

    factory.registerBuilder<IsLead>("IsLead", 
        [brain](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<IsLead>(name, config, brain);
        });

    factory.registerBuilder<ActionWait>("Wait", 
        [brain](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<ActionWait>(name, config, brain);
        });
}

#endif
