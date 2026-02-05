#ifndef STRATEGY_NODES_H
#define STRATEGY_NODES_H

#include "behaviortree_cpp/bt_factory.h"
#include "brain.h"

// Include our new nodes
#include "../condition/check_ball_dist.h"
#include "../condition/is_lead.h"
#include "../action/action_wait.h"

void RegisterStrategyNodes(BT::BehaviorTreeFactory& factory, Brain* brain);

#endif
