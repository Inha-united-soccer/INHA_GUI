#ifndef TACTICS_NODES_H
#define TACTICS_NODES_H

#include "behaviortree_cpp/bt_factory.h"
#include "brain.h"

void RegisterTacticsNodes(BT::BehaviorTreeFactory& factory, Brain* brain);

#endif
