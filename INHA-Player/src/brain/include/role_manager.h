#ifndef ROLE_MANAGER_H
#define ROLE_MANAGER_H

#include "brain_tree.h"
#include "behaviortree_cpp/behavior_tree.h"

void RegisterRoleManager(BT::BehaviorTreeFactory &factory, Brain* brain);

#endif // ROLE_MANAGER_H
