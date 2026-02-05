#ifndef IS_LEAD_H
#define IS_LEAD_H

#include "behaviortree_cpp/behavior_tree.h"
#include "../../brain.h"

// Condition Node: IsLead
// Returns SUCCESS if this robot is the "Lead" (e.g. closest to ball)
class IsLead : public BT::ConditionNode {
public:
    IsLead(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<Brain> brain)
        : BT::ConditionNode(name, config), brain_(brain) {}

    static BT::PortsList providedPorts() {
        return {}; 
    }

    BT::NodeStatus tick() override {
        // Logic: Check if we are closest to ball among teammates
        // For now, let's assume if our role is 'striker', we are Lead.
        // Or check a blackboard variable "is_closest_to_ball"
        
        // Simple implementation: If role == striker (or dynamically assigned lead), return SUCCESS
        // But better: use new dynamic role logic later.
        
        // For Swarm MVP: Success if ball dist < 2.0 (Active Zone) ? 
        // No, let's stick to name.
        
        // Hack for now: Assume always true for testing, or check simplistic logic
        string role = brain_->blackboard->get<string>("player_role");
        if (role == "striker") return BT::NodeStatus::SUCCESS;
        
        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<Brain> brain_;
};

#endif
