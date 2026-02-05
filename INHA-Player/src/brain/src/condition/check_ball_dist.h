#ifndef CHECK_BALL_DIST_H
#define CHECK_BALL_DIST_H

#include "behaviortree_cpp/behavior_tree.h"
#include "../../brain.h"

// Condition Node: CheckBallDist
// Returns SUCCESS if ball distance satisfies the condition (LT < dist or GT > dist)
class CheckBallDist : public BT::ConditionNode {
public:
    CheckBallDist(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<Brain> brain)
        : BT::ConditionNode(name, config), brain_(brain) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("op"),   // "LT" (Less Than) or "GT" (Greater Than)
            BT::InputPort<double>("dist")       // Distance value 
        };
    }

    BT::NodeStatus tick() override {
        // 1. Get arguments
        std::string op;
        double threshold_dist;
        if (!getInput("op", op)) {
            throw BT::RuntimeError("missing required input [op]");
        }
        if (!getInput("dist", threshold_dist)) {
            throw BT::RuntimeError("missing required input [dist]");
        }

        // 2. Get current ball distance from Blackboard or Brain
        // Using Blackboard entry "ball_range" which is updated in brain_tree.cpp
        // But better to access brain_->ball_range directly if available, or blackboard
        auto ball_range = brain_->blackboard->get<double>("ball_range");
        
        // 3. Compare
        bool result = false;
        if (op == "LT") {
            result = (ball_range < threshold_dist);
        } else if (op == "GT") {
            result = (ball_range > threshold_dist);
        }

        // 4. Return
        return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<Brain> brain_;
};

#endif
