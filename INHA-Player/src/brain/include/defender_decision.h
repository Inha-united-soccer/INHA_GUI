#ifndef DEFENDER_DECISION_H
#define DEFENDER_DECISION_H

#include "brain_tree.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

using namespace BT;

class DefenderDecision : public BrainTreeNode
{
public: 
    DefenderDecision(const string &name, const NodeConfiguration &config, Brain *brain) : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold"),
            InputPort<double>("clearing_threshold"),
            InputPort<string>("decision_in"),
            OutputPort<string>("decision_out")
        };
    }

    NodeStatus tick() override;
    
    rclcpp::Time timeLastTick;
    double lastDeltaDir = 0.0;
};

class DefenderClearingDecide : public BrainTreeNode {
public:
    DefenderClearingDecide(const std::string& name, const NodeConfiguration &config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("chase_threshold"),
            InputPort<string>("clearing_decision_in"),
            InputPort<string>("decision_in"), 
            OutputPort<string>("clearing_decision_out")
        };
    }

    NodeStatus tick() override;
    
    rclcpp::Time timeLastTick;
    double lastDeltaDir = 0;
};

// Renamed from CalcClearingDir (Verifier conflict with Goalie)
class CalcDefenderClearingDir : public BrainTreeNode {
public:
    CalcDefenderClearingDir(const std::string& name, const NodeConfiguration &config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("offset_degree")
        };
    }

    NodeStatus tick() override;
};



void RegisterDefenderDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

#endif