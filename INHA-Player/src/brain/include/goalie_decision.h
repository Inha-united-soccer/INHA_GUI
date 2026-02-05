#ifndef GOALIE_DECISION_H 
#define GOALIE_DECISION_H

#include "brain_tree.h"
#include "behaviortree_cpp/behavior_tree.h"

using namespace BT;

// From decision_role.cpp
class GoalieDecide : public BrainTreeNode {
public:
    GoalieDecide(const std::string& name, const NodeConfig& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("chase_threshold"),
            InputPort<string>("decision_in"),
            InputPort<double>("ctPosx"),
            InputPort<double>("ctPosy"),
            InputPort<double>("closer_margin"),
            InputPort<double>("clearing_max"),
            OutputPort<string>("decision_out")
        };
    }

    NodeStatus tick() override;
};

class GoalieClearingDecide : public BrainTreeNode {
public:
    GoalieClearingDecide(const std::string& name, const NodeConfig& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("chase_threshold"),
            InputPort<string>("clearing_decision_in"),
            InputPort<double>("ctPosx"),
            InputPort<double>("ctPosy"),
            InputPort<string>("decision_in"), 
            OutputPort<string>("clearing_decision_out"),
            OutputPort<string>("decision_out")
        };
    }

    NodeStatus tick() override;
};

// From clearing.cpp
class CalcClearingDir : public BrainTreeNode {
public:
    CalcClearingDir(const std::string& name, const NodeConfig& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("goalx"),
            InputPort<double>("goaly")
        };
    }

    NodeStatus tick() override;
};

class ClearingChase : public BrainTreeNode {
public:
    ClearingChase(const std::string& name, const NodeConfig& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("vx_limit"),
            InputPort<double>("vy_limit"),
            InputPort<double>("vtheta_limit"),
            InputPort<double>("dist"),
            InputPort<double>("safeDist"),
            InputPort<double>("p_gain")
        };
    }

    NodeStatus tick() override;
};

// From hold.cpp
class PredictBallTraj : public BrainTreeNode {
public:
    PredictBallTraj(const std::string& name, const NodeConfig& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("R_meas"),
            InputPort<double>("sigma_a"),
            InputPort<double>("P0_pos"),
            InputPort<double>("P0_vel"),
            InputPort<double>("drop_time"),
            InputPort<double>("vel_decay"),
            InputPort<double>("a_min"),
            InputPort<double>("a_max"),
            InputPort<double>("k_av"),
            InputPort<double>("horizon"),
            InputPort<double>("ctPosx"),
            InputPort<double>("ctPosy")
        };
    }

    NodeStatus tick() override;

private:
    // KF state
    double x_ = 0.0, y_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0;
    double P_[4][4];

    rclcpp::Time prev_time_;
    bool has_prev_time_ = false;

    rclcpp::Time last_meas_stamp_;
    bool has_last_meas_ = false;
    bool kf_initialized_ = false;
};

class CalcGoliePos : public BrainTreeNode {
public:
    CalcGoliePos(const std::string& name, const NodeConfig& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("golie_radius"),
            InputPort<double>("ctPosx"),
            InputPort<double>("ctPosy")
        };
    }

    NodeStatus tick() override;
};

class GolieMove : public BrainTreeNode {
public:
    GolieMove(const std::string& name, const NodeConfig& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("stop_threshold"),
            InputPort<double>("ctPosx"),
            InputPort<double>("ctPosy"),
            InputPort<double>("Kp_theta"),
            InputPort<double>("Kp"),
            InputPort<double>("vx_high"),
            InputPort<double>("vx_low"),
            InputPort<double>("vy_high"),
            InputPort<double>("vy_low")
        };
    }

    NodeStatus tick() override;
};

class GolieInitPos : public BrainTreeNode {
public:
    GolieInitPos(const std::string& name, const NodeConfig& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("turn_threshold"),
            InputPort<double>("stop_threshold"),
            InputPort<double>("vx_limit"),
            InputPort<double>("vy_limit"),
            InputPort<double>("init_golie_pos_x"),
            InputPort<double>("init_golie_pos_y"),
            InputPort<double>("init_golie_pos_theta")
        };
    }

    NodeStatus tick() override;
};


void RegisterGoalieDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

#endif // GOALIE_DECISION_H
