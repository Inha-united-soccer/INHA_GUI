#ifndef ACTION_WAIT_H
#define ACTION_WAIT_H

#include "behaviortree_cpp/behavior_tree.h"
#include "brain.h"
#include <thread>
#include <chrono>

class ActionWait : public BT::SyncActionNode {
public:
    ActionWait(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<Brain> brain)
        : BT::SyncActionNode(name, config), brain_(brain) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("seconds") };
    }

    BT::NodeStatus tick() override {
        double seconds;
        if (!getInput("seconds", seconds)) {
            seconds = 1.0; // default
        }
        
        // Blocking wait (simplest for now, though async is better for responsiveness)
        // For SyncActionNode, sleeping blocks the whole tree tick. 
        // Ideally should be Coroutine or Async, but let's stick to Sync for short waits.
        std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<Brain> brain_;
};

#endif
