#pragma once

#include <tuple>
#include <mutex>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

// Base class for Brain nodes
class BrainTreeNode : public BT::SyncActionNode
{
public:
    BrainTreeNode(const std::string& name, const BT::NodeConfiguration& config, Brain* brain)
        : BT::SyncActionNode(name, config), brain(brain) {}

protected:
    Brain* brain;
};

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();
    void tick();
    void reloadTree(std::string path);

    // 블랙보드 변수 접근 함수
    template <typename T>
    inline T getEntry(const string &key){
        T value;
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // 블랙보드 변수 셋팅 함수
    template <typename T>
    inline void setEntry(const string &key, const T &value){
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;

    void initEntry(); // 블랙보드 초기화는 여기서 진행
    
    std::mutex treeMutex; // Thread protection for strategy reloading
};
