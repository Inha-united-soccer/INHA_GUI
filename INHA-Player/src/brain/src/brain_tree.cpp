#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>

#include "brain_tree.h"
#include "locator.h"
#include "brain.h"

#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <ios>
#include "gotopose.h"

// Strategy
#include "strategy/strategy_nodes.h"
// Tactics
#include "tactics/tactics_nodes.h"

#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

#include <mutex>

void BrainTree::init(){
    BehaviorTreeFactory factory;

    brain->registerWalkNodes(factory); // walk 관련 노드 등록
    brain->registerMoveHeadNodes(factory); // head move 관련 노드 등록
    brain->registerLocatorNodes(factory); // locator 관련 노드 등록
    brain->registerChaseNodes(factory); // chase 관련 노드 등록
    RegisterKickNodes(factory, brain); // kick 관련 노드 등록
    brain->registerAdjustNodes(factory); // adjust 관련 노드 등록
    brain->registerSpeakNodes(factory); // speak 관련 노드 등록
    brain->registerOfftheballNodes(factory); // offtheball 관련 노드 등록
    brain->registerDefenderDecisionNodes(factory); // defender decision 관련 노드 등록
    brain->registerStrikerDecisionNodes(factory); // striker decision 관련 노드 등록
    brain->registerGoalieDecisionNodes(factory); // goalie decision 관련 노드 등록
    brain->registerRoleManager(factory); // role manager 관련 노드 등록
    RegisterGotoposeNodes(factory, brain); // gotopose node registration
    brain->registerPassReceiveNodes(factory); // pass receive 노드 등록
    brain->registerPassNodes(factory); // pass direction node 등록
    
    // NEW: Register Strategy & Tactics Nodes
    RegisterStrategyNodes(factory, brain);
    RegisterTacticsNodes(factory, brain);
    
    
    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    
    // Smart Tree Selection Logic (Same as reloadTree)
    auto registered_trees = factory.registeredBehaviorTrees();
    std::string target_tree_id = "MainTree";
    bool found = false;
    
    // 1. Check for standard IDs
    std::vector<std::string> priorities = {"MainTree", "Striker", "Defender", "Goalkeeper", "Goalie", "StrategyTree"};
    for (const auto& p : priorities) {
        for (const auto& t : registered_trees) {
            if (t == p) {
                target_tree_id = p;
                found = true;
                break;
            }
        }
        if (found) break;
    }

    // 2. Fallback
    if (!found && !registered_trees.empty()) {
        target_tree_id = registered_trees[0];
    }
    
    if (registered_trees.empty()) {
        std::cerr << "[BrainTree] ERROR: No behavior trees found in " << brain->config->treeFilePath << std::endl;
        // Fallback to avoid immediate crash if possible, or let it crash on createTree
    }

    tree = factory.createTree(target_tree_id);

    // 여기서 블랙보드가 초기화됨
    initEntry();
}

void BrainTree::initEntry(){
    // 여기서 블랙보드를 초기화하면 됨 
    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", ""); // 현재 ready,set,play,end 중 하나일 것 
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false); // 우리 팀 킥오프인지
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);
    setEntry<int>("control_state", 3); // control_state == 1 이면 단순 걷기로 (1->3 play.xml test)
    
    setEntry<string>("decision", ""); // pass / chase / adjust / kick / assist 더 추가 예정
    setEntry<bool>("wait_for_opponent_kickoff", false);
    setEntry<string>("player_role", brain->config->playerRole); // play.xml (striker / defender / goal_keeper)
    brain->log->logToScreen("debug/Blackboard", "BB Role: " + brain->config->playerRole, 0x00FF00FF);


    // 실제 경기 중 상황 
    // 공 
    setEntry<bool>("ball_out", false); // 공이 필드 밖으로 나갔는지  
    setEntry<bool>("ball_location_known", false); // 공 위치를 알고 있는지
    setEntry<bool>("tm_ball_pos_reliable", false); // 팀원 공 위치 정보가 신뢰할 만한지 확인
    setEntry<double>("ball_range", 0); // 공과의 거리

    //  승재욱 추가 : chase -> adjust -> kick
    setEntry<string>("striker_state", "chase");   

    // 위치 추정 보정 여부
    setEntry<bool>("odom_calibrated", false);
}

void BrainTree::tick(){ 
    std::lock_guard<std::recursive_mutex> lock(treeMutex);
    tree.tickOnce(); 
}

void BrainTree::reloadTree(std::string path) {
    std::cout << "[BrainTree] Reloading Tree from: " << path << std::endl;
    tree.haltTree();
    
    BehaviorTreeFactory factory;

    brain->registerWalkNodes(factory);
    brain->registerMoveHeadNodes(factory);
    brain->registerLocatorNodes(factory);
    brain->registerChaseNodes(factory);
    RegisterKickNodes(factory, brain);
    brain->registerAdjustNodes(factory);
    brain->registerSpeakNodes(factory);
    brain->registerOfftheballNodes(factory);
    brain->registerDefenderDecisionNodes(factory);
    brain->registerStrikerDecisionNodes(factory);
    brain->registerGoalieDecisionNodes(factory);
    brain->registerRoleManager(factory);
    RegisterGotoposeNodes(factory, brain);
    brain->registerPassReceiveNodes(factory);
    brain->registerPassNodes(factory);
    
    RegisterStrategyNodes(factory, brain);
    RegisterTacticsNodes(factory, brain);
    
    try {
        factory.registerBehaviorTreeFromFile(path);
        
        // Smart Tree Selection Logic
        auto registered_trees = factory.registeredBehaviorTrees();
        std::string target_tree_id = "MainTree"; // Default
        bool found = false;

        if (registered_trees.empty()) {
            throw std::runtime_error("No behavior trees found in XML file");
        }

        // 1. Check for specific Role-based IDs
        std::vector<std::string> priorities = {"MainTree", "Striker", "Defender", "Goalkeeper", "Goalie", "StrategyTree"};
        for (const auto& p : priorities) {
            for (const auto& t : registered_trees) {
                if (t == p) {
                    target_tree_id = p;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }

        // 2. Fallback: Use the first one found
        if (!found) {
            target_tree_id = registered_trees[0];
            std::cout << "[BrainTree] Warning: No standard tree ID found. Using: " << target_tree_id << std::endl;
        }

        std::cout << "[BrainTree] Creating tree with ID: " << target_tree_id << std::endl;
        
        // CRITICAL: Protect tree swap with Mutex
        {
            std::lock_guard<std::recursive_mutex> lock(treeMutex);
            tree = factory.createTree(target_tree_id);
            // Re-initialize blackboard entries for the new tree
            initEntry(); 
        }
        
        std::cout << "[BrainTree] Successfully reloaded tree!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[BrainTree] Failed to reload tree: " << e.what() << std::endl;
    }
}
