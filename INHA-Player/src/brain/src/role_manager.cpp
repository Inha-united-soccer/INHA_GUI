#include "brain.h"
#include "brain_tree.h"
#include "behaviortree_cpp/behavior_tree.h"

using namespace BT;

class RoleManager : public BrainTreeNode {
public:
    RoleManager(const std::string& name, const NodeConfiguration& config, Brain* brain)
        : BrainTreeNode(name, config, brain) {}

    static PortsList providedPorts() {
        return {
            InputPort<std::string>("current_role"),
            OutputPort<std::string>("new_role")
        };
    }

    NodeStatus tick() override {
        // 1. 현재 게임 정보를 가져온다
        auto ball = brain->data->ball;
        double ballX = ball.posToField.x;
        // 2. 역할 결정 로직
        // - GK: 고정
        // - Lead: 공을 가진 로봇은 무조건 공격(Striker)
        // - Supporter: 
        //    A. 공이 아주 깊숙한 곳(>1.0m)에 있으면 -> 같이 올라가서 공격 가담 (Striker)
        //    B. 그 외(수비, 미드필드) 상황이면 -> 뒤에서 수비 라인 유지 (Defender)

        std::string role; // Declare role variable
        std::string defaultRole = brain->config->playerRole;
        bool isLead = brain->tree->getEntry<bool>("is_lead");
        
        // 골키퍼 우선권 
        if (defaultRole == "goal_keeper") {
             role = "goal_keeper";
        }
        else {
            // 필드 플레이어 역할 분담
            
            // 1) 리더(Lead)는 무조건 Striker
            if (isLead) {
                role = "striker";
            }
            // 2) 리드가 아닌 경우 (Supporter)
            else {
                // 공격/수비 상황 판단 (Game Phase)
                
                // 공이 상대 진영 깊숙이(<-1.0m) 있다면? -> 확실한 공격 찬스
                // 과감하게 올라가서 패스를 받거나 세컨볼을 노림
                if (ballX < -1.0) {
                    role = "striker"; // OffTheBall 수행
                }
                // 아직 미드필드 싸움이거나 수비 상황이라면? -> 밸런스 유지
                // 섣불리 올라가지 않고 수비 라인을 지킴
                else {
                    role = "defender"; 
                }
            }
        }
        

        // 블랙보드 업데이트
        brain->tree->setEntry("player_role", role);
        setOutput("new_role", role);

        brain->log->logToScreen("tree/RoleManager", "Role: " + role, 0xFF00FFFF);

        return NodeStatus::SUCCESS;
    }
};

void RegisterRoleManager(BT::BehaviorTreeFactory &factory, Brain* brain){
    factory.registerBuilder<RoleManager>("RoleManager", 
        [brain](const string &name, const NodeConfiguration &config) { 
            return make_unique<RoleManager>(name, config, brain); 
        });
}
