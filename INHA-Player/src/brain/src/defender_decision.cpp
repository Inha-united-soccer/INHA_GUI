#include "brain.h"
#include "brain_tree.h"
#include "defender_decision.h"
#include "utils/math.h" // For pointMinDistToLine, etc

#include <cstdlib>
#include <ctime>
#include <cmath>

#define REGISTER_DEFENDER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfiguration &config) { return make_unique<Name>(name, config, brain); });


void RegisterDefenderDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_DEFENDER_BUILDER(DefenderDecision)
    REGISTER_DEFENDER_BUILDER(DefenderClearingDecide)
    REGISTER_DEFENDER_BUILDER(CalcDefenderClearingDir)

}

// ==========================================
// DefenderDecision (from DefenderDecide in decision_role.cpp)
// ==========================================
NodeStatus DefenderDecision::tick() {
    double paramChaseSpeed = 0.8;
    double paramDefenseLineX = -10.0; // 기본값: 제한 없음
    double paramKickThreshold = 0.3;  
    
    // Default 값
    if (auto val = brain->tree->getEntry<double>("Strategy.param_chase_speed_limit")) paramChaseSpeed = val;
    if (auto val = brain->tree->getEntry<double>("Strategy.param_defense_line_x")) paramDefenseLineX = val;
    if (auto val = brain->tree->getEntry<double>("Strategy.param_kick_threshold")) paramKickThreshold = val;
    
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision;
    getInput("decision_in", lastDecision);
    double clearingThreshold;
    getInput("clearing_threshold", clearingThreshold);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    const double goalpostMargin = 0.5; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");
    
    // 장애물 회피 로직
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < -2.0 
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;


    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < 0.1
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    bool passFound = brain->tree->getEntry<bool>("pass_found");
    bool isLead = brain->tree->getEntry<bool>("is_lead");

    auto pose = brain->data->robotPoseToField;

    // ===clearing condition===
    bool shouldClearing = false;
    auto Opponents = brain->data->getRobots(); 
    double OpponentBallDist = 999999.0;
    for (const auto& opponent : Opponents){
        if (opponent.label != "Opponent") continue; // 상대팀이 아니면 스킵
        double dist = norm(opponent.posToField.x - ball.posToField.x, opponent.posToField.y - ball.posToField.y);
        if(dist<OpponentBallDist) OpponentBallDist = dist;
    } // calculate nearest ball-opponent distance
    shouldClearing = (OpponentBallDist < clearingThreshold) ? true : false;
    // =========================

    // 1) 공을 모르면 -> find
    if (!(iKnowBallPos || tmBallPosReliable)) {
        newDecision = "find";
        color = 0xFFFFFFFF;
    }
    // 2) non-lead인데 레인 밖이면 -> return (레인 복귀)
    else if (!isLead) {
        newDecision = "return";
        color = 0xFFFF00FF;
        
        if (lastDecision != "return") {
        brain->tree->setEntry("return_x", pose.x);

        // 화면 로그로 확인 (콘솔/화면)
        brain->log->logToScreen(
            "debug/ReturnTarget",
            format("Saved return_x=%.2f (pose=(%.2f,%.2f,%.2f)) target=(%.2f,-2.5)",
                   pose.x, pose.x, pose.y, pose.theta, pose.x),
            0xFFFF00FF
        );
        }
    }
    // 3) lead인데 opponent가 너무 가까우면 clearing
    else if (shouldClearing) {
        newDecision = "clearing";
        color = 0xFFFF00FF;
    }
    // 4) clearing 할 상황은 아닌데 lead이면 -> (기존대로) chase / pass / adjust
    else if (isLead) {
        // 멀면 chase
        bool wasChasing = (lastDecision == "chase");
        if (ballRange > chaseRangeThreshold * (wasChasing ? 0.9 : 1.0)) {
            newDecision = "chase";
            color = 0x0000FFFF;
        }
        // 킥(패스) 조건
        else if (
            (reachedKickDir) &&
            brain->data->ballDetected &&
            std::fabs(brain->data->ball.yawToRobot) < 0.1 &&
            !avoidKick &&
            ball.range < 1.5
        ) {
            if (passFound) newDecision = "pass";
            else newDecision = "kick";
            color = 0x00FF00FF;
            brain->data->isFreekickKickingOff = false;
        }
        // 그 외 adjust
        else {
            newDecision = "adjust";
            color = 0xFFFF00FF;
        }
    }
    // 5) non-lead면서 레인 안이면 -> side_chase (항상)
    else {
        newDecision = "side_chase";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Defend",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

// ==========================================
// DefenderClearingDecide
// ==========================================
NodeStatus DefenderClearingDecide::tick() {
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision;
    getInput("clearing_decision_in", lastDecision);

    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField;

    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir =
        deltaDir * lastDeltaDir <= 0 &&
        fabs(deltaDir) < 0.1 &&
        dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");

    string newDecision;
    auto color = 0xFFFFFFFF;

    if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0)) {
        newDecision = "chase";
        color = 0x0000FFFF;
    } else if (reachedKickDir &&
               brain->data->ballDetected &&
               fabs(ballYaw) < 0.1 &&
               ball.range < 1.5) {
        newDecision = "kick";
        color = 0x00FF00FF;
    } else {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("clearing_decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Clearing",
        format("ClearingDecision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f",
               newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f),
        color
    );
    return NodeStatus::SUCCESS;
}

// ==========================================
// CalcDefenderClearingDir (Renamed from CalcClearingDir in kick.cpp of Defender repo)
// ==========================================
NodeStatus CalcDefenderClearingDir::tick() {
    double OffsetDegree;
    getInput("offset_degree", OffsetDegree);
    
    // auto ballPos = brain->data->ball.posToField;
    // Note: Defender repo used emaball. Assuming Striker repo has emaball or I should use ball.
    // Let's use ball for safety if emaball is not guaranteed in Striker brain data struct, 
    // BUT if Unified means merging all, I assume brain_data.h is consistent.
    // Accessing emaball from brain->data.
    auto ballPos = brain->data->ball.posToField; 
    auto opponents = brain->data->getRobots();

    int nearestIdx = -1;
    double minDist = 1e9;

    for (size_t i = 0; i < opponents.size(); ++i) {
        const auto& opponent = opponents[i];
        if (opponent.label != "Opponent") continue;
        double dist = norm(opponent.posToField.x - ballPos.x,
                           opponent.posToField.y - ballPos.y);
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = static_cast<int>(i);
        }
    }

    double clearingDir = 0.0; // 기본은 +x 방향
    if (nearestIdx >= 0) {
        const auto& nearestOpponent = opponents[nearestIdx];
        double angleToOpponent = atan2(
            nearestOpponent.posToField.y - ballPos.y,
            nearestOpponent.posToField.x - ballPos.x
        );
        const double offset = deg2rad(OffsetDegree);
        // 상대 각도에서 +x 방향(0rad) 쪽으로 30도 꺾기
        clearingDir = angleToOpponent + (angleToOpponent > 0.0 ? -offset : offset);
        clearingDir = toPInPI(clearingDir);
    }

    brain->data->kickDir = clearingDir;

    brain->log->setTimeNow();
    brain->log->log(
        "field/clearing_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{(float)ballPos.x, (float)-ballPos.y}})
            .with_colors({0x00FF00FF})
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}