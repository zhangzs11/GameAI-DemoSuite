#pragma once
#include "BasicBehaviorTree.h"
#include "..\Pathfinding\PathFinder.h"
// Heuristics 
float BTmanhattanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
    auto& currentVertex = vertices.at(current);
    auto& goalVertex = vertices.at(goal);
    return std::abs(currentVertex.position.x * ofGetWidth() - goalVertex.position.x * ofGetWidth()) + std::abs(currentVertex.position.y * ofGetHeight() - goalVertex.position.y * ofGetHeight());
}

//Action Task----------------------------------
//
class BTSayAction : public BTAction {
public:
    string sentence;
    BTSayAction(string sentence) :sentence(sentence) {}

    Status perform(Tick& tick) override {
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        if (!monster) {
            std::cerr << "BTMonsterSayAction failed: monster is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }
        monster->saySentence = sentence;
        // 设置当前正在执行的 action
        tick.setData("currentAction", std::string("SayAction: ") + sentence);
        return Status::Success;
    }
};

class BTChangeColorAction : public BTAction {
public:
    ofColor color;
    BTChangeColorAction(ofColor color) :color(color) {}

    Status perform(Tick& tick) override {
        // 从黑板获取 RigidbodyForWander 实例
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        if (!monster) {
            std::cerr << "BTChangeColorAction failed: monster is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }

        monster->color = color;
        tick.setData("currentAction", std::string("Change Color"));
        return Status::Success;
    }
};

class BTRandomResetFindTargetAction : public BTAction {
public:
    BTRandomResetFindTargetAction() {}

    Status perform(Tick& tick) override {
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        auto* pathFinder = std::any_cast<PathFinder*>(tick.getData("pathFinder"));
        auto* graphIndoor = std::any_cast<Map*>(tick.getData("graphIndoor"));
        auto* monsterWanderTime = std::any_cast<float*>(tick.getData("monsterWanderTime"));
        if (!monster || !pathFinder) {
            return Status::Failure; // 必要组件不可用
        }
        *monsterWanderTime = 0.1f;

        float width = ofGetWidth();
        float height = ofGetHeight();
        float randomX = ofRandom(0, width);
        float randomY = ofRandom(0, height);

        ofVec2f targetPoint = pathFinder->GetPositionOnMouseClick(randomX, randomY);

        pathFinder->targetPosition.set(targetPoint.x, targetPoint.y);
        pathFinder->startNode = graphIndoor->findNearestReachableNodeToClick(monster->position.x, monster->position.y);
        pathFinder->goalNode = graphIndoor->findNearestReachableNodeToClick(targetPoint.x, targetPoint.y);
        pathFinder->currentNodeIndex = 0;
        pathFinder->nextNodeIndex = 1;

        auto result = graphIndoor->graph->aStar(pathFinder->startNode, pathFinder->goalNode, BTmanhattanDistance);
        pathFinder->path = graphIndoor->graph->getPath(pathFinder->goalNode, result.first);
        tick.setData("isPathFinding", true);
        tick.setData("currentAction", std::string("Random Reset Find Target"));
        return Status::Success;
    }
};

class BTFindPathToPlayerAction : public BTAction {
public:
    BTFindPathToPlayerAction() {}

    Status perform(Tick& tick) override {
        auto* pathFinder = std::any_cast<PathFinder*>(tick.getData("pathFinder"));
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        auto* graphIndoor = std::any_cast<Map*>(tick.getData("graphIndoor"));
        auto* player = std::any_cast<Character*>(tick.getData("player"));

        if (!pathFinder || !monster) {
            return Status::Failure;
        }
        float playerX = player->position.x;
        float playerY = player->position.y;

        pathFinder->targetPosition.set(player->position.x, player->position.y);
        pathFinder->startNode = graphIndoor->findNearestReachableNodeToClick(monster->position.x, monster->position.y);
        pathFinder->goalNode = graphIndoor->findNearestReachableNodeToClick(player->position.x, player->position.y);
        pathFinder->currentNodeIndex = 0;
        pathFinder->nextNodeIndex = 1;

        auto result = graphIndoor->graph->aStar(pathFinder->startNode, pathFinder->goalNode, BTmanhattanDistance);
        pathFinder->path = graphIndoor->graph->getPath(pathFinder->goalNode, result.first);
        tick.setData("isPathFinding", true);
        tick.setData("currentAction", std::string("Find Path To Playe"));
        return Status::Success;
    }
};

class BTFollowRandomPathAction : public BTAction {
public:
    BTFollowRandomPathAction() {}

    Status perform(Tick& tick) override {
        auto* pathFinder = std::any_cast<PathFinder*>(tick.getData("pathFinder"));
        if (!pathFinder) {
            return Status::Failure;  // 路径完成或无效的 pathFinder
        }
        auto* monsterWanderTime = std::any_cast<float*>(tick.getData("monsterWanderTime"));

        pathFinder->updatePositionUsingGraph(tick.getFrameTime());
        *monsterWanderTime += tick.getFrameTime();
        tick.setData("currentAction", std::string("Follow Random Path"));
        return Status::Success;
    }
};

class BTFollowPathToPlayerAction : public BTAction {
public:
    BTFollowPathToPlayerAction() {}

    Status perform(Tick& tick) override {
        auto* pathFinder = std::any_cast<PathFinder*>(tick.getData("pathFinder"));
        if (!pathFinder) {
            return Status::Failure;  // 路径完成或无效的 pathFinder
        }

        pathFinder->updatePositionUsingGraph(tick.getFrameTime());
        // *monsterWanderTime += tick.getFrameTime();
        tick.setData("currentAction", std::string("Follow Path To Player"));
        return Status::Success;
    }
};

//ConditionNode--------------------------------
//
class CanSeePlayer : public BTConditionalDecoratorNode {

public:
    CanSeePlayer(Task* child)
        : BTConditionalDecoratorNode(child) {}

    bool condition(Tick& tick) override {
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        auto* player = std::any_cast<Character*>(tick.getData("player"));
        auto* graphIndoor = std::any_cast<Map*>(tick.getData("graphIndoor"));

        // 检查视线与每个障碍物的交叉
        for (const auto& polygon : graphIndoor->collisionList) {
            if (willCollide(player->position, monster->position, polygon)) {
                return false;  // 如果有障碍物阻挡视线，则返回false
            }
        }
        //std::cout << "can see" << std::endl;
        return true;  // 如果没有障碍物阻挡视线，则可以看到玩家
    }
};

class IsCloseToPlayer : public BTConditionalDecoratorNode {
private:
    float distanceThreshold;  // 距离阈值

public:
    IsCloseToPlayer(Task* child, float threshold)
        : BTConditionalDecoratorNode(child), distanceThreshold(threshold) {}

    bool condition(Tick& tick) override {
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        auto* player = std::any_cast<Character*>(tick.getData("player"));

        return monster->position.distance(player->position) < distanceThreshold;
    }
};

class IsWanderTimeExceeded : public BTConditionalDecoratorNode {
private:
    float timeThreshold;  // 时间阈值

public:
    IsWanderTimeExceeded(Task* child, float threshold)
        : BTConditionalDecoratorNode(child), timeThreshold(threshold) {}

    bool condition(Tick& tick) override {
        auto* monsterWanderTime = std::any_cast<float*>(tick.getData("monsterWanderTime"));
        return (*monsterWanderTime > timeThreshold) || (*monsterWanderTime == 0);
    }
};
