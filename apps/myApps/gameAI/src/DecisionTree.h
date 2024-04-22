#pragma once
#include "Graph.h"
#include "WanderBehaviours.h"
float manhattanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
    auto& currentVertex = vertices.at(current);
    auto& goalVertex = vertices.at(goal);
    return std::abs(currentVertex.x * ofGetWidth() - goalVertex.x * ofGetWidth()) + std::abs(currentVertex.y * ofGetHeight() - goalVertex.y * ofGetHeight());
}

float euclideanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
    auto& currentVertex = vertices.at(current);
    auto& goalVertex = vertices.at(goal);
    return std::sqrt(std::pow(currentVertex.x * ofGetWidth() - goalVertex.x * ofGetWidth(), 2) + std::pow(currentVertex.y * ofGetHeight() - goalVertex.y * ofGetHeight(), 2));
}
class DecisionTreeNode {
public:
    virtual DecisionTreeNode* makeDecision() = 0;
};

class Action : public DecisionTreeNode {
public:
    PathFinder* pathfinder;
    RigidbodyForWander* wanderer;
    RigidbodyForDynamicSteering* dynamicer;
    Character* character;

    DirectedGraph graphIndoor;
    std::vector<int> path;


    DecisionTreeNode* makeDecision() override {
        return this;
    }

    void performAction() {
        //std::cout << "Performing an action" << std::endl;
        //TO DO：Performance the action
    }
};

class Decision : public DecisionTreeNode {
protected:
    DecisionTreeNode* trueNode;
    DecisionTreeNode* falseNode;

    virtual bool getBranch() {
        return true;
    }

public:
    Decision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode)
        : trueNode(trueNode), falseNode(falseNode) {}

    DecisionTreeNode* makeDecision() override {
        if (getBranch()) {
            return trueNode->makeDecision();
        }
        else {
            return falseNode->makeDecision();
        }
    }
};
class IfFindingDecision : public Decision {
protected:
    PathFinder* pathfinder;
public:
    IfFindingDecision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, PathFinder* pathfinder)
        : Decision(trueNode, falseNode), pathfinder(pathfinder){}

    bool getBranch() override {
        std::cout << "IfFindingDecision" << std::endl;
        return pathfinder->character->isPathFinding;
    }
};
class FindTimeDecision : public Decision {
protected:
    float timeThreshold;
    PathFinder* pathfinder;
public:
    FindTimeDecision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, PathFinder* pathfinder, float threshold)
        : Decision(trueNode, falseNode), pathfinder(pathfinder), timeThreshold(threshold) {}
    bool getBranch() override {
        std::cout << "FindTimeDecision" << std::endl;
        return pathfinder->character->findTime > timeThreshold;
    }
};
class IfCollisionForwardDecision : public Decision {
protected:
    float range;
    PathFinder* pathfinder;
public:
    IfCollisionForwardDecision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, PathFinder* pathfinder, float range)
        : Decision(trueNode, falseNode), pathfinder(pathfinder), range(range){}
    bool getBranch() override {
        std::cout << "IfCollisionForwardDecision" << std::endl;
        return willCollide(pathfinder->character->position, pathfinder->character->position + ofVec2f(cos(pathfinder->character->orientation), sin(pathfinder->character->orientation)) * range, pathfinder->collisionList);
    }
};
class DistanceWithPathTargetDecision : public Decision {
protected:
    float distanceThreshold;
    PathFinder* pathfinder;
public:
    DistanceWithPathTargetDecision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, PathFinder* pathfinder, float threshold)
        : Decision(trueNode, falseNode), pathfinder(pathfinder), distanceThreshold(threshold) {}
    bool getBranch() override {
        std::cout << "DistanceWithPathTargetDecision" << std::endl;
        return pathfinder->playerPosition.distance(pathfinder->targetPosition) > distanceThreshold;
    }
};
class SpeedDecision : public Decision {
protected:
    float speedThreshold;
    Character* character;
public:
    SpeedDecision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, Character* character, float threshold)
        : Decision(trueNode, falseNode), character(character), speedThreshold(threshold) {}
    bool getBranch() override {
        std::cout << "SpeedDecision" << std::endl;
        return character->velocity.length() > speedThreshold;
    }
};
class StopFindingAction : public Action {
public:
    DecisionTreeNode* makeDecision() override {
        performAction(); // 执行动作
        return this; // 返回当前Action实例
    }

    void performAction() {
        character->isPathFinding = false;
        character->findTime = 0;
        SteeringOutput steering = wanderer->getSteering();
        wanderer->update(steering, 1.0f / 60.0f);
        std::cout << "StopFindingAction" << std::endl;
    }
};
class FindingAction : public Action {
public:
    DecisionTreeNode* makeDecision() override {
        performAction(); // 执行动作
        return this; // 返回当前Action实例
    }

    void performAction() {
        character->isPathFinding = true;
        if (character->ifHaveGotPath) {
            SteeringOutput steering = pathfinder->getSteering();
            pathfinder->update(steering, 1.0f / 60.0f);
        }

        float currentTime = ofGetElapsedTimef();
        character->findTime += 1.0f / 60.0f;
        std::cout << "FindingAction" << std::endl;

    }
};
class ResetFindTargetAction : public Action {
public:
    DecisionTreeNode* makeDecision() override {
        performAction(); // 执行动作
        return this; // 返回当前Action实例
    }

    void performAction() {
        character->ifHaveGotPath = false;
        character->isPathFinding = true;
        character->findTime = 0;

        float width = ofGetWidth();
        float height = ofGetHeight();

        float randomX = ofRandom(0, width);
        float randomY = ofRandom(0, height);

        ofVec2f targetPoint = pathfinder->onMouseClick(randomX, randomY);

        pathfinder->targetPosition.set(targetPoint.x, targetPoint.y);
        pathfinder->startNode = graphIndoor.findNearestReachableNodeToClick(pathfinder->character->position.x, pathfinder->character->position.y);
        pathfinder->goalNode = graphIndoor.findNearestReachableNodeToClick(targetPoint.x, targetPoint.y);
        pathfinder->currentNodeIndex = 0;
        pathfinder->nextNodeIndex = 1;

        auto result = pathfinder->graph.aStar(pathfinder->startNode, pathfinder->goalNode, manhattanDistance);
        auto came_from = result.first;
        path = graphIndoor.getPath(pathfinder->goalNode, came_from);
        pathfinder->path = path;
        character->ifHaveGotPath = true;
        std::cout << "ResetFindTargetAction" << std::endl;
    }
};
class WanderAction : public Action {
public:
    DecisionTreeNode* makeDecision() override {
        performAction(); // 执行动作
        return this; // 返回当前Action实例
    }

    void performAction() {
        SteeringOutput steering = wanderer->getSteering();
        wanderer->update(steering, 1.0f / 60.0f);
        std::cout << "WanderAction" << std::endl;
    }
};
class GobackAction : public Action {
public:
    DecisionTreeNode* makeDecision() override {
        performAction(); // 执行动作
        return this; // 返回当前Action实例
    }

    void performAction() {
        SteeringOutput steering = dynamicer->getSteering(-1 * dynamicer->character->velocity);
        dynamicer->update(steering, 1.0f / 60.0f);
        std::cout << "GobackAction" << std::endl;
    }
};
class SpeedUpAction : public Action {
public:
    DecisionTreeNode* makeDecision() override {
        performAction();
        return this;
    }

    void performAction() {
        SteeringOutput steering = dynamicer->getSteering(dynamicer->character->velocity);
        dynamicer->update(steering, 1.0f / 60.0f);
        std::cout << "SpeedUpAction" << std::endl;
    }
};