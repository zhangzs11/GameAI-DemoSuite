#pragma once
#include "GoapAction.h"
#include "Graph.h"
class GoapGoRandomAction : public GoapAction {
public:
    PathFinder* pathFinder;
    float goRange;
    float timeAfterFindRandom = 0;
    //time

    GoapGoRandomAction(std::string name, float cost, float range, PathFinder* pathFinder)
        : GoapAction(name, cost), goRange(range), pathFinder(pathFinder){
    }

    //bool checkProceduralPrecondition(const GoapStatus& agentStatus) override {
    //    
    //}

    ActionStatus CheckAgentStatus(GoapStatus& agentStatus) {
        StatusValue ifCanSee;
        agentStatus.get("seeMonster", ifCanSee);
        bool b1 = std::get<bool>(ifCanSee);
        StatusValue ifCanHear;
        agentStatus.get("hearMonster", ifCanHear);
        bool b2 = std::get<bool>(ifCanHear);
        if (b1 || b2) return ActionStatus::SUCCESS;
        else return ActionStatus::RUNNING;
    }

    ActionStatus perform(GoapStatus& agentStatus) override {
        if (timeAfterFindRandom < 20 && timeAfterFindRandom != 0) {
            timeAfterFindRandom += 0.1f;
            SteeringOutput steering = pathFinder->getSteering();
            pathFinder->update(steering, 1.0f / 60.0f);
            std::cout << "timeAfterFindRandom : "<< timeAfterFindRandom << std::endl;

        }
        else {
            float width = ofGetWidth();
            float height = ofGetHeight();
            float randomX = ofRandom(max(pathFinder->character->position.x - goRange, 0.0f), max(pathFinder->character->position.x + goRange, width));
            float randomY = ofRandom(max(pathFinder->character->position.y - goRange, 0.0f), max(pathFinder->character->position.y + goRange, height));

            ofVec2f targetPoint = pathFinder->onMouseClick(randomX, randomY);
            pathFinder->targetPosition.set(targetPoint.x, targetPoint.y);
            pathFinder->startNode = pathFinder->graph.findNearestReachableNodeToClick(pathFinder->character->position.x, pathFinder->character->position.y);
            pathFinder->goalNode = pathFinder->graph.findNearestReachableNodeToClick(targetPoint.x, targetPoint.y);
            pathFinder->currentNodeIndex = 0;
            pathFinder->nextNodeIndex = 1;

            auto result = pathFinder->graph.aStar(pathFinder->startNode, pathFinder->goalNode, manhattanDistance);
            pathFinder->path = pathFinder->graph.getPath(pathFinder->goalNode, result.first);
            pathFinder->character->ifHaveGotPath = true;

            timeAfterFindRandom = 1;
        }
        ActionStatus status = CheckAgentStatus(agentStatus);
        return status;
    }

    void updateEffects(GoapStatus& status) {
       
    }
};