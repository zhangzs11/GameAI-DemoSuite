#pragma once
#include "..\Core\GoapAction.h"
#include "Graph.h"
#include "Character.h"

class GoapGoToTargetAction : public GoapAction {
public:
    PathFinder* pathFinder;
    Character* targetCharacter;
    std::string targetIdentifier; // 新增目标标识符

    GoapGoToTargetAction(std::string name, float cost, std::string identifier, PathFinder* pathFinder, Character* targetCharacter)
        : GoapAction(name, cost),targetIdentifier(identifier), pathFinder(pathFinder), targetCharacter(targetCharacter){
    }

    //bool checkProceduralPrecondition(const GoapStatus& agentStatus) override {
    //   
    //    return true;
    //}

    ActionStatus isCompleted() const override {
        if (pathFinder->character->position.distance(targetCharacter->position) < 50) {
            if (targetIdentifier == "Gun") {
                targetCharacter->itemHasPickUp = true;
            }
            return ActionStatus::SUCCESS;
        }
        else {
            return ActionStatus::RUNNING;
        }
    }

    ActionStatus CheckAgentStatus(GoapStatus& agentStatus) {
        if (targetIdentifier == "Monster") {
            StatusValue ifCanSee;
            agentStatus.get("seeMonster", ifCanSee);
            bool b1 = std::get<bool>(ifCanSee);
            if (b1) return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }

    ActionStatus perform(GoapStatus& agentStatus) override {

        pathFinder->targetPosition = targetCharacter->position;

        pathFinder->startNode = pathFinder->graph.findNearestReachableNodeToClick(pathFinder->character->position.x, pathFinder->character->position.y);
        pathFinder->goalNode = pathFinder->graph.findNearestReachableNodeToClick(targetCharacter->position.x, targetCharacter->position.y);
        pathFinder->currentNodeIndex = 0;
        pathFinder->nextNodeIndex = 1;

        auto result = pathFinder->graph.aStar(pathFinder->startNode, pathFinder->goalNode, manhattanDistance);
        auto came_from = result.first;
        pathFinder->path = pathFinder->graph.getPath(pathFinder->goalNode, came_from);

        SteeringOutput steering = pathFinder->getSteering();
        pathFinder->update(steering, 1.0f / 60.0f);

        ActionStatus status = CheckAgentStatus(agentStatus);
        if (status == ActionStatus::SUCCESS) {
            return ActionStatus::SUCCESS;
        }
        status = isCompleted();
        if (status == ActionStatus::SUCCESS) {
            updateEffects(agentStatus);
            return ActionStatus::SUCCESS;
        }
        else if (status == ActionStatus::RUNNING) {
            return ActionStatus::RUNNING;
        }

    }

    void updateEffects(GoapStatus& status) {
        status.set("At" + targetIdentifier, true);
    }
};
