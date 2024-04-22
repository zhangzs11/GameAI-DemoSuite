#pragma once
#include "GoapAction.h"
#include "Character.h"
class GoapShootAction : public GoapAction {
public:
    Character* player;
    Character* shootTarget;
    GoapShootAction(std::string name, float cost, Character* player, Character* shootTarget)
        : GoapAction(name, cost), player(player), shootTarget(shootTarget) {
    }

    /*bool checkProceduralPrecondition(const GoapStatus& agentStatus) override {

    }*/

    ActionStatus perform(GoapStatus& agentStatus) override {
        player->shootTime = 1;
        player->shootTarget = shootTarget->position;
        shootTarget->ifDie = true;
        updateEffects(agentStatus);
        return ActionStatus::SUCCESS;
    }

    void updateEffects(GoapStatus& status) {
        status.set("MonsterHP", 0);
    }
};