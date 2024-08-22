#pragma once
#include "..\Core\GoapAction.h"
#include "Character.h"
class GoapDanceAction : public GoapAction {
public:
    Character* player;
    //time

    GoapDanceAction(std::string name, float cost, Character* player)
        : GoapAction(name, cost), player(player) {
    }

    /*bool checkProceduralPrecondition(const GoapStatus& agentStatus) override {

    }*/

    ActionStatus perform(GoapStatus& agentStatus) override {
        player->color = ofColor(ofRandom(0, 255), ofRandom(0, 255), ofRandom(0, 255));
        updateEffects(agentStatus);
        return ActionStatus::RUNNING;
    }

    void updateEffects(GoapStatus& status) {
        status.set("success", true);
    }
};