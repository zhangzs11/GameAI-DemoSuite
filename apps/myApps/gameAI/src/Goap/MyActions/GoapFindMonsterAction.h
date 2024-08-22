#pragma once
#include "..\Core\GoapAction.h"
class GoapFindMonsterAction : public GoapAction {
public:

    GoapFindMonsterAction(std::string name, float cost)
        : GoapAction(name, cost) {
    }

    /*bool checkProceduralPrecondition(const GoapStatus& agentStatus) override {
        
    }*/

    ActionStatus perform(GoapStatus& agentStatus) override {
        updateEffects(agentStatus);
        return ActionStatus::SUCCESS;
    }

    void updateEffects(GoapStatus& status) {
        status.set("findMonster", true);
    }
};