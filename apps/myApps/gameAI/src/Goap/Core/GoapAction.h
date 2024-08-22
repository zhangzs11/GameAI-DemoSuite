#pragma once
#include <string>
#include <functional>
#include "GoapStatus.h"
enum class ActionStatus {
    SUCCESS,
    FAILURE,
    RUNNING
};

class GoapAction {
protected:
    GoapStatus preconditions;
    GoapStatus effects;
    float cost;
    std::string name;

public:
    GoapAction(std::string actionName, float actionCost) : name(actionName), cost(actionCost) {}

    void addPrecondition(const std::string& key, const StatusValue& value) {
        preconditions.set(key, value);
    }

    void addEffect(const std::string& key, const StatusValue& value) {
        effects.set(key, value);
    }

    const GoapStatus& getPreconditions() const {
        return preconditions;
    }

    const GoapStatus& getEffects() const {
        return effects;
    }

    float getCost() const {
        return cost;
    }

    void setCost(float newCost) {
        cost = newCost;
    }

    virtual bool checkProceduralPrecondition(const GoapStatus& agentStatus) {
        return agentStatus.meetsConditions(preconditions);
    }

    virtual ActionStatus perform(GoapStatus& agentStatus) {
        for (const auto& [key, value] : effects.getStatuses()) {
            agentStatus.set(key, value);
        }
        return ActionStatus::SUCCESS;
    }

    virtual ActionStatus isCompleted() const {
        return ActionStatus::SUCCESS;
    }

    void print() const {
        std::cout << "Action: " << name << ", Cost: " << cost << std::endl;
        std::cout << "Preconditions:" << std::endl;
        preconditions.print();
        std::cout << "Effects:" << std::endl;
        effects.print();
    }
};
