#pragma once
#include <vector>
#include <memory>
#include <algorithm>
#include "GoapStatus.h"
#include "GoapAction.h"
class GoapAgent {
public:
    GoapStatus currentState;
    GoapStatus goalState; 
    std::vector<GoapAction*> actions;
    std::vector<GoapAction*> currentPlan;
    std::vector<GoapStatus> closedList;

    struct Node {
        GoapStatus state;
        GoapAction* action;
        Node* parent;
        float gCost; 
        float hCost; 
        float fCost() const { return gCost + hCost; }

        Node(GoapStatus s, GoapAction* a, Node* p, float g, float h)
            : state(s), action(a), parent(p), gCost(g), hCost(h) {}
    };

    float heuristic(const GoapStatus& current, const GoapStatus& goal) {
        int mismatchCount = 0;
        for (const auto& [key, value] : goal.getStatuses()) {
            StatusValue current_value;
            if (!current.get(key, current_value) || current_value != value) {
                mismatchCount++;
            }
        }
        return static_cast<float>(mismatchCount);
    }

public:
    GoapAgent() : goalState() {}

    void setGoalState(const GoapStatus& state) {
        goalState = state;
    }

    void addAction(GoapAction* action) {
        actions.push_back(action);
    }

    void setCurrentState(const GoapStatus& state) {
        currentState = state;
    }

    bool isClosed(const GoapStatus& status) {
        return std::find_if(closedList.begin(), closedList.end(), [&status](const GoapStatus& s) {
            return s == status;
            }) != closedList.end();
    }

    bool planActions() {
        std::vector<Node*> openList;

        auto startNode = new Node(currentState, nullptr, nullptr, 0.0f, heuristic(currentState, goalState));
        openList.push_back(startNode);

        while (!openList.empty()) {
            auto current_node = *std::min_element(openList.begin(), openList.end(),
                [](const Node* a, const Node* b) {
                    return a->fCost() < b->fCost();
                });
            openList.erase(std::find(openList.begin(), openList.end(), current_node));
            closedList.push_back(current_node->state);

            if (current_node->state.meetsConditions(goalState)) {
                while (current_node != nullptr) {
                    if (current_node->action != nullptr) currentPlan.insert(currentPlan.begin(), current_node->action);
                    current_node = current_node->parent;
                }
                return true;
            }

            for (auto& action : actions) {
                if (action->checkProceduralPrecondition(current_node->state)) {
                    GoapStatus newState = current_node->state.applyEffects(action->getEffects()); // 使用效果预测状态变化
                    if (!isClosed(newState)) {
                        float tentativeGCost = current_node->gCost + action->getCost();
                        float hCost = heuristic(newState, goalState);
                        auto next_node = new Node(newState, action, current_node, tentativeGCost, hCost);
                        openList.push_back(next_node);
                    }
                }
            }
        }

        return false; // 如果开放列表清空仍未找到路径
    }

    // 执行计划中的第一个动作
    bool performAction() {
        if (!currentPlan.empty()) {
            auto action = currentPlan.front();
            auto status = action->perform(currentState);
            if (status == ActionStatus::SUCCESS) {
                currentPlan.erase(currentPlan.begin());
                return true;
            }
            else if (status == ActionStatus::RUNNING) {
                return true;
            }
            else if (status == ActionStatus::FAILURE) {
                currentPlan.clear();
                return false;
            }
        }
        return false;
    }

    void update() {
        if (currentPlan.empty()) {
            if (!planActions()) {
                std::cout << "No actions planned." << std::endl;
            }
        }

        if (!performAction()) {
            std::cout << "Action failed or no actions to perform." << std::endl;
        }
    }

    void printStatus() const {
        currentState.print();
    }
};
