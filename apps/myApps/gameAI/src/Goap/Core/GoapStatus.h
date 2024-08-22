#pragma once
#include <map>
#include <string>
#include <variant>
#include <iostream>

using StatusValue = std::variant<int, float, bool, std::string>;

class GoapStatus {
private:
    std::map<std::string, StatusValue> statuses;

public:
    void set(const std::string& key, const StatusValue& value) {
        statuses[key] = value;
    }

    bool get(const std::string& key, StatusValue& value) const {
        auto it = statuses.find(key);
        if (it != statuses.end()) {
            value = it->second;
            return true;
        }
        return false;
    }

    std::map<std::string, StatusValue> getStatuses() const {
        return statuses;
    }

    bool has(const std::string& key) const {
        return statuses.find(key) != statuses.end();
    }

    void remove(const std::string& key) {
        statuses.erase(key);
    }

    bool operator==(const GoapStatus& other) const {
        return statuses == other.statuses;
    }

    bool meetsConditions(const GoapStatus& conditions) const { 
        for (const auto& [key, value] : conditions.statuses) {
            auto it = statuses.find(key);
            if (it == statuses.end() || it->second != value) {
                return false; 
            }
        }
        return true;
    }

    GoapStatus applyEffects(const GoapStatus& effects) const {
        GoapStatus newState = *this;
        for (const auto& [key, value] : effects.getStatuses()) {
            newState.set(key, value);
        }
        return newState;
    }

    void print() const {
        for (const auto& [key, value] : statuses) {
            std::visit([key](auto&& arg) {
                std::cout << "Key: " << key << ", Value: " << arg << std::endl;
                }, value);
        }
    }
};

