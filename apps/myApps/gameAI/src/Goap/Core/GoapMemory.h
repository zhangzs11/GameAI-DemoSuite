#pragma once
#include <list>
#include <chrono>
#include "GoapStatus.h"
class GoapMemory {
private:
    std::list<std::pair<std::chrono::steady_clock::time_point, GoapStatus>> history;

public:
    void record(const GoapStatus& status) {
        auto now = std::chrono::steady_clock::now();
        history.push_back({ now, status });
    }

    bool getLatest(GoapStatus& status) const {
        if (!history.empty()) {
            status = history.back().second;
            return true;
        }
        return false;
    }

    bool getStatusAtTime(std::chrono::steady_clock::time_point time, GoapStatus& status) const {
        for (auto it = history.rbegin(); it != history.rend(); ++it) {
            if (it->first <= time) {
                status = it->second;
                return true;
            }
        }
        return false;
    }

    void rollbackTo(std::chrono::steady_clock::time_point time) {
        while (!history.empty() && history.back().first > time) {
            history.pop_back();
        }
    }

    void printHistory() const {
        for (const auto& [time, status] : history) {
            std::cout << "Time: " << std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count() << "s, Status:" << std::endl;
            status.print();
        }
    }
};
