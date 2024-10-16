#pragma once
#include <unordered_map>
#include <string>
#include <vector>
#include <random>
#include <memory>
#include <any>

class Blackboard {
public:
    std::unordered_map<std::string, std::any> data;

public:
    Blackboard() {}

    void set(const std::string& key, const std::any& value) {
        data[key] = value;
    }

    std::any get(const std::string& key) const {
        auto it = data.find(key);
        if (it != data.end()) {
            return it->second;
        }
        // if the key is not exist
        return {};
    }

    template<typename T>
    T get(const std::string& key, const T& defaultValue) const {
        auto it = data.find(key);
        if (it != data.end()) {
            try {
                return std::any_cast<T>(it->second);
            }
            catch (const std::bad_any_cast&) {
                // 类型不匹配，返回默认值
            }
        }
        return defaultValue;
    }

    const std::unordered_map<std::string, std::any>& getData() const {
        return data;
    }
};
