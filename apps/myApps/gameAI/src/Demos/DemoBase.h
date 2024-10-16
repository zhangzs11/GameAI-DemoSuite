#pragma once
#include <fstream>
#include <filesystem>
#include <string>

#include "../include/json.hpp"
#include "ofxImGui.h"

#include "..\Pathfinding\PathFinder.h"
using json = nlohmann::json;

const std::string defaultMapDataFolder = "../mapData/";

// 保存地图到JSON文件
void saveMapToJson(const Map& map, const std::string& filename) {
    json j;

    // 序列化 Graph 的节点
    j["graph"]["nodes"] = json::array();
    for (const auto& [id, node] : map.graph->vertices) {
        j["graph"]["nodes"].push_back({
            {"id", id},
            {"position", {node.position.x, node.position.y}}
            });
    }

    // 序列化 Graph 的边
    j["graph"]["edges"] = json::array();
    for (const auto& edge : map.graph->edges) {
        j["graph"]["edges"].push_back({
            {"source", edge.getSource()},
            {"sink", edge.getSink()},
            {"weight", edge.getWeight()}
            });
    }

    // 序列化 PolygonCollision
    j["obstacles"] = json::array();
    for (const auto& polygon : map.collisionList) {
        json vertices = json::array();
        for (const auto& vertex : polygon.vertices) {
            vertices.push_back({ vertex.x, vertex.y });
        }
        j["obstacles"].push_back({
            {"id", polygon.id},
            {"vertices", vertices}
            });
    }

    // 检查并创建文件夹
    if (std::filesystem::create_directories(defaultMapDataFolder)) {
        std::cout << "Directory created: " << defaultMapDataFolder << std::endl;
    }
    else {
        std::cout << "Directory already exists or failed to create: " << defaultMapDataFolder << std::endl;
    }

    // 组合文件路径
    std::string fullPath = defaultMapDataFolder + filename;
    std::cout << "Full file path: " << fullPath << std::endl;  // 输出调试信息

    // 将 JSON 写入文件
    std::ofstream file(fullPath);
    if (file.is_open()) {
        file << j.dump(4);  // 使用 4 空格缩进格式化输出
        file.close();
        std::cout << "Map saved to " << fullPath << std::endl;
    }
    else {
        std::cerr << "Error opening file for writing: " << fullPath << std::endl;
    }
}

// 从JSON文件加载地图数据
void loadMapFromJson(Map& map, const std::string& filename) {
    // 组合文件路径
    std::string fullPath = defaultMapDataFolder + filename;

    std::ifstream file(fullPath);
    if (!file.is_open()) {
        std::cerr << "Error opening file for reading: " << fullPath << std::endl;
        return;
    }

    json j;
    file >> j;  // 从文件中读取 JSON
    file.close();

    // 清空当前 Map 数据
    map.graph->vertices.clear();
    map.graph->edges.clear();
    map.collisionList.clear();

    // 反序列化 Graph 的节点
    for (const auto& nodeJson : j["graph"]["nodes"]) {
        int id = nodeJson["id"];
        float x = nodeJson["position"][0];
        float y = nodeJson["position"][1];
        map.graph->addVertex(id, x, y);
    }

    // 反序列化 Graph 的边
    for (const auto& edgeJson : j["graph"]["edges"]) {
        int source = edgeJson["source"];
        int sink = edgeJson["sink"];
        float weight = edgeJson["weight"];
        map.graph->addEdge(source, sink, weight);
    }

    // 反序列化 PolygonCollision
    for (const auto& obstacleJson : j["obstacles"]) {
        int id = obstacleJson["id"];
        std::vector<ofVec2f> vertices;
        for (const auto& vertexJson : obstacleJson["vertices"]) {
            float x = vertexJson[0];
            float y = vertexJson[1];
            vertices.push_back(ofVec2f(x, y));
        }
        map.addPolygon(id, vertices);
    }

    std::cout << "Map loaded from " << fullPath << std::endl;
}
class DemoBase {
public:
    virtual ~DemoBase() = default;

    virtual void setup() = 0;
    virtual void update(float deltaTime) = 0;
    virtual void draw() = 0;
    virtual void keyPressed(int key) = 0;
    virtual void mousePressed(int x, int y, int button) = 0;
};


/*
#pragma once

#include "ofMain.h"
#include "DemoBase.h"
#include "MovementDemo.h"
// 其他 Demo 的头文件

class ofApp : public ofBaseApp {
private:
    std::unique_ptr<DemoBase> currentDemo;

public:
    void setup() override {
        // 根据初始选择加载一个Demo
        currentDemo = std::make_unique<MovementDemo>();
        currentDemo->setup();
    }

    void update() override {
        currentDemo->update();
    }

    void draw() override {
        currentDemo->draw();
    }

    void keyPressed(int key) override {
        currentDemo->keyPressed(key);
    }

    void mousePressed(int x, int y, int button) override {
        currentDemo->mousePressed(x, y, button);
    }

    void switchDemo(int demoIndex) {
        // 根据 demoIndex 切换 Demo
        switch (demoIndex) {
        case 0:
            currentDemo = std::make_unique<MovementDemo>();
            break;
        case 1:
            // 切换到其他 Demo
            break;
            // 其他 Demo 切换逻辑
        }
        currentDemo->setup();  // 重新初始化新的 Demo
    }
};
*/