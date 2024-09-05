#pragma once
#include <fstream>
#include <filesystem>
#include <string>

#include "../include/json.hpp"

#include "DemoBase.h"
#include "PathFinder.h"

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

enum ModeType {
    Add_Node,
    Remove_Node,
    Add_Edge,
    Remove_Edge,
    Add_Obstacle,
    Remove_Obstacle,
    Path_Finding
};

// Heuristics 
inline float manhattanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
    auto& currentVertex = vertices.at(current);
    auto& goalVertex = vertices.at(goal);
    return std::abs(currentVertex.position.x * ofGetWidth() - goalVertex.position.x * ofGetWidth()) + std::abs(currentVertex.position.y * ofGetHeight() - goalVertex.position.y * ofGetHeight());
}

inline float euclideanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
    auto& currentVertex = vertices.at(current);
    auto& goalVertex = vertices.at(goal);
    return std::sqrt(std::pow(currentVertex.position.x * ofGetWidth() - goalVertex.position.x * ofGetWidth(), 2) + std::pow(currentVertex.position.y * ofGetHeight() - goalVertex.position.y * ofGetHeight(), 2));
}

class PathfindingDemo : public DemoBase {
public:
    Character mainCharacter;
    bool isPathFinding;
    DirectedGraph graph;
    Map map;
    std::shared_ptr<Arrive> arriveBehavior;
    std::shared_ptr<Seek> seekBehavior;
    PathFinder pathFinder;

    ModeType currentMode;

    void setup() override {
        // Init character
        mainCharacter.position = ofVec2f(100, 100);
        mainCharacter.color = ofColor::blue;
        mainCharacter.radius = 20;
        mainCharacter.isDead = false;
        isPathFinding = false;
        // Init map
        map.graph = &graph;

        // Init Arrive and Seek
        arriveBehavior = std::make_shared<Arrive>(&mainCharacter, 100.0f, 50.0f, 50.0f, 10.0f);
        seekBehavior = std::make_shared<Seek>(&mainCharacter, 100.0f, 50.0f, mainCharacter.position);

        // Init pathfinder
        pathFinder.character = &mainCharacter;
        pathFinder.map = &map;
        pathFinder.arriveBehavior = arriveBehavior;
        pathFinder.seekBehavior = seekBehavior;
    }

    void update(float deltaTime) override {
        if (isPathFinding) {
            pathFinder.updatePositionUsingGraph(deltaTime);
        }
        
    }

    void draw() override {
        graph.drawMap();
        graph.drawPath(pathFinder.path);
        for (const auto& polygon : map.collisionList) {
            polygon.drawPolygon();
        }
        renderImGui();
    }

    void renderImGui() {
        ImGui::Begin("Path finding");

        // 选择操作mode
        const char* modes[] = { "Graph Editing", "Obstacle Editing", "Pathfinding Settings", "Save/Load Map" };
        static int currentMode = 0;

        ImGui::Combo("Edit Mode", &currentMode, modes, IM_ARRAYSIZE(modes));

        if (currentMode == 0 && ImGui::CollapsingHeader("Graph Editing")) {
            // 添加节点
            static int newNodeId = 0;  // 初始节点ID
            static float newNodePosition[2] = { 0.5f, 0.5f };  // 初始位置

            ImGui::InputInt("New Node ID", &newNodeId);  // 输入新节点ID
            ImGui::InputFloat2("New Node Position", newNodePosition);  // 输入新节点位置

            if (ImGui::Button("Add Node")) {
                graph.addVertex(newNodeId, newNodePosition[0], newNodePosition[1]);
            }

            // 删除节点
            static int selectedDeleteNode = -1;
            if (ImGui::BeginCombo("Select Node to Delete", (selectedDeleteNode == -1) ? "Choose..." : std::to_string(selectedDeleteNode).c_str())) {
                for (const auto& [id, _] : graph.vertices) {
                    bool isSelected = (selectedDeleteNode == id);
                    if (ImGui::Selectable(std::to_string(id).c_str(), isSelected)) {
                        selectedDeleteNode = id;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::Button("Delete Node") && selectedDeleteNode != -1) {
                graph.deleteVertex(selectedDeleteNode);
                selectedDeleteNode = -1;  // 重置选择
            }

            // 添加边
            static int addNode1 = -1;
            static int addNode2 = -1;
            static float edgeWeight = 1.0f;  // 初始权重

            if (ImGui::BeginCombo("Select Start Node to Add Edge", (addNode1 == -1) ? "Choose..." : std::to_string(addNode1).c_str())) {
                for (const auto& [id, _] : graph.vertices) {
                    bool isSelected = (addNode1 == id);
                    if (ImGui::Selectable(std::to_string(id).c_str(), isSelected)) {
                        addNode1 = id;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            if (ImGui::BeginCombo("Select End Node to Add Edge", (addNode2 == -1) ? "Choose..." : std::to_string(addNode2).c_str())) {
                for (const auto& [id, _] : graph.vertices) {
                    bool isSelected = (addNode2 == id);
                    if (ImGui::Selectable(std::to_string(id).c_str(), isSelected)) {
                        addNode2 = id;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::InputFloat("Edge Weight", &edgeWeight, 0.1f, 1.0f, "%.2f");  // 输入权重

            if (ImGui::Button("Add Edge") && addNode1 != -1 && addNode2 != -1 && addNode1 != addNode2) {
                graph.addEdgeDouble(addNode1, addNode2, edgeWeight);  // 添加双向边
                addNode1 = -1;
                addNode2 = -1;  // 重置选择
            }

            // 删除边
            static int deleteNode1 = -1;
            static int deleteNode2 = -1;

            if (ImGui::BeginCombo("Select Start Node for Edge", (deleteNode1 == -1) ? "Choose..." : std::to_string(deleteNode1).c_str())) {
                for (const auto& [id, _] : graph.vertices) {
                    bool isSelected = (deleteNode1 == id);
                    if (ImGui::Selectable(std::to_string(id).c_str(), isSelected)) {
                        deleteNode1 = id;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            if (ImGui::BeginCombo("Select End Node for Edge", (deleteNode2 == -1) ? "Choose..." : std::to_string(deleteNode2).c_str())) {
                for (const auto& [id, _] : graph.vertices) {
                    bool isSelected = (deleteNode2 == id);
                    if (ImGui::Selectable(std::to_string(id).c_str(), isSelected)) {
                        deleteNode2 = id;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            if (ImGui::Button("Delete Edge") && deleteNode1 != -1 && deleteNode2 != -1) {
                graph.deleteEdgeDouble(deleteNode1, deleteNode2);  // 删除双向边
                deleteNode1 = -1;
                deleteNode2 = -1;  // 重置选择
            }
        }

        if (currentMode == 1 && ImGui::CollapsingHeader("Obstacle Editing")) {
            // 添加多边形障碍物
            static std::vector<ofVec2f> newObstacleVertices;
            static float vertexPos[2] = { 0.0f, 0.0f };
            static int polygonId = 0;

            ImGui::InputInt("Polygon ID", &polygonId);  // 手动输入多边形ID
            ImGui::InputFloat2("New Vertex Position", vertexPos);
            if (ImGui::Button("Add Vertex")) {
                newObstacleVertices.push_back(ofVec2f(vertexPos[0], vertexPos[1]));
            }

            if (ImGui::Button("Create Obstacle") && newObstacleVertices.size() >= 3) {
                map.addPolygon(polygonId, newObstacleVertices);
                newObstacleVertices.clear();  // 清空，准备创建下一个障碍物
            }

            // 删除障碍物
            static int selectedObstacle = -1;
            if (ImGui::BeginCombo("Select Obstacle to Delete", (selectedObstacle == -1) ? "Choose..." : std::to_string(selectedObstacle).c_str())) {
                for (const auto& [id, _] : map.collisionList) {
                    bool isSelected = (selectedObstacle == id);
                    if (ImGui::Selectable(std::to_string(id).c_str(), isSelected)) {
                        selectedObstacle = id;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            if (ImGui::Button("Delete Obstacle") && selectedObstacle != -1) {
                map.removePolygon(selectedObstacle);
                selectedObstacle = -1;  // 重置选择
            }
        }

        if (currentMode == 2 && ImGui::CollapsingHeader("Pathfinding Settings")) {
            const char* algorithms[] = { "Dijkstra", "A*", "BFS", "DFS" };
            static int currentAlgorithm = 0;

            ImGui::Combo("Algorithm", &currentAlgorithm, algorithms, IM_ARRAYSIZE(algorithms));
            // 处理算法选择的逻辑...
        }

        // 添加保存和加载地图的功能
        if (currentMode == 3 && ImGui::CollapsingHeader("Save/Load Map")) {
            static char fileName[128] = "my_map";  // 默认文件名（不包含后缀）
            static bool saveSuccess = false;  // 保存成功标志
            static bool loadSuccess = false;  // 加载成功标志
            static std::string savePath;  // 保存的完整路径
            static std::string loadPath;  // 加载的完整路径

            // 文件名输入框
            ImGui::InputText("File Name", fileName, IM_ARRAYSIZE(fileName));

            // 保存地图按钮
            if (ImGui::Button("Save Map")) {
                savePath = defaultMapDataFolder + std::string(fileName) + ".json";  // 自动加上 ".json"
                saveMapToJson(map, savePath);  // 调用保存函数
                saveSuccess = true;  // 假设保存成功，可以根据 saveMapToJson 函数返回值来进一步优化
                loadSuccess = false;  // 重置加载状态
            }

            // 加载地图按钮
            if (ImGui::Button("Load Map")) {
                loadPath = defaultMapDataFolder + std::string(fileName) + ".json";  // 自动加上 ".json"
                loadMapFromJson(map, loadPath);  // 调用加载函数
                loadSuccess = true;  // 假设加载成功，可以根据 loadMapFromJson 函数返回值来进一步优化
                saveSuccess = false;  // 重置保存状态
            }
        }

        ImGui::End();

    }

    void keyPressed(int key) override {
        
    }

    void mousePressed(int x, int y, int button) override {
        switch (currentMode) {
            case Add_Node: {
                // addVertex(38, 0.45f, 0.6f);
                break;
            }
            case Remove_Node: {
                // addVertex(38, 0.45f, 0.6f);
                break;
            }
            case Add_Edge: {
                // addEdge(DirectedWeightedEdge(1, 2, 13.0f));
                break;
            }
            case Remove_Edge: {
                // addEdge(DirectedWeightedEdge(1, 2, 13.0f));
                break;
            }
            case Add_Obstacle: {
                // collisionList.push_back(PolygonCollision(ofVec2f(0.53, 0.53), ofVec2f(0.53, 0.67), ofVec2f(0.67, 0.53)));
                break;
            }
            case Remove_Obstacle: {
                // collisionList.push_back(PolygonCollision(ofVec2f(0.53, 0.53), ofVec2f(0.53, 0.67), ofVec2f(0.67, 0.53)));
                break;
            }
            case Path_Finding: {

                ofVec2f targetPoint = pathFinder.GetPositionOnMouseClick(x, y);

                float normalizedX = static_cast<float>(targetPoint.x) / ofGetWidth();
                float normalizedY = static_cast<float>(targetPoint.y) / ofGetHeight();
                float normalizedCharacterX = static_cast<float>(mainCharacter.position.x) / ofGetWidth();
                float normalizedCharacterY = static_cast<float>(mainCharacter.position.y) / ofGetHeight();

                pathFinder.targetPosition.set(targetPoint.x, targetPoint.y);
                pathFinder.startNode = map.findNearestReachableNodeToClick(mainCharacter.position.x, mainCharacter.position.y);
                pathFinder.goalNode = map.findNearestReachableNodeToClick(targetPoint.x, targetPoint.y);
                pathFinder.currentNodeIndex = 0;
                pathFinder.nextNodeIndex = 1;

                auto result = graph.aStar(pathFinder.startNode, pathFinder.goalNode, manhattanDistance);
                auto came_from = result.first;
                pathFinder.path = graph.getPath(pathFinder.goalNode, came_from);
                isPathFinding = true;
                break;
            }
        }
    }
};