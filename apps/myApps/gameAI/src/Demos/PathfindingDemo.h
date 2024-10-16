#pragma once

#include "DemoBase.h"

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
    //std::shared_ptr<Seek> seekBehavior;
    PathFinder pathFinder;

    std::vector<ofVec2f> newObstacleVertices;

    int currentMode;

    // Draw mode
    bool ifDrawNode = true;

    void setup() override {
        // Init character
        mainCharacter.position = ofVec2f(0.5, 0.5);
        mainCharacter.color = ofColor::blue;
        mainCharacter.radius = 5;
        mainCharacter.isDead = false;
        isPathFinding = false;
        // Init map
        map.graph = &graph;

        // Init Arrive and Seek
        arriveBehavior = std::make_shared<Arrive>(&mainCharacter, 100.0f, 50.0f, 50.0f, 10.0f);
        //seekBehavior = std::make_shared<Seek>(&mainCharacter, 100.0f, 50.0f, mainCharacter.position);

        // Init pathfinder
        pathFinder.character = &mainCharacter;
        pathFinder.map = &map;
        pathFinder.arriveBehavior = arriveBehavior;
        //pathFinder.seekBehavior = seekBehavior;
    }

    void update(float deltaTime) override {
        if (isPathFinding) {
            pathFinder.updatePositionUsingGraph(deltaTime);
        }
        
    }

    void draw() override {
        if (ifDrawNode) {
            graph.drawMap();
        }
        graph.drawPath(pathFinder.path);
        for (const auto& polygon : map.collisionList) {
            polygon.drawPolygon();
        }
        if (currentMode == 4) {
            mainCharacter.draw();
        }
        for (ofVec2f vertexPosition : newObstacleVertices) {
            ofSetColor(ofColor::azure);
            ofDrawCircle(vertexPosition.x * ofGetWidth(), vertexPosition.y * ofGetHeight(), 2);
        }
        renderImGui();
    }

    void renderImGui() {
        ImGui::Begin("Path finding");

        // 选择操作mode
        const char* modes[] = { "Graph Editing", "Obstacle Editing", "Pathfinding Settings", "Save/Load Map", "start pathFinding!"};
        //static int currentMode = 0;

        ImGui::Combo("Edit Mode", &currentMode, modes, IM_ARRAYSIZE(modes));

        static bool ifDraw = false;
        ImGui::Checkbox("if Draw Node", &ifDraw);
        if (ifDraw) {
            ifDrawNode = true;
        }
        else {
            ifDrawNode = false;
        }

        if (currentMode == 0 && ImGui::CollapsingHeader("Graph Editing")) {
            // 添加节点
            static int newNodeId = 0;  // 初始节点ID
            static float newNodePosition[2] = { 0.5f, 0.5f };  // 初始位置

            ImGui::InputInt("New Node ID", &newNodeId);  // 输入新节点ID
            ImGui::Text("New Node Position");
            ImGui::Text("X: ");  // 显示 "X:"
            ImGui::SameLine();  // 在同一行显示
            ImGui::SliderFloat("##X", &newNodePosition[0], 0.0f, 1.0f);  // X 坐标的滑块
            ImGui::Text("Y: ");  // 显示 "Y:"
            ImGui::SameLine();  // 在同一行显示
            ImGui::SliderFloat("##Y", &newNodePosition[1], 0.0f, 1.0f);  // Y 坐标的滑块

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
            //static std::vector<ofVec2f> newObstacleVertices;
            static float vertexPos[2] = { 0.0f, 0.0f };
            static int polygonId = 0;

            ImGui::InputInt("Polygon ID", &polygonId);  // 手动输入多边形ID
            ImGui::Text("New Vertex Position");
            ImGui::Text("X: ");  // 显示 "X:"
            ImGui::SameLine();  // 在同一行显示
            ImGui::SliderFloat("##X", &vertexPos[0], 0.0f, 1.0f);  // X 坐标的滑块

            ImGui::Text("Y: ");  // 显示 "Y:"
            ImGui::SameLine();  // 在同一行显示
            ImGui::SliderFloat("##Y", &vertexPos[1], 0.0f, 1.0f);  // Y 坐标的滑块
            if (ImGui::Button("Add Vertex")) {
                newObstacleVertices.push_back(ofVec2f(vertexPos[0], vertexPos[1]));
            }
            if (ImGui::Button("Undo adding Vertex")) {
                if(!newObstacleVertices.empty())
                    newObstacleVertices.pop_back();
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

            // 自动生成节点的设置
            static float nodeDensity = 1.0f;
            static float maxDistanceFactor = 1.0f;
            static int maxDepth = 5.0f;
            static float minSize = 50.0f;
            ImGui::Text("Node Generation Settings:");
            // ImGui::SliderFloat("Node Density", &nodeDensity, 1.0f, 100.0f);  // 控制节点密度
            ImGui::SliderFloat("Max Distance Factor", &maxDistanceFactor, 0.01f, 3.0f);  // 控制最大连接距离系数
            ImGui::SliderInt("Max Recurse Depth", &maxDepth, 1, 20); // 最大递归深度
            ImGui::SliderFloat("min deveide size", &minSize, 10.0f, 100.0f); // 最小的递归矩形宽高（像素坐标）

            if (ImGui::Button("Auto Generate Nodes")) {
                map.graph->clearEdgeAndVertice();
                // map.generateNodes(nodeDensity, maxDistanceFactor);
                map.generateNodesRecursively(ofRectangle(0.0f, 0.0f, 1.0f, 1.0f), maxDepth, minSize, maxDistanceFactor);
            }
            if (ImGui::Button("Clear All Nodes")) {
                map.graph->clearEdgeAndVertice();  // 清空所有节点
                isPathFinding = false; // node清空了，停止寻路
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
            static char fileName[128] = "pathfinding_testmap1";  // 默认文件名（不包含后缀）
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

        // pathfinding展示
        if (currentMode == 4 && ImGui::CollapsingHeader("path finding")) {
            float maxForce = arriveBehavior->getMaxForce();
            if (ImGui::SliderFloat("Max Acceleration", &maxForce, 0.0f, 10000000000000000.0f)) {
                arriveBehavior->setMaxForce(maxForce);
                //seekBehavior->setMaxForce(maxForce);
            }

            float maxSpeed = arriveBehavior->getMaxSpeed();
            if (ImGui::SliderFloat("Max Speed", &maxSpeed, 0.0f, 500.0f)) {
                arriveBehavior->setMaxSpeed(maxSpeed);
                //->setMaxSpeed(maxSpeed);
            }

            float slowingRadius = arriveBehavior->getSlowingRadius();
            if (ImGui::SliderFloat("Slowing Radius", &slowingRadius, 0.0f, 1000.0f)) {
                arriveBehavior->setSlowingRadius(slowingRadius);
            }

            float stopRadius = arriveBehavior->getStopRadius();
            if (ImGui::SliderFloat("Stop Radius", &stopRadius, 0.0f, 500.0f)) {
                arriveBehavior->setStopRadius(stopRadius);
            }
        }

        ImGui::End();

    }

    void keyPressed(int key) override {
        
    }

    void mousePressed(int x, int y, int button) override {
        if (currentMode == 4) {
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
        }
    }
};