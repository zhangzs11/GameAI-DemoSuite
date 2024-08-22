#pragma once
#include "DemoBase.h"
#include "PathFinder.h"

enum ModeType {
    Revise_Node,
    Revise_Edge,
    Revise_Obstacle,
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
        pathFinder.updatePositionUsingGraph(deltaTime);
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
        
    }

    void keyPressed(int key) override {
        
    }

    void mousePressed(int x, int y, int button) override {
        switch (currentMode) {
            case Revise_Node: {
                // if(add)
                // addVertex(38, 0.45f, 0.6f);
                break;
            }
            
            case Revise_Edge: {
                // if(add)
                // addEdge(DirectedWeightedEdge(1, 2, 13.0f));
                break;
            }

            case Revise_Obstacle: {
                // if(add)
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
                //isPathFinding = true;
                break;
            }
        }
    }
};