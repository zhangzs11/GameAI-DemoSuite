#pragma once
#include "Map.h"
#include "..\Objects\Character.h"
#include "..\Movements\Arrive.h"
#include "..\Movements\Seek.h"

class PathFinder {
public:
    Character* character;
    Map* map;

    std::vector<int> path;
    int startNode;
    int goalNode;
    int currentNodeIndex;
    int nextNodeIndex;

    ofVec2f playerPosition;
    ofVec2f targetPosition;

    std::shared_ptr<Arrive> arriveBehavior;
    std::shared_ptr<Seek> seekBehavior;

    PathFinder() {
    }

    ofVec2f GetPositionOnMouseClick(int mouseX, int mouseY) {
        ofVec2f clickPoint(mouseX, mouseY);
        for (const auto& polygon : map->collisionList) {
            while (isPointInsidePolygon(clickPoint, polygon)) {
                clickPoint.y -= 10.0;
                clickPoint.x -= 10.0;
            }
        }
        return clickPoint;
    }

    void updatePositionUsingGraph(float deltaTime)
    {
        ofVec2f movingTarget;
        playerPosition = character->position;
        // 检查currentNodeIndex和path范围是否有效
        if (currentNodeIndex < path.size() - 1) {
            ofVec2f nextNodePosition = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex + 1]].getPosition());

            // 检查和当前的目标node的下一个node是否有碰撞
            bool collisionDetected = false;
            for (const auto& polygon : map->collisionList) {
                if (willCollide(playerPosition, nextNodePosition, polygon)) {
                    collisionDetected = true;
                    break;
                }
            }

            if (!collisionDetected) {
                // 如果没有，目标移动到下一个节点
                currentNodeIndex++;
                nextNodeIndex++;
                movingTarget = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition());
                // seekBehavior->setTargetPosition(movingTarget);
                // seekBehavior->updatePosition(deltaTime);
                arriveBehavior->setTargetPosition(movingTarget);
                arriveBehavior->updatePosition(deltaTime);
            }
            else if (playerPosition.distance(mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition())) < 20.0f) {
                // 如果有，目标还是当前node，但是看是否抵达，如果抵达，移动到下一个节点
                currentNodeIndex++;
                nextNodeIndex++;
                movingTarget = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition());
                // seekBehavior->setTargetPosition(movingTarget);
                // seekBehavior->updatePosition(deltaTime);
                arriveBehavior->setTargetPosition(movingTarget);
                arriveBehavior->updatePosition(deltaTime);
            }
            else {
                // 和下一个node之间有障碍物，当前node也没抵达，就简单朝着当前的node移动
                movingTarget = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition());
                // seekBehavior->setTargetPosition(movingTarget);
                // seekBehavior->updatePosition(deltaTime);
                arriveBehavior->setTargetPosition(movingTarget);
                arriveBehavior->updatePosition(deltaTime);
            }
        }
        else if (currentNodeIndex == path.size() - 1) {
            // 和最终目标之间的障碍物检查，目前是朝向最后一个节点
            bool collisionDetected = false;
            for (const auto& polygon : map->collisionList) {
                if (willCollide(playerPosition, targetPosition, polygon)) {
                    collisionDetected = true;
                    break;
                }
            }

            if (!collisionDetected) {
                // 没有障碍物，直接朝向目标移动
                // currentNodeIndex = path.size();
                movingTarget = targetPosition;
                arriveBehavior->setTargetPosition(movingTarget);
                arriveBehavior->updatePosition(deltaTime);
            }
            else if (playerPosition.distance(mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition())) < 0.02) {
                // 有障碍物，但是已经抵达当前node，朝向最终目标
                currentNodeIndex = path.size();
                movingTarget = targetPosition;
                arriveBehavior->setTargetPosition(movingTarget);
                arriveBehavior->updatePosition(deltaTime);
            }
            else {
                // 有障碍物的情况下，移动到当前最后一个节点
                movingTarget = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition());
                // seekBehavior->setTargetPosition(movingTarget);
                // seekBehavior->updatePosition(deltaTime);
                arriveBehavior->setTargetPosition(movingTarget);
                arriveBehavior->updatePosition(deltaTime);
            }
        }
    }

    void updatePositionUsingGrid(float deltaTime)
    {
    
    }
};