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
        // 如果和current node的下一个也没有碰撞，就以下一个node为target
        if (currentNodeIndex < path.size() - 1 && !willCollide(mapToScreenCoordinates(playerPosition), mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex + 1]].getPosition()), map->collisionList)) {
            currentNodeIndex++;
            nextNodeIndex++;
            movingTarget = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition());
            seekBehavior->setTargetPosition(movingTarget);
            seekBehavior->updatePosition(deltaTime);
        }
        // 如果抵达了这个current node,朝向下一个
        else if (currentNodeIndex < path.size() - 1 && playerPosition.distance(map->graph->vertices[path[currentNodeIndex]].getPosition()) < 0.02) {
            currentNodeIndex++;
            nextNodeIndex++;
            movingTarget = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition());
            seekBehavior->setTargetPosition(movingTarget);
            seekBehavior->updatePosition(deltaTime);
        }
        // 和下一个node之间有障碍物，就简单朝着当前的node移动
        else if (currentNodeIndex < path.size() - 1) {
            movingTarget = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition());
            seekBehavior->setTargetPosition(movingTarget);
            seekBehavior->updatePosition(deltaTime);
        }
        // 如果当前的目标已经是最后一个node， 看和真正的目标有没有碰撞，没有的话就直接向目标做最后的移动
        else if (currentNodeIndex == path.size() - 1 && !willCollide(mapToScreenCoordinates(playerPosition), targetPosition, map->collisionList)) {
            currentNodeIndex = path.size();
            movingTarget = targetPosition;
            arriveBehavior->setTargetPosition(movingTarget);
            arriveBehavior->updatePosition(deltaTime);
        }
        // 如果抵达了最后的node，朝向最终的目标移动
        else if (currentNodeIndex == path.size() - 1 && playerPosition.distance(map->graph->vertices[path[currentNodeIndex]].getPosition()) < 0.02 && currentNodeIndex == path.size() - 1) {
            currentNodeIndex = path.size();
            movingTarget = targetPosition;
            arriveBehavior->setTargetPosition(movingTarget);
            arriveBehavior->updatePosition(deltaTime);
        }
        // 和最终目标有障碍物，也没达到最后的node，简单朝向最后的node移动
        else if (currentNodeIndex == path.size() - 1) {
            movingTarget = mapToScreenCoordinates(map->graph->vertices[path[currentNodeIndex]].getPosition());
            seekBehavior->setTargetPosition(movingTarget);
            seekBehavior->updatePosition(deltaTime);
        }

    }

    void updatePositionUsingGrid(float deltaTime)
    {
    
    }
};