#pragma once
#include "Graph.h"
#include "Grid.h"

class PolygonCollision {
public:
    std::vector<ofVec2f> vertices; //clockwise

    PolygonCollision(const std::vector<ofVec2f>& vertices) : vertices(vertices) {}

    PolygonCollision(std::initializer_list<ofVec2f> verts) : vertices(verts) {}

    void drawPolygon() const {
        if (vertices.size() < 2) return;

        ofSetColor(ofColor::blue);

        for (size_t i = 0; i < vertices.size(); ++i) {
            ofVec2f currentVertex = vertices[i];
            ofVec2f nextVertex = vertices[(i + 1) % vertices.size()];

            ofDrawLine(ofPoint(currentVertex.x * ofGetWidth(), currentVertex.y * ofGetHeight()),
                ofPoint(nextVertex.x * ofGetWidth(), nextVertex.y * ofGetHeight()));
        }

        ofSetColor(ofColor::white);
    }
};
bool isPointInsidePolygon(const ofVec2f& point, const PolygonCollision& polygon) {
    bool inside = false;
    for (int i = 0, j = polygon.vertices.size() - 1; i < polygon.vertices.size(); j = i++) {
        if (((polygon.vertices[i].y * ofGetHeight() > point.y) != (polygon.vertices[j].y * ofGetHeight() > point.y)) &&
            (point.x < (polygon.vertices[j].x * ofGetWidth() - polygon.vertices[i].x * ofGetWidth()) * (point.y - polygon.vertices[i].y * ofGetHeight()) / (polygon.vertices[j].y * ofGetHeight() - polygon.vertices[i].y * ofGetHeight()) + polygon.vertices[i].x * ofGetWidth())) {
            inside = !inside;
        }
    }
    return inside;
}

bool rayIntersectsPolygon(const ofVec2f& rayOrigin, const ofVec2f& rayDirection, const PolygonCollision& polygon) {
    bool intersects = false;

    ofVec2f rayEnd = rayOrigin + rayDirection;

    for (size_t i = 0; i < polygon.vertices.size(); i++) {
        size_t j = (i + 1) % polygon.vertices.size();
        ofVec2f p1 = mapToScreenCoordinates(polygon.vertices[i]);

        ofVec2f p2 = mapToScreenCoordinates(polygon.vertices[j]);

        ofVec2f edge = p2 - p1;

        ofVec2f rayVec = rayEnd - rayOrigin;
        ofVec2f toStart = p1 - rayOrigin;

        float edgeCrossRayScalar = edge.x * rayDirection.y - edge.y * rayDirection.x;

        if (std::fabs(edgeCrossRayScalar) < 0.0001f) {
            continue;
        }

        float u = (toStart.x * rayDirection.y - toStart.y * rayDirection.x) / (-edgeCrossRayScalar);
        float t = (toStart.x * edge.y - toStart.y * edge.x) / (-edgeCrossRayScalar);

        if ((t > 0 && t < 1) && (u > 0 && u < 1)) {
            intersects = true;
            break;
        }
    }

    return intersects;
}

bool willCollide(const ofVec2f& playerPosition, const ofVec2f& targetPosition, const std::vector<PolygonCollision>& collisionList) {
    ofVec2f rayDirection = targetPosition - playerPosition;
    for (const auto& polygon : collisionList) {
        if (rayIntersectsPolygon(playerPosition, rayDirection, polygon)) {
            return true;
        }
    }
    return false;
}
class Map {
public:
	std::vector<PolygonCollision> collisionList;
	DirectedGraph* graph;
    Grid* grid;

    int findNearestReachableNodeToClick(int clickX, int clickY) {
        float nearestDistance = std::numeric_limits<float>::max();
        int nearestNodeId = 1;

        ofVec2f clickPosition(clickX, clickY);

        for (const auto& [id, node] : graph->vertices) {
            ofVec2f nodePosition = mapToScreenCoordinates(node.position);
            if (!willCollide(nodePosition, clickPosition, collisionList)) {
                float distance = clickPosition.distance(nodePosition);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestNodeId = id;
                    //std::cout << "nearest Node ID : " << id << "  Distance : "<< nearestDistance <<std::endl;
                }
            }
        }

        return nearestNodeId;
    }

    int getNodeAtPosition(int clickX, int clickY) {
        ofVec2f clickPosition(clickX, clickY);
        float selectionRadius = 10.0f;  // 定义一个选择半径，用于判断鼠标是否点击到节点

        for (const auto& [id, node] : graph->vertices) {
            ofVec2f nodePosition = mapToScreenCoordinates(node.position);
            float distance = clickPosition.distance(nodePosition);

            // 如果点击位置与节点中心的距离小于定义的选择半径值，则认为点击到了该节点
            if (distance <= selectionRadius) {
                return id;
            }
        }

        // 如果没有点击到任何节点，返回 -1 无效的 ID
        return -1;
    }
};