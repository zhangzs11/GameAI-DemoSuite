#pragma once
#include "Graph.h"
#include "Grid.h"

class PolygonCollision {
public:
    int id;
    std::vector<ofVec2f> vertices; //clockwise

    PolygonCollision(int id, const std::vector<ofVec2f>& vertices) : id(id), vertices(vertices) {}

    PolygonCollision(int id, std::initializer_list<ofVec2f> verts) : id(id), vertices(verts) {}

    void drawPolygon() const {
        if (vertices.size() < 2) return;

        ofSetColor(ofColor::blue);

        ofPath path;  // 创建一个 ofPath 对象
        path.setFillColor(ofColor::blue);  // 设置填充颜色
        for (size_t i = 0; i < vertices.size(); ++i) {
            ofVec2f currentVertex = vertices[i];
            if (i == 0) {
                path.moveTo(currentVertex.x * ofGetWidth(), currentVertex.y * ofGetHeight());  // 移动到第一个顶点
            }
            else {
                path.lineTo(currentVertex.x * ofGetWidth(), currentVertex.y * ofGetHeight());  // 连线到后续的每个顶点
            }
        }
        path.close();  // 闭合路径
        path.draw();   // 绘制实心多边形

        ofSetColor(ofColor::white);
    }
};
//bool isPointInsidePolygon(const ofVec2f& point, const PolygonCollision& polygon) {
//    bool inside = false;
//    for (int i = 0, j = polygon.vertices.size() - 1; i < polygon.vertices.size(); j = i++) {
//        if (((polygon.vertices[i].y * ofGetHeight() > point.y) != (polygon.vertices[j].y * ofGetHeight() > point.y)) &&
//            (point.x < (polygon.vertices[j].x * ofGetWidth() - polygon.vertices[i].x * ofGetWidth()) * (point.y - polygon.vertices[i].y * ofGetHeight()) / (polygon.vertices[j].y * ofGetHeight() - polygon.vertices[i].y * ofGetHeight()) + polygon.vertices[i].x * ofGetWidth())) {
//            inside = !inside;
//        }
//    }
//    return inside;
//}

bool isPointInsidePolygon(const ofVec2f& point, const PolygonCollision& polygon) {
    bool inside = false;
    float px = point.x * ofGetWidth();  // 点的像素坐标
    float py = point.y * ofGetHeight(); // 点的像素坐标

    for (int i = 0, j = polygon.vertices.size() - 1; i < polygon.vertices.size(); j = i++) {
        // 获取多边形的边的两个顶点的像素坐标
        float xi = polygon.vertices[i].x * ofGetWidth();
        float yi = polygon.vertices[i].y * ofGetHeight();
        float xj = polygon.vertices[j].x * ofGetWidth();
        float yj = polygon.vertices[j].y * ofGetHeight();

        // 检查射线是否穿过多边形的边
        bool intersect = ((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi);

        if (intersect) {
            inside = !inside;  // 交点数为奇数时在内部
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

bool willCollide(const ofVec2f& playerPosition, const ofVec2f& targetPosition, const PolygonCollision& polygon) {
    ofVec2f rayDirection = targetPosition - playerPosition;
    return rayIntersectsPolygon(playerPosition, rayDirection, polygon);
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

            bool collisionDetected = false;
            for (const auto& polygon : collisionList) {
                if (willCollide(nodePosition, clickPosition, polygon)) {
                    collisionDetected = true;
                    break;
                }
            }
            if (!collisionDetected) {
                float distance = clickPosition.distance(nodePosition);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestNodeId = id;
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

    // 添加多边形
    void addPolygon(int id, const std::vector<ofVec2f>& vertices) {
        if (vertices.size() < 3) {
            std::cout << "A polygon must have at least 3 vertices." << std::endl;
            return;
        }

        // Check ID
        for (const auto& polygon : collisionList) {
            if (polygon.id == id) {
                std::cout << "Polygon ID already exists!" << std::endl;
                return;
            }
        }

        PolygonCollision newPolygon(id, vertices);
        collisionList.push_back(newPolygon);
    }

    // 删除多边形障碍物
    void removePolygon(int id) {
        collisionList.erase(
            std::remove_if(collisionList.begin(), collisionList.end(),
                [id](const PolygonCollision& polygon) { return polygon.id == id; }),
            collisionList.end()
        );
    }

    // 自动生成节点函数
    void generateNodes(float density, float maxDistanceFactor) {  // 密度和最大距离系数作为参数
        float step = 1.0f / density;  // 根据密度计算步长
        float maxConnectionDistance = step * maxDistanceFactor;  // 最大连接距离由步长和系数决定

        // 遍历整个地图区域（假设归一化坐标 [0, 1]）
        for (float x = 0.0f; x <= 1.0f; x += step) {
            for (float y = 0.0f; y <= 1.0f; y += step) {
                ofVec2f potentialNode(x, y);  // 生成候选节点位置
                bool isCollision = false;

                // 检查该位置是否与任何多边形相交
                for (const auto& polygon : collisionList) {
                    if (isPointInsidePolygon(potentialNode, polygon)) {
                        isCollision = true;  // 发生碰撞
                        break;
                    }
                }

                // 如果没有与任何障碍物碰撞，添加该节点
                if (!isCollision) {
                    int newId = graph->vertices.size();  // 自动生成节点ID
                    graph->addVertex(newId, x, y);

                    // 检查与相邻节点的连接并添加边
                    for (const auto& [id, node] : graph->vertices) {
                        if (id != newId) {
                            float distance = potentialNode.distance(node.position);
                            if (distance <= maxConnectionDistance) {  // 使用基于step的最大距离

                                // 检查两个节点之间是否有障碍物
                                bool pathBlocked = false;
                                for (const auto& polygon : collisionList) {
                                    if (willCollide(mapToScreenCoordinates(potentialNode), mapToScreenCoordinates(node.position), polygon)) {
                                        pathBlocked = true;
                                        break;
                                    }
                                }

                                // 如果路径没有被障碍物阻挡，添加双向边
                                if (!pathBlocked) {
                                    float weight = distance;  // 使用距离作为权重
                                    graph->addEdgeDouble(newId, id, weight);  // 添加双向边
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // ----------------------------------
    // 优化的node自动生成
    //
    
    // 判断区域是否完全在多边形内
    bool isRegionInsidePolygon(const ofRectangle& region, const PolygonCollision& polygon) {
        return isPointInsidePolygon(region.getTopLeft(), polygon) &&
            isPointInsidePolygon(region.getTopRight(), polygon) &&
            isPointInsidePolygon(region.getBottomLeft(), polygon) &&
            isPointInsidePolygon(region.getBottomRight(), polygon);
    }

    // 判断区域是否与多边形相交
    bool doesRegionIntersectPolygon(const ofRectangle& region, const PolygonCollision& polygon) {
        ofRectangle screenRegion(
            region.getX() * ofGetWidth(),
            region.getY() * ofGetHeight(),
            region.getWidth() * ofGetWidth(),
            region.getHeight() * ofGetHeight()
        );
        for (const auto& vertex : polygon.vertices) {
            if (screenRegion.inside(vertex.x * ofGetWidth(), vertex.y * ofGetHeight())) {
                return true;
            }
        }
        return false;
    }

    // 尝试连接到现有节点
    void connectToExistingNodes(int newNodeId, float maxDistanceFactor) {
        const ofVec2f& newNodePosition = graph->vertices[newNodeId].position;
        float maxConnectionDistance = maxDistanceFactor;  // 基于step的最大距离

        for (const auto& [id, node] : graph->vertices) {
            if (id != newNodeId) {
                float distance = newNodePosition.distance(node.position);
                bool pathBlocked = false;
                for (const auto& polygon : collisionList) {
                    if (willCollide(mapToScreenCoordinates(newNodePosition), mapToScreenCoordinates(node.position), polygon)) {
                        pathBlocked = true;
                        break;
                    }
                }

                if (!pathBlocked && distance < maxConnectionDistance) {
                    float weight = distance;  // 使用距离作为权重
                    graph->addEdgeDouble(newNodeId, id, weight);  // 添加双向边
                }
            }
        }
    }

    void generateNodesRecursively(const ofRectangle& region, int maxDepth, float minSize, float maxDistanceFactor) {
        // 基本终止条件：如果递归深度达到上限或区域太小
        if (maxDepth <= 0 || region.getWidth() * ofGetWidth() <= minSize || region.getHeight() * ofGetHeight() <= minSize) {
            return;
        }

        // 检查区域与障碍物的关系
        bool hasCollision = false;
        bool fullyCovered = true;

        for (const auto& polygon : collisionList) {
            if (isRegionInsidePolygon(region, polygon)) {
                // 区域完全被障碍物覆盖
                return;
            }
            if (doesRegionIntersectPolygon(region, polygon)) {
                hasCollision = true;  // 区域与障碍物相交或部分覆盖
                fullyCovered = false;
            }
        }

        // 如果没有障碍物，直接在区域中心生成一个节点
        if (!hasCollision) {
            ofVec2f center = region.getCenter();
            int newId = graph->vertices.size();  // 生成新的节点ID
            graph->addVertex(newId, center.x, center.y);

            // 尝试连接到现有节点
            connectToExistingNodes(newId, maxDistanceFactor);
            return;
        }

        // 如果部分有障碍物，则继续划分该区域
        float halfWidth = region.getWidth() / 2.0f;
        float halfHeight = region.getHeight() / 2.0f;

        // 递归处理四个子区域
        generateNodesRecursively(ofRectangle(region.getX(), region.getY(), halfWidth, halfHeight), maxDepth - 1, minSize, maxDistanceFactor);  // 左上
        generateNodesRecursively(ofRectangle(region.getX() + halfWidth, region.getY(), halfWidth, halfHeight), maxDepth - 1, minSize, maxDistanceFactor);  // 右上
        generateNodesRecursively(ofRectangle(region.getX(), region.getY() + halfHeight, halfWidth, halfHeight), maxDepth - 1, minSize, maxDistanceFactor);  // 左下
        generateNodesRecursively(ofRectangle(region.getX() + halfWidth, region.getY() + halfHeight, halfWidth, halfHeight), maxDepth - 1, minSize, maxDistanceFactor);  // 右下
    }
};