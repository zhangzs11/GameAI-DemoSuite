#pragma once
#include <vector>
#include <algorithm>

#include "DynamicSteeringBehaviours.h"
#include "ofMain.h"
class Node {
public:
    int id;
    float x, y; //position

    Node() : id(0), x(0), y(0) {}
    Node(int id, float x, float y) : id(id), x(x), y(y) {}
    ofVec2f getPosition() { return ofVec2f(x, y); }
};

class PolygonCollision {
public:
    std::vector<ofVec2f> vertices; //clockwise

    PolygonCollision(const std::vector<ofVec2f>& vertices) : vertices(vertices) {}
    PolygonCollision(ofVec2f vertice1, ofVec2f vertice2, ofVec2f vertice3) {
        vertices.push_back(vertice1);
        vertices.push_back(vertice2);
        vertices.push_back(vertice3);
    }
    PolygonCollision(ofVec2f vertice1, ofVec2f vertice2, ofVec2f vertice3, ofVec2f vertice4) {
        vertices.push_back(vertice1);
        vertices.push_back(vertice2);
        vertices.push_back(vertice3);
        vertices.push_back(vertice4);
    }
    PolygonCollision(ofVec2f vertice1, ofVec2f vertice2, ofVec2f vertice3, ofVec2f vertice4, ofVec2f vertice5) {
        vertices.push_back(vertice1);
        vertices.push_back(vertice2);
        vertices.push_back(vertice3);
        vertices.push_back(vertice4);
        vertices.push_back(vertice5);
    }
    PolygonCollision(ofVec2f vertice1, ofVec2f vertice2, ofVec2f vertice3, ofVec2f vertice4, ofVec2f vertice5, ofVec2f vertice6) {
        vertices.push_back(vertice1);
        vertices.push_back(vertice2);
        vertices.push_back(vertice3);
        vertices.push_back(vertice4);
        vertices.push_back(vertice5);
        vertices.push_back(vertice6);
    }
    void drawPolygon() const {
        if (vertices.size() < 2) return;

        ofVec2f center(0, 0);
        for (const auto& vertex : vertices) {
            center += vertex;
        }
        center /= vertices.size();

        ofSetColor(ofColor::blue);

        float scale = 1.0f;

        for (size_t i = 0; i < vertices.size(); ++i) {
            ofVec2f currentVertex = vertices[i];
            ofVec2f nextVertex = vertices[(i + 1) % vertices.size()];

            currentVertex = center + (currentVertex - center) * scale;
            nextVertex = center + (nextVertex - center) * scale;

            ofDrawLine(ofPoint(currentVertex.x * ofGetWidth(), currentVertex.y * ofGetHeight()), ofPoint(nextVertex.x * ofGetWidth(), nextVertex.y * ofGetHeight()));
        }
        ofSetColor(ofColor::white);
    }

};
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

        float u = (toStart.x * rayDirection.y - toStart.y * rayDirection.x) / (- edgeCrossRayScalar);
        float t = (toStart.x * edge.y - toStart.y * edge.x) / ( - edgeCrossRayScalar);

        if ((t > 0 && t < 1) && (u > 0 && u < 1)) {
            intersects = true;
            /*std::cout << "P1 : " << p1 << std::endl;
            std::cout << "P2 : " << p2 << std::endl;
            std::cout << "edge : " << edge << std::endl;
            std::cout << "rayOrigin : " << rayOrigin << std::endl;
            std::cout << "rayEnd : " << rayEnd << std::endl;
            std::cout << "rayDirection : " << rayDirection << std::endl;
            std::cout << "toStart : " << toStart << std::endl;
            std::cout << "edgeCrossRayScalar : " << edgeCrossRayScalar << std::endl;
            std::cout << "u : " << u << std::endl;
            std::cout << "t : " << t << std::endl;*/
            break;
        }
    }

    return intersects;
}


bool willCollide(const int& nodeId, const ofVec2f& playerPosition, const ofVec2f& targetPosition, const std::vector<PolygonCollision>& collisionList) {
    ofVec2f rayDirection = targetPosition - playerPosition;
    for (const auto& polygon : collisionList) {
        if (rayIntersectsPolygon(playerPosition, rayDirection, polygon)) {
            //std::cout << "There is collision!  TargetID: " << nodeId << " ClickPosition: " << targetPosition << std::endl;
            return true;
        }
    }
    return false;
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
class DirectedWeightedEdge {
public:
    int source;
    int sink;
    float weight;

public:
    DirectedWeightedEdge(int source, int sink, float weight)
        : source(source), sink(sink), weight(weight) {}

    float getWeight() const {
        return weight;
    }

    int getSource() const {
        return source;
    }

    int getSink() const {
        return sink;
    }
};

class DirectedGraph {
public:
    std::vector<DirectedWeightedEdge> edges;
    std::unordered_map<int, Node> vertices;
    std::vector<PolygonCollision> collisionList;

    DirectedGraph() {
    }

public:
    void addVertex(int id, float x, float y) {
        vertices[id] = Node(id, x, y);
    }

    void addEdge(const DirectedWeightedEdge& edge) {
        edges.push_back(edge);
    }
    void addEdgeDouble(const DirectedWeightedEdge& edge) {
        edges.push_back(edge);
        DirectedWeightedEdge edge2(edge.getSink(), edge.getSource(), edge.getWeight());
        edges.push_back(edge2);
    }

    int findNearestReachableNodeToClick(int clickX, int clickY) {
        float nearestDistance = std::numeric_limits<float>::max();
        int nearestNodeId = 1;

        ofVec2f clickPosition(clickX, clickY);

        for (const auto& [id, node] : vertices) {
            ofVec2f nodePosition = mapToScreenCoordinates(ofVec2f(node.x, node.y));
            if (!willCollide(id, nodePosition, clickPosition, collisionList)) {
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

    std::vector<DirectedWeightedEdge> getOutgoingEdges(int source) const {
        std::vector<DirectedWeightedEdge> outgoingEdges;
        for (const auto& edge : edges) {
            if (edge.getSource() == source) {
                outgoingEdges.push_back(edge);
            }
        }
        return outgoingEdges;
    }

    float getCost(int source, int sink) const {
        for (const auto& edge : edges) {
            if (edge.getSource() == source && edge.getSink() == sink) {
                return edge.getWeight();
            }
        }
        return std::numeric_limits<float>::max();
    }
    // Generate a small graph
    void generateSmallGraph() {
        addVertex(1, 0.1f, 0.9f);
        addVertex(2, 0.1f, 0.7f);
        addVertex(3, 0.3f, 0.8f);
        addVertex(4, 0.3f, 0.9f);
        addVertex(5, 0.1f, 0.5f);
        addVertex(6, 0.5f, 0.9f);
        addVertex(7, 0.1f, 0.3f);
        addVertex(8, 0.1f, 0.1f);
        addVertex(9, 0.5f, 0.1f);
        addVertex(10, 0.8f, 0.1f);
        addVertex(11, 0.8f, 0.2f);
        addVertex(12, 0.9f, 0.1f);
        addVertex(13, 0.9f, 0.4f);
        addVertex(14, 0.9f, 0.7f);
        addVertex(15, 0.9f, 0.9f);
        addVertex(16, 0.7f, 0.9f);
        addVertex(17, 0.3f, 0.3f);
        addVertex(18, 0.4f, 0.3f);
        addVertex(19, 0.5f, 0.45f);
        addVertex(20, 0.6f, 0.5f);
        addVertex(21, 0.7f, 0.5f);
        addVertex(22, 0.8f, 0.8f);
        addVertex(23, 0.5f, 0.3f);
        addVertex(24, 0.45f, 0.5f);
        addVertex(25, 0.4f, 0.5f);
        addVertex(26, 0.35f, 0.5f);
        addVertex(27, 0.5f, 0.7f);
        addVertex(28, 0.5f, 0.2f);
        addVertex(29, 0.6f, 0.3f);
        addVertex(30, 0.4f, 0.2f);
        addVertex(31, 0.3f, 0.2f);
        addVertex(32, 0.2f, 0.2f);
        addVertex(33, 0.7f, 0.35f);
        addVertex(34, 0.8f, 0.5f);
        addVertex(35, 0.3f, 0.5f);
        addVertex(36, 0.2f, 0.3f);
        addVertex(37, 0.5f, 0.8f);
        addVertex(38, 0.45f, 0.6f);
        addEdge(DirectedWeightedEdge(1, 2, 13.0f));
        addEdge(DirectedWeightedEdge(1, 3, 3.0f));
        addEdge(DirectedWeightedEdge(1, 4, 3.0f));
        addEdge(DirectedWeightedEdge(2, 5, 3.0f));
        addEdge(DirectedWeightedEdge(3, 25, 3.0f));
        addEdge(DirectedWeightedEdge(4, 6, 3.0f));
        addEdge(DirectedWeightedEdge(5, 7, 3.0f));
        addEdge(DirectedWeightedEdge(5, 35, 3.0f));
        addEdge(DirectedWeightedEdge(6, 37, 3.0f));
        addEdge(DirectedWeightedEdge(6, 16, 3.0f));
        addEdge(DirectedWeightedEdge(7, 8, 3.0f));
        addEdge(DirectedWeightedEdge(7, 36, 3.0f));
        addEdge(DirectedWeightedEdge(8, 9, 3.0f));
        addEdge(DirectedWeightedEdge(8, 32, 3.0f));
        addEdge(DirectedWeightedEdge(9, 10, 30.0f));
        addEdge(DirectedWeightedEdge(10, 12, 3.0f));
        addEdge(DirectedWeightedEdge(11, 12, 3.0f));
        addEdge(DirectedWeightedEdge(13, 12, 3.0f));
        addEdge(DirectedWeightedEdge(14, 13, 3.0f));
        addEdge(DirectedWeightedEdge(15, 14, 3.0f));
        addEdge(DirectedWeightedEdge(15, 22, 3.0f));
        addEdge(DirectedWeightedEdge(16, 15, 3.0f));
        addEdge(DirectedWeightedEdge(17, 18, 3.0f));
        addEdge(DirectedWeightedEdge(18, 30, 3.0f));
        addEdge(DirectedWeightedEdge(18, 28, 6.0f));
        addEdge(DirectedWeightedEdge(18, 19, 3.0f));
        addEdge(DirectedWeightedEdge(19, 23, 3.0f));
        addEdge(DirectedWeightedEdge(20, 19, 3.0f));
        addEdge(DirectedWeightedEdge(20, 29, 3.0f));
        addEdge(DirectedWeightedEdge(21, 20, 3.0f));
        addEdge(DirectedWeightedEdge(21, 34, 3.0f));
        addEdge(DirectedWeightedEdge(22, 21, 3.0f));
        addEdge(DirectedWeightedEdge(23, 11, 3.0f));
        addEdge(DirectedWeightedEdge(23, 29, 3.0f));
        addEdge(DirectedWeightedEdge(24, 19, 3.0f));
        addEdge(DirectedWeightedEdge(25, 24, 3.0f));
        addEdge(DirectedWeightedEdge(25, 38, 3.0f));
        addEdge(DirectedWeightedEdge(25, 26, 3.0f));
        addEdge(DirectedWeightedEdge(26, 36, 3.0f));
        addEdge(DirectedWeightedEdge(26, 18, 50.0f));
        addEdge(DirectedWeightedEdge(26, 24, 5.0f));
        addEdge(DirectedWeightedEdge(27, 20, 3.0f));
        addEdge(DirectedWeightedEdge(27, 22, 3.0f));
        addEdge(DirectedWeightedEdge(28, 10, 10.0f));
        addEdge(DirectedWeightedEdge(28, 23, 3.0f));
        addEdge(DirectedWeightedEdge(29, 33, 3.0f));
        addEdge(DirectedWeightedEdge(30, 28, 3.0f));
        addEdge(DirectedWeightedEdge(31, 30, 3.0f));
        addEdge(DirectedWeightedEdge(32, 31, 3.0f));
        addEdge(DirectedWeightedEdge(32, 17, 3.0f));
        addEdge(DirectedWeightedEdge(33, 11, 3.0f));
        addEdge(DirectedWeightedEdge(33, 13, 3.0f));
        addEdge(DirectedWeightedEdge(34, 33, 3.0f));
        addEdge(DirectedWeightedEdge(35, 36, 3.0f));
        addEdge(DirectedWeightedEdge(36, 26, 3.0f));
        addEdge(DirectedWeightedEdge(37, 27, 3.0f));
        addEdge(DirectedWeightedEdge(38, 27, 3.0f));
    }

    // Generate a large random graph
    float calculateWeight(int source, int sink) {
        // Implement the weight calculation based on your vertices' positions
        float distance = std::sqrt(std::pow(vertices[source].x - vertices[sink].x, 2) + std::pow(vertices[source].y - vertices[sink].y, 2));
        return distance;
    }
    void generateLargeGraph(int numberOfVertices, int numberOfEdges, float maxDistance) {
        vertices.clear();
        edges.clear();
        srand(static_cast<unsigned int>(time(NULL))); // Seed the RNG

        // Generate all vertices with random positions
        for (int i = 1; i <= numberOfVertices; ++i) {
            float x = static_cast<float>(rand()) / RAND_MAX;
            float y = static_cast<float>(rand()) / RAND_MAX;
            addVertex(i, x, y);
        }

        // Ensure basic connectivity
        for (int i = 1; i < numberOfVertices; ++i) { // Create a path
            int source = i;
            int sink = i + 1;
            float weight = calculateWeight(source, sink); // Implement this based on your graph structure
            addEdge(DirectedWeightedEdge(source, sink, weight));
        }

        // Step 2: Add additional edges randomly
        int edgesAdded = numberOfVertices - 1;
        while (edgesAdded < numberOfEdges) {
            int source = rand() % numberOfVertices + 1;
            int sink = rand() % numberOfVertices + 1;
            if (source == sink) continue; // Skip self-loops

            float weight = calculateWeight(source, sink); // Implement this based on your graph structure
            if (weight <= maxDistance) {
                addEdge(DirectedWeightedEdge(source, sink, weight));
                edgesAdded++;
            }
        }
    }

    // Generate an indoor environment graph
    void generateIndoorEnvironmentGraph() {
        // Clear existing vertices and edges
        vertices.clear();
        edges.clear();

        //addVertex(i, x, y);
        addVertex(1, 0.5, 0.5);
        addVertex(2, 0.5, 0.25);
        addVertex(3, 0.7, 0.5);
        addVertex(4, 0.5, 0.7);
        addVertex(5, 0.4, 0.5);
        addVertex(6, 0.75, 0.5);
        addVertex(7, 0.775, 0.4);
        addVertex(8, 0.8, 0.3);
        addVertex(9, 0.825, 0.4);
        addVertex(10, 0.85, 0.5);
        addVertex(11, 0.9, 0.3);
        addVertex(12, 0.9, 0.1);
        addVertex(13, 0.9, 0.8);
        addVertex(14, 0.7, 0.1);
        addVertex(15, 0.7, 0.9);
        addVertex(16, 0.4, 0.9);
        addVertex(17, 0.4, 0.1);
        addVertex(18, 0.35, 0.36);
        addVertex(19, 0.3, 0.5);
        addVertex(20, 0.05, 0.5);
        addVertex(21, 0.3, 0.9);
        addVertex(22, 0.2, 0.6);
        addVertex(23, 0.2, 0.9);
        addVertex(24, 0.1, 0.8);
        addVertex(25, 0.2, 0.36);
        addVertex(26, 0.2, 0.15);
        addVertex(27, 0.12, 0.15);
        addVertex(28, 0.1, 0.9);
        addVertex(29, 0.05, 0.6);
        addEdgeDouble(DirectedWeightedEdge(1, 2, 5));
        addEdgeDouble(DirectedWeightedEdge(1, 3, 1));
        addEdgeDouble(DirectedWeightedEdge(1, 4, 1));
        addEdgeDouble(DirectedWeightedEdge(1, 5, 1));
        addEdgeDouble(DirectedWeightedEdge(2, 14, 1));
        addEdgeDouble(DirectedWeightedEdge(2, 17, 1));
        addEdgeDouble(DirectedWeightedEdge(2, 18, 1));
        addEdgeDouble(DirectedWeightedEdge(3, 14, 1));
        addEdgeDouble(DirectedWeightedEdge(3, 6, 1));
        addEdgeDouble(DirectedWeightedEdge(4, 16, 1));
        addEdgeDouble(DirectedWeightedEdge(5, 19, 1));
        addEdgeDouble(DirectedWeightedEdge(6, 7, 1));
        addEdgeDouble(DirectedWeightedEdge(6, 15, 1));
        addEdgeDouble(DirectedWeightedEdge(7, 8, 1));
        addEdgeDouble(DirectedWeightedEdge(8, 9, 1));
        addEdgeDouble(DirectedWeightedEdge(9, 10, 1));
        addEdgeDouble(DirectedWeightedEdge(10, 11, 1));
        addEdgeDouble(DirectedWeightedEdge(11, 12, 1));
        addEdgeDouble(DirectedWeightedEdge(11, 13, 1));
        addEdgeDouble(DirectedWeightedEdge(12, 14, 1));
        //addEdgeDouble(DirectedWeightedEdge(13, 15, 1));
        addEdgeDouble(DirectedWeightedEdge(14, 17, 1));
        addEdgeDouble(DirectedWeightedEdge(15, 16, 1));
        addEdgeDouble(DirectedWeightedEdge(17, 18, 1));
        addEdgeDouble(DirectedWeightedEdge(18, 19, 1));
        addEdgeDouble(DirectedWeightedEdge(18, 25, 1));
        addEdgeDouble(DirectedWeightedEdge(19, 20, 1));
        addEdgeDouble(DirectedWeightedEdge(19, 21, 1));
        //addEdgeDouble(DirectedWeightedEdge(20, 22, 1));
        addEdgeDouble(DirectedWeightedEdge(20, 25, 1));
        addEdgeDouble(DirectedWeightedEdge(22, 23, 1));
        //addEdgeDouble(DirectedWeightedEdge(23, 24, 1));
        addEdgeDouble(DirectedWeightedEdge(25, 26, 1));
        addEdgeDouble(DirectedWeightedEdge(26, 27, 1));
        addEdgeDouble(DirectedWeightedEdge(28, 24, 1));
        addEdgeDouble(DirectedWeightedEdge(23, 28, 1));
        addEdgeDouble(DirectedWeightedEdge(29, 20, 1));
        addEdgeDouble(DirectedWeightedEdge(29, 22, 1));
    }

    // Dijkstra's Algorithm
    std::pair<std::unordered_map<int, int>, std::unordered_map<int, float>> dijkstra(int start, int goal) const {
        std::unordered_map<int, int> came_from;
        std::unordered_map<int, float> cost_so_far;
        std::set<std::pair<float, int>> frontier;

        //number of nodes visited
        int nodeNum = 0;

        came_from[start] = -1;
        cost_so_far[start] = 0;
        frontier.insert({ 0, start });

        while (!frontier.empty()) {
            int current = frontier.begin()->second;
            frontier.erase(frontier.begin());

            if (current == goal) {
                break;
            }

            for (const auto& edge : getOutgoingEdges(current)) {
                int next = edge.getSink();
                float new_cost = cost_so_far[current] + getCost(current, next);

                if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                    nodeNum++;

                    cost_so_far[next] = new_cost;
                    float priority = new_cost;
                    frontier.erase({ cost_so_far[next], next });
                    frontier.insert({ priority, next });
                    came_from[next] = current;
                }
            }
        }
        //std::cout << "Dijkstra Algorithm  number of nodes visited : " << nodeNum << std::endl;
        return { came_from, cost_so_far };
    }
    // Heuristics 
    inline float manhattanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
        auto& currentVertex = vertices.at(current);
        auto& goalVertex = vertices.at(goal);
        return std::abs(currentVertex.x * ofGetWidth() - goalVertex.x * ofGetWidth()) + std::abs(currentVertex.y * ofGetHeight() - goalVertex.y * ofGetHeight());
    }

    inline float euclideanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
        auto& currentVertex = vertices.at(current);
        auto& goalVertex = vertices.at(goal);
        return std::sqrt(std::pow(currentVertex.x * ofGetWidth() - goalVertex.x * ofGetWidth(), 2) + std::pow(currentVertex.y * ofGetHeight() - goalVertex.y * ofGetHeight(), 2));
    }

    // A*
    std::pair<std::unordered_map<int, int>, std::unordered_map<int, float>> aStar(int start, int goal, std::function<float(int, int, const std::unordered_map<int, Node>&)> heuristic) const {
        std::unordered_map<int, int> came_from;
        std::unordered_map<int, float> cost_so_far;
        std::set<std::pair<float, int>> frontier;

        //number of nodes visited
        int nodeNum = 0;

        came_from[start] = -1;
        cost_so_far[start] = 0;
        frontier.insert({ 0, start });

        while (!frontier.empty()) {
            int current = frontier.begin()->second;
            frontier.erase(frontier.begin());

            if (current == goal) break;

            for (const auto& edge : getOutgoingEdges(current)) {
                int next = edge.getSink();
                float new_cost = cost_so_far[current] + getCost(current, next);
                float priority = new_cost + heuristic(next, goal, vertices);

                if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    frontier.erase({ cost_so_far[next] + heuristic(next, goal, vertices), next });
                    frontier.insert({ priority, next });
                    came_from[next] = current;
                    nodeNum++;
                }
            }
        }

        //std::cout << "A star Algorithm  number of nodes visited : " << nodeNum << std::endl;
        return { came_from, cost_so_far };
    }

    std::vector<int> getPath(int goal, std::unordered_map<int, int> came_from) {
        std::vector<int> path;
        for (int current = goal; current != -1; current = came_from[current]) {
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    void drawMap() const {
        ofSetLineWidth(2);
        ofNoFill();
        ofSetColor(ofColor::white);
        for (const auto& edge : edges) {
            Node sourceVertex = vertices.at(edge.getSource());
            Node sinkVertex = vertices.at(edge.getSink());

            float sourceX = sourceVertex.x * ofGetWidth();
            float sourceY = sourceVertex.y * ofGetHeight();
            float sinkX = sinkVertex.x * ofGetWidth();
            float sinkY = sinkVertex.y * ofGetHeight();

            ofDrawArrow(ofPoint(sourceX, sourceY), ofPoint(sinkX, sinkY), 5.0f);
        }

        for (const auto& vertex : vertices) {
            float x = vertex.second.x * ofGetWidth();
            float y = vertex.second.y * ofGetHeight();

            ofDrawCircle(x, y, 15);
            ofDrawBitmapString(vertex.first, x+10, y+10);
        }
    }

    void drawPath(std::vector<int> path) const {
        ofSetColor(ofColor::red);
        for (size_t i = 0; i < path.size() - 1; ++i) {
            int source = path[i];
            int sink = path[i + 1];
            Node sourceVertex = vertices.at(source);
            Node sinkVertex = vertices.at(sink);

            float sourceX = sourceVertex.x * ofGetWidth();
            float sourceY = sourceVertex.y * ofGetHeight();
            float sinkX = sinkVertex.x * ofGetWidth();
            float sinkY = sinkVertex.y * ofGetHeight();

            ofDrawLine(ofPoint(sourceX, sourceY), ofPoint(sinkX, sinkY));
        }
        ofSetColor(ofColor::white);
    }
};

class PathFinder {
public:
    Character* character;

    DirectedGraph graph;
    std::vector<PolygonCollision> collisionList;
    std::vector<int> path;
    int startNode;
    int goalNode;
    int currentNodeIndex;
    int nextNodeIndex;

    ofVec2f playerPosition;
    ofVec2f targetPosition;


    float targetRadius = 10;
    float slowRadius = 50;
    float timeToTarget = 0.5;

    PathFinder() {
    }

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
    ofVec2f findNearestOutsidePoint(const ofVec2f& insidePoint, const PolygonCollision& polygon) {
        ofVec2f testPoint = insidePoint;
        //std::cout << "Inside!" << std::endl;
        while (isPointInsidePolygon(testPoint, polygon)) {
            //std::cout << "Inside!" << std::endl;
            testPoint.y -= 10.0;
            testPoint.x -= 10.0;
        }
        return testPoint;
    }
    ofVec2f onMouseClick(int mouseX, int mouseY) {
        ofVec2f clickPoint(mouseX, mouseY);
        for (const auto& polygon : collisionList) {
            if (isPointInsidePolygon(clickPoint, polygon)) {
                return findNearestOutsidePoint(clickPoint, polygon);
            }
        }
        return clickPoint;
    }

    SteeringOutput getSteeringToTarget(const ofVec2f& target) {

        SteeringOutput steering;
        ofVec2f direction = target - character->position;
        float distance = direction.length();
        if (distance < targetRadius) {
            return steering;//do not need any behaviour
        }

        float targetSpeed;
        if (distance > slowRadius) {
            targetSpeed = character->maxSpeed;
        }
        else {
            targetSpeed = character->maxSpeed * distance / slowRadius;//smooth stop
        }

        ofVec2f targetVelocity = direction.getNormalized() * targetSpeed;
        steering.linear = (targetVelocity - character->velocity) / timeToTarget;
        if (steering.linear.length() > character->maxAcceleration) {
            steering.linear.normalize();
            steering.linear *= character->maxAcceleration;
        }

        return steering;
    }

    SteeringOutput getSteering() {
        SteeringOutput steering;
        if (currentNodeIndex < path.size() - 1 && !willCollide(mapToScreenCoordinates(playerPosition), mapToScreenCoordinates(graph.vertices[path[currentNodeIndex +1]].getPosition()), collisionList)) {
            currentNodeIndex++;
            nextNodeIndex++;
            steering = getSteeringToTarget(mapToScreenCoordinates(graph.vertices[path[currentNodeIndex]].getPosition()));
        }
        if (currentNodeIndex < path.size() - 1) {
            steering = getSteeringToTarget(mapToScreenCoordinates(graph.vertices[path[currentNodeIndex]].getPosition()));
        }
        if (currentNodeIndex < path.size() - 1 && playerPosition.distance(graph.vertices[path[currentNodeIndex]].getPosition()) < 0.02) {
            currentNodeIndex++;
            nextNodeIndex++;
            steering = getSteeringToTarget(mapToScreenCoordinates(graph.vertices[path[currentNodeIndex]].getPosition()));
        }
        else if (currentNodeIndex == path.size() - 1 && !willCollide(mapToScreenCoordinates(playerPosition), targetPosition, collisionList)) {
            currentNodeIndex = path.size();
            steering = getSteeringToTarget(targetPosition);
        }
        else if (currentNodeIndex == path.size() - 1 && playerPosition.distance(graph.vertices[path[currentNodeIndex]].getPosition()) < 0.02 && currentNodeIndex == path.size() - 1) {
            currentNodeIndex = path.size();
            steering = getSteeringToTarget(targetPosition);
        }
        else if (currentNodeIndex == path.size() - 1) {
            steering = getSteeringToTarget(mapToScreenCoordinates(graph.vertices[path[currentNodeIndex]].getPosition()));
        }
        else if (currentNodeIndex == path.size()) {
            steering = getSteeringToTarget(targetPosition);
        }

        return steering;
    }

    void update(const SteeringOutput& steering, float deltaTime) {
        character->velocity += steering.linear * deltaTime;
        if (character->velocity.length() > character->maxSpeed) {
            character->velocity.normalize();
            character->velocity *= character->maxSpeed;
        }
        character->position += character->velocity * deltaTime;
        if (character->velocity.lengthSquared() > 0) {
            character->orientation = atan2(character->velocity.y, character->velocity.x);
        }

        float normalizedX = static_cast<float>(character->position.x) / ofGetWidth();
        float normalizedY = static_cast<float>(character->position.y) / ofGetHeight();
        ofVec2f normalizedPosition(normalizedX, normalizedY);
        playerPosition = normalizedPosition;
    }
};

