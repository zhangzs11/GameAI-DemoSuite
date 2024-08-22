#pragma once
#include "../Utils.h"
class Node {
public:
    int id;
    ofVec2f position;

    Node() : id(0), position(ofVec2f(0.0f, 0.0f)){}
    Node(int id, ofVec2f position) : id(id), position(position){}
    ofVec2f getPosition() { return position; }
};

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

    DirectedGraph() {
    }

public:
    void addVertex(int id, float x, float y) {
        vertices[id] = Node(id, ofVec2f(x, y));
    }

    void addEdge(const DirectedWeightedEdge& edge) {
        edges.push_back(edge);
    }
    void addEdgeDouble(const DirectedWeightedEdge& edge) {
        edges.push_back(edge);
        DirectedWeightedEdge edge2(edge.getSink(), edge.getSource(), edge.getWeight());
        edges.push_back(edge2);
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

    std::vector<int> getPath(int goal, std::unordered_map<int, int> came_from) {
        std::vector<int> path;
        for (int current = goal; current != -1; current = came_from[current]) {
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
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
        return std::abs(currentVertex.position.x * ofGetWidth() - goalVertex.position.x * ofGetWidth()) + std::abs(currentVertex.position.y * ofGetHeight() - goalVertex.position.y * ofGetHeight());
    }

    inline float euclideanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
        auto& currentVertex = vertices.at(current);
        auto& goalVertex = vertices.at(goal);
        return std::sqrt(std::pow(currentVertex.position.x * ofGetWidth() - goalVertex.position.x * ofGetWidth(), 2) + std::pow(currentVertex.position.y * ofGetHeight() - goalVertex.position.y * ofGetHeight(), 2));
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

    void drawMap() const {
        ofSetLineWidth(2);
        ofNoFill();
        ofSetColor(ofColor::white);
        for (const auto& edge : edges) {
            Node sourceVertex = vertices.at(edge.getSource());
            Node sinkVertex = vertices.at(edge.getSink());

            float sourceX = sourceVertex.position.x * ofGetWidth();
            float sourceY = sourceVertex.position.y * ofGetHeight();
            float sinkX = sinkVertex.position.x * ofGetWidth();
            float sinkY = sinkVertex.position.y * ofGetHeight();

            ofDrawArrow(ofPoint(sourceX, sourceY), ofPoint(sinkX, sinkY), 5.0f);
        }

        for (const auto& vertex : vertices) {
            float x = vertex.second.position.x * ofGetWidth();
            float y = vertex.second.position.y * ofGetHeight();

            ofDrawCircle(x, y, 15);
            ofDrawBitmapString(vertex.first, x + 10, y + 10);
        }
    }

    void drawPath(std::vector<int> path) const {
        ofSetColor(ofColor::red);
        for (size_t i = 0; i < path.size() - 1; ++i) {
            int source = path[i];
            int sink = path[i + 1];
            Node sourceVertex = vertices.at(source);
            Node sinkVertex = vertices.at(sink);

            float sourceX = sourceVertex.position.x * ofGetWidth();
            float sourceY = sourceVertex.position.y * ofGetHeight();
            float sinkX = sinkVertex.position.x * ofGetWidth();
            float sinkY = sinkVertex.position.y * ofGetHeight();

            ofDrawLine(ofPoint(sourceX, sourceY), ofPoint(sinkX, sinkY));
        }
        ofSetColor(ofColor::white);
    }
};