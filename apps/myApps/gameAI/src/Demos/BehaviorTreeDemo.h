#pragma once

#include "DemoBase.h"
#include "..\BehaviorTree\MyBehaviorTree.h"

class BehaviorTreeDemo : public DemoBase {
public:
    // Map
    DirectedGraph graph;
    Map map;

    // Me
    Character myCharacter;
    bool isMePathFinding;
    std::shared_ptr<Arrive> myArriveBehavior;
    //std::shared_ptr<Seek> mySeekBehavior;
    PathFinder myPathFinder;
    bool isPlayerPathFinding;

    // BT AI
    Character BTAICharacter;
    // bool isBTAIPathFinding;
    std::shared_ptr<Arrive> BTAIArriveBehavior;
    //std::shared_ptr<Seek> BTAISeekBehavior;
    PathFinder BTAIPathFinder;

    // Behavior Tree
    BehaviourTree* behaviourTree;
    Blackboard* blackboard;
    Tick* tick;

    void initializeBehaviourTree() {
        // 创建黑板数据
        blackboard->set("monster", &BTAICharacter);
        blackboard->set("player", &myCharacter);
        blackboard->set("pathFinder", &BTAIPathFinder);
        blackboard->set("graphIndoor", &map);
        blackboard->set("monsterWanderTime", new float(0.0f));
        blackboard->set("currentAction", string("root"));
        // 创建Action
        BTSayAction* saySeeAction = new BTSayAction("I see you!");
        BTSayAction* sayHearAction = new BTSayAction("I hear you!");
        BTSayAction* sayWhereAreYouAction = new BTSayAction("Where are you?");
        BTFindPathToPlayerAction* findPathToPlayerAction = new BTFindPathToPlayerAction();
        BTFollowPathToPlayerAction* followPathToPlayerAction = new BTFollowPathToPlayerAction();
        BTRandomResetFindTargetAction* randomPathAction = new BTRandomResetFindTargetAction();
        BTFollowRandomPathAction* followRandomPathAction = new BTFollowRandomPathAction();

        // 创建ActionNode
        BTActionNode* saySeeNode = new BTActionNode(saySeeAction);
        BTActionNode* sayHearNode = new BTActionNode(sayHearAction);
        BTActionNode* sayWhereAreYouNode = new BTActionNode(sayWhereAreYouAction);
        BTActionNode* findPathNode = new BTActionNode(findPathToPlayerAction);
        BTActionNode* followPathNode = new BTActionNode(followPathToPlayerAction);
        BTActionNode* randomPathNode = new BTActionNode(randomPathAction);
        BTActionNode* followRandomPathNode = new BTActionNode(followRandomPathAction);

        // 序列和选择器节点
        BTSequenceNode* seePlayerSequence = new BTSequenceNode();
        seePlayerSequence->addChild(findPathNode);
        seePlayerSequence->addChild(saySeeNode);
        seePlayerSequence->addChild(followPathNode);

        BTSequenceNode* hearPlayerSequence = new BTSequenceNode();
        // hearPlayerSequence->addChild(randomPathNode);
        hearPlayerSequence->addChild(sayHearNode);
        // hearPlayerSequence->addChild(followRandomPathNode);

        BTSequenceNode* wanderSequence = new BTSequenceNode();
        wanderSequence->addChild(sayWhereAreYouNode);
        wanderSequence->addChild(followRandomPathNode);

        // 创建Condition 装饰节点
        CanSeePlayer* canSeePlayerCondition = new CanSeePlayer(seePlayerSequence);
        IsCloseToPlayer* isCloseToPlayerCondition = new IsCloseToPlayer(hearPlayerSequence, 200.0f); // 距离阈值200
        IsWanderTimeExceeded* isWanderTimeExceededCondition = new IsWanderTimeExceeded(randomPathNode, 3.0f);

        // Repeat Until Fail Nodes
        BTRepeatUntilFailNode* repeatSeeUntilFail = new BTRepeatUntilFailNode(canSeePlayerCondition);

        BTRepeatUntilFailNode* repeatHearUntilFail = new BTRepeatUntilFailNode(isCloseToPlayerCondition);

        // Selector Node
        BTSelectorNode* rootSelector = new BTSelectorNode();
        rootSelector->addChild(repeatSeeUntilFail);
        rootSelector->addChild(repeatHearUntilFail);
        rootSelector->addChild(isWanderTimeExceededCondition); // 如果 wander 超时或未开始，生成随机路径
        rootSelector->addChild(wanderSequence); // 默认说"你在哪"并移动

        
        // 根节点
        RootTask* root = new RootTask(rootSelector);
        // 初始化行为树
        behaviourTree = new BehaviourTree(root, blackboard, tick);
    }

    void setup() override {
        // Init map
        map.graph = &graph;
        static char fileName[128] = "pathfinding_testmap1";
        static std::string loadPath = defaultMapDataFolder + std::string(fileName) + ".json";  // 自动加上 ".json"
        loadMapFromJson(map, loadPath);
        map.generateNodesRecursively(ofRectangle(0.0f, 0.0f, 1.0f, 1.0f), 15, 10.0f, 2.1f);

        // Init player
        myCharacter.position = ofVec2f(0.5, 0.5);
        myCharacter.color = ofColor::blue;
        myCharacter.radius = 5;
        isPlayerPathFinding = false;

        // Init player movement Arrive and Seek
        myArriveBehavior = std::make_shared<Arrive>(&myCharacter, 100.0f, 15000.0f, 150.0f, 10.0f);
        //mySeekBehavior = std::make_shared<Seek>(&myCharacter, 100.0f, 50.0f, myCharacter.position);

        // Init player pathfinder
        myPathFinder.character = &myCharacter;
        myPathFinder.map = &map;
        myPathFinder.arriveBehavior = myArriveBehavior;
        //myPathFinder.seekBehavior = mySeekBehavior;

        // Init AI monster
        BTAICharacter.position = ofVec2f(ofGetWidth() * 0.9, ofGetHeight() * 0.9);
        BTAICharacter.color = ofColor::red;
        BTAICharacter.radius = 5;
        // Init AI movement
        BTAIArriveBehavior = std::make_shared<Arrive>(&BTAICharacter, 100.0f, 15000.0f, 150.0f, 10.0f);
        // Init AI finder
        BTAIPathFinder.character = &BTAICharacter;
        BTAIPathFinder.map = &map;
        BTAIPathFinder.arriveBehavior = BTAIArriveBehavior;
        // Init Behavior Tree
        blackboard = new Blackboard();
        tick = new Tick(blackboard);
        // tick->frameTime = 1.0f / 60.0f;
        initializeBehaviourTree();

    }

    void update(float deltaTime) override {
        if (isPlayerPathFinding) {
            myPathFinder.updatePositionUsingGraph(deltaTime);
        }
        behaviourTree->makeDecision(deltaTime);
    }

    void renderImGui() {
        // 开始 ImGui 窗口
        ImGui::Begin("Blackboard Data");

        const auto& data = blackboard->getData();

        for (const auto& [key, value] : data) {
            // 尝试转换为已知类型
            if (value.type() == typeid(float*)) {
                float* val = std::any_cast<float*>(value);
                ImGui::Text("%s: %.3f", key.c_str(), *val);
            }
            else if (value.type() == typeid(bool)) {
                bool val = std::any_cast<bool>(value);
                ImGui::Text("%s: %s", key.c_str(), val ? "true" : "false");
            }
            else if (value.type() == typeid(ofVec2f)) {
                ofVec2f val = std::any_cast<ofVec2f>(value);
                ImGui::Text("%s: (%.3f, %.3f)", key.c_str(), val.x, val.y);
            }
            else if (value.type() == typeid(Character*)) {
                Character* character = std::any_cast<Character*>(value);
                if (character) {
                    // 展示 Character 对象的具体成员变量
                    ImGui::Text("%s Position: (%.2f, %.2f)", key.c_str(), character->position.x, character->position.y);
                }
            }
            else if (value.type() == typeid(string)) {
                string val = std::any_cast<string>(value);
                ImGui::Text("%s: %s", key.c_str(), val.c_str());
            }
            // 添加更多类型的处理...
            /*else {
                ImGui::Text("%s: [Unknown Type]", key.c_str());
            }*/
        }

        // 结束 ImGui 窗口
        ImGui::End();
    }
    void draw() override {
        // Draw map
        for (const auto& polygon : map.collisionList) {
            polygon.drawPolygon();
        }
        // Draw player
        myCharacter.draw();
        graph.drawPath(myPathFinder.path);
        //Draw monster
        BTAICharacter.draw();
        graph.drawPath(BTAIPathFinder.path);
        renderImGui();
    }

    void keyPressed(int key) override {
        
    }

    void mousePressed(int x, int y, int button) override {
        // player path finding
        ofVec2f targetPoint = myPathFinder.GetPositionOnMouseClick(x, y);

        myPathFinder.targetPosition.set(targetPoint.x, targetPoint.y);
        myPathFinder.startNode = map.findNearestReachableNodeToClick(myCharacter.position.x, myCharacter.position.y);
        myPathFinder.goalNode = map.findNearestReachableNodeToClick(targetPoint.x, targetPoint.y);
        myPathFinder.currentNodeIndex = 0;
        myPathFinder.nextNodeIndex = 1;

        auto result = graph.aStar(myPathFinder.startNode, myPathFinder.goalNode, BTmanhattanDistance);
        auto came_from = result.first;
        myPathFinder.path = graph.getPath(myPathFinder.goalNode, came_from);
        isPlayerPathFinding = true;
    }
};