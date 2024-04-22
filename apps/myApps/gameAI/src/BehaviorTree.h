#pragma once
#include "DecisionTree.h"
#include "Blackboard.h"
enum class Status {
    Success,
    Failure,
    Running,
    Error
};

class Tick;

class Task {
public:
    virtual ~Task() = default;
    virtual Status run(Tick& tick) = 0;
    virtual void onEnter() {}
    virtual void onExit() {}
};

class Tick {
private:
    Blackboard* bb;
    Status currentStatus = Status::Running; // 默认状态为Running
    Task* currentTask = nullptr;
    float frameTime = 1 / 60;
public:
    Tick(Blackboard* bb) : bb(bb) {}

    float getFrameTime() {
        return frameTime;
    }

    void open(Task& t) {
        currentTask = &t;
        t.onEnter();
    }

    void enter(Task& t) {
        // 通常在任务开始时调用
    }

    Status execute(Task& t) {
        currentStatus = t.run(*this);
        return currentStatus;
    }

    void close(Task& t) {
        t.onExit();
        currentTask = nullptr;
    }

    void exit(Task& t) {
        // 通常在任务结束时调用
    }

    void setStatus(Status status) {
        currentStatus = status;
    }

    Status getStatus() const {
        return currentStatus;
    }

    void setData(const std::string& key, const std::any& value) {
        bb->set(key, value);
    }

    std::any getData(const std::string& key) {
        return bb->get(key);
    }
};

class BehaviourTree {
private:
    Task* root;
    Blackboard* blackboard;

public:
    BehaviourTree(Task* root, Blackboard* blackboard) : root(root), blackboard(blackboard) {}

    Status makeDecision() {
        Tick t(blackboard); // 假设Tick构造函数接受一个Blackboard引用
        Status result = root->run(t);
        return result;
    }
};

class RootTask : public Task {
private:
    Task* child;

public:
    RootTask(Task* child) : child(child) {}

    virtual ~RootTask() {
        delete child;
    }

    Status run(Tick& tick) override {  // 使用 override 明确表示重写
        if (child != nullptr) {
            return child->run(tick);
        }
        return Status::Failure;  // 如果没有子任务，返回失败状态
    }
};

//---------------------------------------------------------------------------------------------------------------------------------------
class BTAction {
public:
    virtual ~BTAction() = default;
    virtual Status perform(Tick& tick) = 0; // 执行具体的行动，并返回执行结果
};

class BTActionNode : public Task {
private:
    BTAction* action; // 持有一个Action对象的引用或指针
public:
    BTActionNode(BTAction* action) : action(action) {}

    virtual ~BTActionNode() {
        delete action; // 根据实际的所有权策略管理内存
    }

    Status run(Tick& tick) override {
        if (action) {
            return action->perform(tick); // 调用Action的perform方法
        }
        return Status::Error; // 如果没有设置action，则返回错误状态
    }
};

class BTSelectorNode : public Task {
    std::vector<Task*> children;

public:
    void addChild(Task* child) {
        children.push_back(child);
    }

    Status run(Tick& tick) override {
        for (auto& child : children) {
            tick.open(*child); // 准备执行子节点
            Status status = tick.execute(*child);
            if (status != Status::Failure) {
                tick.close(*child); // 成功或运行中，结束子节点的执行
                return status; // 返回子节点的状态
            }
            tick.close(*child); // 子节点失败，结束子节点的执行
        }
        return Status::Failure; // 所有子节点都失败
    }
};

class BTNondeterministicSelectorNode : public Task {
private:
    std::vector<Task*> children;
    std::vector<double> weights;
    std::discrete_distribution<> distribution;

public:
    void addChild(Task* child, double weight) {
        children.push_back(child);
        weights.push_back(weight);
        updateDistribution();
    }

    void updateDistribution() {
        if (!weights.empty()) {
            distribution = std::discrete_distribution<>(weights.begin(), weights.end());
        }
    }

    Status run(Tick& tick) override {
        if (children.empty()) return Status::Failure;

        std::random_device rd;
        std::mt19937 gen(rd());
        int chosenIndex = distribution(gen);

        tick.open(*children[chosenIndex]);
        Status result = tick.execute(*children[chosenIndex]);
        tick.close(*children[chosenIndex]);
        return result;
    }
};

class BTSequenceNode : public Task {
private:
    std::vector<Task*> children;

public:
    void addChild(Task* child) {
        children.push_back(child);
    }

    Status run(Tick& tick) override {
        for (Task* child : children) {
            tick.open(*child);
            Status status = tick.execute(*child);
            tick.close(*child);
            if (status != Status::Success) {
                return status; // 如果任一子节点失败或运行中，返回该状态
            }
        }
        return Status::Success; // 所有子节点成功
    }
};


class BTDecoratorNode : public Task {
protected:
    Task* child;

public:
    BTDecoratorNode(Task* child) : child(child) {}
    virtual ~BTDecoratorNode() {
        delete child;
    }
};

class BTRepeatUntilFailNode : public BTDecoratorNode {
public:
    BTRepeatUntilFailNode(Task* child) : BTDecoratorNode(child) {}

    Status run(Tick& tick) override {
        while (true) {
            tick.open(*child);
            Status status = tick.execute(*child);
            tick.close(*child);
            if (status != Status::Success) {
                return Status::Failure; // 一旦子节点失败，停止执行并返回失败
            }
        }
        return Status::Running; // 理论上不应该到这里
    }
};


class BTConditionalDecoratorNode : public BTDecoratorNode {
public:
    BTConditionalDecoratorNode(Task* child) : BTDecoratorNode(child) {}

    virtual bool condition(Tick& tick) = 0;  // 必须由具体类实现

    Status run(Tick& tick) override {
        if (condition(tick)) {
            tick.open(*child);
            Status status = tick.execute(*child);
            tick.close(*child);
            return status;
        }
        return Status::Failure; // 条件不满足，返回失败
    }
};


class BTInverterNode : public BTDecoratorNode {
public:
    BTInverterNode(Task* child) : BTDecoratorNode(child) {}

    Status run(Tick& tick) override {
        tick.open(*child); // 准备执行子节点
        Status status = tick.execute(*child);
        tick.close(*child); // 结束子节点的执行

        // 反转子节点的成功/失败状态
        if (status == Status::Success) {
            return Status::Failure;
        }
        else if (status == Status::Failure) {
            return Status::Success;
        }
        // 如果子节点是运行中或错误，保持原状态
        return status;
    }
};
//ConditionNode--------------------------------
//
class CanSeePlayer : public BTConditionalDecoratorNode {

public:
    CanSeePlayer(Task* child)
        : BTConditionalDecoratorNode(child) {}

    bool condition(Tick& tick) override {
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        auto* player = std::any_cast<Character*>(tick.getData("player"));
        auto* obstacles = std::any_cast<std::vector<PolygonCollision>*>(tick.getData("obstacles"));

        // 检查视线与每个障碍物的交叉
        if (willCollide(player->position, monster->position, *obstacles)) {
            std::cout << "can not see"<< std::endl;
            return false;  // 如果有障碍物阻挡视线，则返回false
        }
        //std::cout << "can see" << std::endl;
        return true;  // 如果没有障碍物阻挡视线，则可以看到玩家
    }
};

class IsCloseToPlayer : public BTConditionalDecoratorNode {
private:
    float distanceThreshold;  // 距离阈值

public:
    IsCloseToPlayer(Task* child, float threshold)
        : BTConditionalDecoratorNode(child), distanceThreshold(threshold) {}

    bool condition(Tick& tick) override {
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        auto* player = std::any_cast<Character*>(tick.getData("player"));

        return monster->position.distance(player->position) < distanceThreshold;
    }
};

class IsWanderTimeExceeded : public BTConditionalDecoratorNode {
private:
    float timeThreshold;  // 时间阈值

public:
    IsWanderTimeExceeded(Task* child, float threshold)
        : BTConditionalDecoratorNode(child), timeThreshold(threshold) {}

    bool condition(Tick& tick) override {
        auto* monsterWanderTime = std::any_cast<float*>(tick.getData("monsterWanderTime"));
        return (*monsterWanderTime > timeThreshold) || (*monsterWanderTime == 0);
    }
};

//Action Task----------------------------------
//
class BTCalculatePathAction : public BTAction {
public:
    BTCalculatePathAction() {}

    Status perform(Tick& tick) override {
        auto* pathFinder = std::any_cast<PathFinder*>(tick.getData("pathFinder"));
        auto* character = std::any_cast<Character*>(tick.getData("monster"));
        auto* graph = std::any_cast<DirectedGraph*>(tick.getData("graphIndoor"));

        if (!pathFinder || !character) {
            return Status::Failure;
        }

        auto* player = std::any_cast<Character*>(tick.getData("player"));
        float playerX = player->position.x;
        float playerY = player->position.y;

        pathFinder->targetPosition.set(player->position.x, player->position.y);
        pathFinder->startNode = graph->findNearestReachableNodeToClick(character->position.x, character->position.y);
        pathFinder->goalNode = graph->findNearestReachableNodeToClick(player->position.x, player->position.y);
        pathFinder->currentNodeIndex = 0;
        pathFinder->nextNodeIndex = 1;

        auto result = graph->aStar(pathFinder->startNode, pathFinder->goalNode, manhattanDistance);
        pathFinder->path = graph->getPath(pathFinder->goalNode, result.first);
        //std::cout << "PathFind" << std::endl;
        return Status::Success;
    }
};

class BTFollowPathAction : public BTAction {
public:
    BTFollowPathAction() {}

    Status perform(Tick& tick) override {
        auto* pathFinder = std::any_cast<PathFinder*>(tick.getData("pathFinder"));
        if (!pathFinder) {
            return Status::Failure;  // 路径完成或无效的 pathFinder
        }
        auto* monsterWanderTime = std::any_cast<float*>(tick.getData("monsterWanderTime"));

        SteeringOutput steering = pathFinder->getSteering();
        pathFinder->update(steering, 1.0f / 60.0f); // 假设帧率是60fps
        *monsterWanderTime += 1.0f / 60.0f;
        //if (pathFinder->currentNodeIndex >= pathFinder->path.size()) {
        //    return Status::Success; // 完成路径导航
        //}
        //std::cout << "go path, wanderTime is : " << *monsterWanderTime << std::endl;
        return Status::Success; // 路径尚未完成，继续执行
    }
};

class BTResetFindTargetAction : public BTAction {
public:
    BTResetFindTargetAction() {}

    Status perform(Tick& tick) override {
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        auto* pathFinder = std::any_cast<PathFinder*>(tick.getData("pathFinder"));
        auto* graphIndoor = std::any_cast<DirectedGraph*>(tick.getData("graphIndoor"));
        auto* monsterWanderTime = std::any_cast<float*>(tick.getData("monsterWanderTime"));

        if (!monster || !pathFinder) {
            return Status::Failure; // 必要组件不可用
        }
        *monsterWanderTime = 0.0f;

        monster->ifHaveGotPath = false;
        monster->isPathFinding = true;
        monster->findTime = 0;

        float width = ofGetWidth();
        float height = ofGetHeight();
        float randomX = ofRandom(0, width);
        float randomY = ofRandom(0, height);

        ofVec2f targetPoint = pathFinder->onMouseClick(randomX, randomY);
        pathFinder->targetPosition.set(targetPoint.x, targetPoint.y);
        pathFinder->startNode = graphIndoor->findNearestReachableNodeToClick(monster->position.x, monster->position.y);
        pathFinder->goalNode = graphIndoor->findNearestReachableNodeToClick(targetPoint.x, targetPoint.y);
        pathFinder->currentNodeIndex = 0;
        pathFinder->nextNodeIndex = 1;

        auto result = graphIndoor->aStar(pathFinder->startNode, pathFinder->goalNode, manhattanDistance);
        pathFinder->path = graphIndoor->getPath(pathFinder->goalNode, result.first);
        monster->ifHaveGotPath = true;

        //std::cout << "reset path" << std::endl;

        return monster->ifHaveGotPath ? Status::Success : Status::Failure;
    }
};

class BTWanderAction : public BTAction {
public:
    BTWanderAction() {}

    Status perform(Tick& tick) override {
        // 从黑板获取 RigidbodyForWander 实例
        auto* wanderer = std::any_cast<RigidbodyForWander*>(tick.getData("wanderer"));
        if (!wanderer) {
            std::cerr << "WanderAction failed: wanderer is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }

        // 执行漫游行为
        SteeringOutput steering = wanderer->getSteering();
        wanderer->update(steering, 1.0f / 60.0f); // 假设游戏运行帧率为60fps
        //std::cout << "WanderAction" << std::endl;

        return Status::Running; // 继续执行，因为漫游通常是一个持续的过程
    }
};

class BTMonsterChangeColorAction : public BTAction {
public:
    ofColor color;
    BTMonsterChangeColorAction(ofColor color) :color(color) {}

    Status perform(Tick& tick) override {
        // 从黑板获取 RigidbodyForWander 实例
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        if (!monster) {
            std::cerr << "BTChangeColorAction failed: monster is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }
        
        monster->color = color;

        return Status::Success;
    }
};

class BTMonsterSayFindUAction : public BTAction {
public:
    string sentence;
    BTMonsterSayFindUAction(string sentence) :sentence(sentence) {}

    Status perform(Tick& tick) override {
        // 从黑板获取 RigidbodyForWander 实例
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        if (!monster) {
            std::cerr << "BTMonsterSayAction failed: monster is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }
        monster->saySentence = sentence;
        /*auto* monsterSayingTime = std::any_cast<float*>(tick.getData("monsterSayingFindUTime"));
        if (*monsterSayingTime < 3.0f) {
            monster->saySentence = sentence;
            *monsterSayingTime += tick.getFrameTime();
        }
        else {
            monster->saySentence = "null";
        }*/
        return Status::Success;
    }
};

class BTMonsterSayHearUAction : public BTAction {
public:
    string sentence;
    BTMonsterSayHearUAction(string sentence) :sentence(sentence) {}

    Status perform(Tick& tick) override {
        // 从黑板获取 RigidbodyForWander 实例
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        if (!monster) {
            std::cerr << "BTMonsterSayAction failed: monster is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }

        auto* monsterSayingTime = std::any_cast<float*>(tick.getData("monsterSayingHearUTime"));
        if (*monsterSayingTime < 3.0f) {
            monster->saySentence = sentence;
            *monsterSayingTime += tick.getFrameTime();
        }
        else {
            monster->saySentence = "null";
        }
        return Status::Success;
    }
};

class BTMonsterEatAction : public BTAction {
public:
    BTMonsterEatAction(){}

    Status perform(Tick& tick) override {
        // 从黑板获取 RigidbodyForWander 实例
        auto* monster = std::any_cast<Character*>(tick.getData("monster"));
        if (!monster) {
            std::cerr << "BTMonsterEatAction failed: monster is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }

        auto* player = std::any_cast<Character*>(tick.getData("player"));
        if (!player) {
            std::cerr << "BTMonsterEatAction failed: player is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }

        auto monsterBirthPlace = std::any_cast<ofVec2f>(tick.getData("monsterBirthPlace"));
        auto playerBirthPlace = std::any_cast<ofVec2f>(tick.getData("playerBirthPlace"));

        monster->position = monsterBirthPlace;
        player->position = playerBirthPlace;

        return Status::Success;
    }
};

class BTMonsterGoTowardsPlayerAction : public BTAction {
public:
    BTMonsterGoTowardsPlayerAction() {}

    Status perform(Tick& tick) override {
        // 从黑板获取 RigidbodyForWander 实例
        auto* dynamicer = std::any_cast<RigidbodyForDynamicSteering*>(tick.getData("dynamicer"));
        if (!dynamicer) {
            std::cerr << "BTMonsterGoTowardsPlayerAction failed: dynamicer is null." << std::endl;
            return Status::Failure; // 如果获取失败，返回失败状态
        }
        auto* player = std::any_cast<Character*>(tick.getData("player"));

        ofVec2f playerPosition = player->position;

        SteeringOutput steering = dynamicer->getSteering(playerPosition);
        dynamicer->update(steering, 1.0f / 60.0f); // 假设游戏运行帧率为60fps
        //std::cout << "GoTowardsPlayerAction" << std::endl;

        return Status::Success; // 继续执行，因为漫游通常是一个持续的过程
    }
};