#pragma once
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
        Tick t(blackboard);
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

class BTAction {
public:
    virtual ~BTAction() = default;
    virtual Status perform(Tick& tick) = 0; // 执行具体的行动，并返回执行结果
};

//----------------------------------------------------------------------
class BTActionNode : public Task {
private:
    BTAction* action; // 持有一个Action对象的引用或指针
public:
    BTActionNode(BTAction* action) : action(action) {}

    virtual ~BTActionNode() {
        delete action;
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