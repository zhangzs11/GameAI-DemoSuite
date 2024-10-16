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
};

class Tick {
public:
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
    }

    Status execute() {
        currentStatus = currentTask->run(*this);
        return currentStatus;
    }

    void close() {
        currentTask = nullptr;
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
    Tick* tick;
public:
    BehaviourTree(Task* root, Blackboard* blackboard, Tick* tick) : root(root), blackboard(blackboard), tick(tick) {}

    Status makeDecision(float deltaTime) {
        Status result;
        result = root->run(*tick);
        tick->frameTime = deltaTime;
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

    Status run(Tick& tick) override {
        if (child != nullptr) {
            tick.open(*child);
            Status status = tick.execute();
            tick.close();
            return status;
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
            tick.open(*child);
            Status status = tick.execute();
            if (status != Status::Failure) {
                tick.close(); // 其中只要有成功或运行中，结束子节点的执行
                return status;
            }
            tick.close(); // 子节点失败，结束子节点的执行
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
        Status result = tick.execute();
        tick.close();
        return result;
    }
};

class BTSequenceNode : public Task {
private:
    std::vector<Task*> children;
    int currentChildIndex = 0; 
public:
    void addChild(Task* child) {
        children.push_back(child);
    }

    Status run(Tick& tick) override {
        // Resume from the last running node
        while (currentChildIndex < children.size()) {
            Task* child = children[currentChildIndex];
            tick.open(*child);
            Status status = tick.execute();
            tick.close();

            if (status == Status::Running) {
                return Status::Running; // Continue running the current child
            }

            if (status != Status::Success) {
                currentChildIndex = 0; // Reset when a child fails
                return status; // Return failure or error
            }

            // Move to the next child if the current one succeeds
            currentChildIndex++;
        }

        currentChildIndex = 0; // Reset when all children succeed
        return Status::Success; // All children have succeeded
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
        // Execute the child task
        tick.open(*child);
        Status status = tick.execute();
        tick.close();

        if (status != Status::Success) {
            return Status::Failure;
        }

        // Return running after each successful execution to allow for frame-by-frame processing
        return Status::Running;
    }
};


class BTConditionalDecoratorNode : public BTDecoratorNode {
public:
    BTConditionalDecoratorNode(Task* child) : BTDecoratorNode(child) {}

    virtual bool condition(Tick& tick) = 0;  // 必须由具体类实现

    Status run(Tick& tick) override {
        if (condition(tick)) {
            tick.open(*child);
            Status status = tick.execute();
            tick.close();
            return status;
        }
        return Status::Failure;
    }
};


class BTInverterNode : public BTDecoratorNode {
public:
    BTInverterNode(Task* child) : BTDecoratorNode(child) {}

    Status run(Tick& tick) override {
        tick.open(*child);
        Status status = tick.execute();
        tick.close();

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