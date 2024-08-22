#pragma once
#include "ofxImGui.h"
class DemoBase {
public:
    virtual ~DemoBase() = default;

    virtual void setup() = 0;
    virtual void update(float deltaTime) = 0;
    virtual void draw() = 0;
    virtual void keyPressed(int key) = 0;
    virtual void mousePressed(int x, int y, int button) = 0;
};


/*
#pragma once

#include "ofMain.h"
#include "DemoBase.h"
#include "MovementDemo.h"
// 其他 Demo 的头文件

class ofApp : public ofBaseApp {
private:
    std::unique_ptr<DemoBase> currentDemo;

public:
    void setup() override {
        // 根据初始选择加载一个Demo
        currentDemo = std::make_unique<MovementDemo>();
        currentDemo->setup();
    }

    void update() override {
        currentDemo->update();
    }

    void draw() override {
        currentDemo->draw();
    }

    void keyPressed(int key) override {
        currentDemo->keyPressed(key);
    }

    void mousePressed(int x, int y, int button) override {
        currentDemo->mousePressed(x, y, button);
    }

    void switchDemo(int demoIndex) {
        // 根据 demoIndex 切换 Demo
        switch (demoIndex) {
        case 0:
            currentDemo = std::make_unique<MovementDemo>();
            break;
        case 1:
            // 切换到其他 Demo
            break;
            // 其他 Demo 切换逻辑
        }
        currentDemo->setup();  // 重新初始化新的 Demo
    }
};
*/