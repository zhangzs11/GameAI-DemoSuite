#pragma once
#include "DemoBase.h"
#include "..\Goap\Core\GoapAction.h"
#include "..\Goap\Core\GoapAgent.h"
#include "..\Goap\Core\GoapSensor.h"
#include "..\Goap\Core\GoapStatus.h"
/*
#include "..\Goap\MyActions\GoapDanceAction.h"
#include "..\Goap\MyActions\GoapFindMonsterAction.h"
#include "..\Goap\MyActions\GoapGoRandomAction.h"
#include "..\Goap\MyActions\GoapGoToTargetAction.h"
#include "..\Goap\MyActions\GoapShootAction.h"
*/
class GoapDemo : public DemoBase {
public:
    void setup() override {
        // 实现 MovementDemo 的 setup 逻辑
    }

    void update(float deltaTime) override {
        // 实现 MovementDemo 的 update 逻辑
    }

    void draw() override {
        // 实现 MovementDemo 的 draw 逻辑
    }

    void keyPressed(int key) override {
        // 实现 MovementDemo 的键盘事件处理
    }

    void mousePressed(int x, int y, int button) override {
        // 实现 MovementDemo 的鼠标事件处理
    }
};