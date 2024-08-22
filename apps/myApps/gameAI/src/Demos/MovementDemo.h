#pragma once
#include "DemoBase.h"
#include "..\Movements\Arrive.h"
#include "..\Movements\Evade.h"
#include "..\Movements\Flee.h"
#include "..\Movements\Pursue.h"
#include "..\Movements\Seek.h"
#include "..\Movements\Wander.h"
class MovementDemo : public DemoBase {
public:
    Character mainCharacter;
    Character evadeTargetCharacter;
    Character purseTargetCharacter;

    // main character movement
    std::vector<std::shared_ptr<BasicMovement>> movements;
    std::shared_ptr<BasicMovement> currentMovement;

    // evade character arrive
    std::shared_ptr<Arrive> evadeTargetCharacterArriveMovement;

    // purse character arrive
    std::shared_ptr<Arrive> purseTargetCharacterArriveMovement;


    // Arrive

    // Evade

    // Flee

    // Purse

    // Seek


    void setup() override {
        movements.push_back(std::make_shared<Arrive>(&mainCharacter,100.0f, 50.0f, 50.0f, 10.0f));
        movements.push_back(std::make_shared<Evade>(&mainCharacter, 100.0f, 50.0f, &evadeTargetCharacter, 25.0f));
        movements.push_back(std::make_shared<Flee>(&mainCharacter, 100.0f, 50.0f, mainCharacter.position+ofVec2f(40.0f, 40.0f)));
        movements.push_back(std::make_shared<Pursue>(&mainCharacter, 100.0f, 50.0f, &purseTargetCharacter));
        movements.push_back(std::make_shared<Seek>(&mainCharacter, 100.0f, 50.0f, mainCharacter.position));

        evadeTargetCharacterArriveMovement = std::make_shared<Arrive>(&evadeTargetCharacter, 100.0f, 50.0f, 50.0f, 10.0f);
        purseTargetCharacterArriveMovement = std::make_shared<Arrive>(&purseTargetCharacter, 100.0f, 50.0f, 50.0f, 10.0f);

        // 设置默认的 Movement
        currentMovement = movements[0];

        // 初始化character参数
        mainCharacter.position = ofVec2f(100,100);
        mainCharacter.color = ofColor::blue;
        mainCharacter.radius = 20;
        mainCharacter.isDead = false;

        evadeTargetCharacter.position = mainCharacter.position + ofVec2f(200, 200);
        evadeTargetCharacter.color = ofColor::red;
        evadeTargetCharacter.radius = 20;
        evadeTargetCharacter.isDead = false;

        purseTargetCharacter.position = mainCharacter.position + ofVec2f(200, 200);
        purseTargetCharacter.color = ofColor::red;
        purseTargetCharacter.radius = 20;
        purseTargetCharacter.isDead = false;
    }

    void update(float deltaTime) override {
        if (currentMovement) {
            currentMovement->updatePosition(deltaTime);
        }
        if (std::dynamic_pointer_cast<Evade>(currentMovement)) {
            evadeTargetCharacterArriveMovement->updatePosition(deltaTime);
        }
        if (std::dynamic_pointer_cast<Pursue>(currentMovement)) {
            purseTargetCharacterArriveMovement->updatePosition(deltaTime);
        }
    }

    void draw() override {
        mainCharacter.draw();
        if(currentMovement == movements[1])
            evadeTargetCharacter.draw();
        if (currentMovement == movements[3])
            purseTargetCharacter.draw();
        renderImGui();
    }

    void renderImGui() {
        ImGui::Begin("Movement Demo Controls");

        // 选择 Movement 类型
        if (ImGui::CollapsingHeader("Movement Type")) {
            if (ImGui::Button("Arrive")) {
                currentMovement = movements[0];
            }
            if (ImGui::Button("Evade")) {
                currentMovement = movements[1];
            }
            if (ImGui::Button("Flee")) {
                currentMovement = movements[2];
            }
            if (ImGui::Button("Pursue")) {
                currentMovement = movements[3];
            }
            if (ImGui::Button("Seek")) {
                currentMovement = movements[4];
            }
        }

        ImGui::End();

        ImGui::Begin("Movement parameter set");

        // 调整Movement的通用参数
        if (currentMovement) {
            float maxForce = currentMovement->getMaxForce();
            if (ImGui::SliderFloat("Max Acceleration", &maxForce, 0.0f, 10000.0f)) {
                currentMovement->setMaxForce(maxForce);
            }

            float maxSpeed = currentMovement->getMaxSpeed();
            if (ImGui::SliderFloat("Max Speed", &maxSpeed, 0.0f, 20000.0f)) {
                currentMovement->setMaxSpeed(maxSpeed);
            }
        }

        // 根据Movement类型调整特有参数
        if (std::dynamic_pointer_cast<Arrive>(currentMovement)) {
            auto arriveMovement = std::dynamic_pointer_cast<Arrive>(currentMovement);

            float slowingRadius = arriveMovement->getSlowingRadius();
            if (ImGui::SliderFloat("Slowing Radius", &slowingRadius, 0.0f, 100.0f)) {
                arriveMovement->setSlowingRadius(slowingRadius);
            }

            float stopRadius = arriveMovement->getStopRadius();
            if (ImGui::SliderFloat("Stop Radius", &stopRadius, 0.0f, 50.0f)) {
                arriveMovement->setStopRadius(stopRadius);
            }
        }

        if (std::dynamic_pointer_cast<Evade>(currentMovement)) {
            //main character
            auto evadeMovement = std::dynamic_pointer_cast<Evade>(currentMovement);
        
            float evadeRadius = evadeMovement->getEvadeRadius();
            if (ImGui::SliderFloat("evade Radius", &evadeRadius, 0.0f, 100.0f)) {
                evadeMovement->setEvadeRadius(evadeRadius);
            }

            // evade target
            float slowingRadius = evadeTargetCharacterArriveMovement->getSlowingRadius();
            if (ImGui::SliderFloat("evade target: Slowing Radius", &slowingRadius, 0.0f, 100.0f)) {
                evadeTargetCharacterArriveMovement->setSlowingRadius(slowingRadius);
            }

            float stopRadius = evadeTargetCharacterArriveMovement->getStopRadius();
            if (ImGui::SliderFloat("evade target: Stop Radius", &stopRadius, 0.0f, 50.0f)) {
                evadeTargetCharacterArriveMovement->setStopRadius(stopRadius);
            }

            float maxForce = evadeTargetCharacterArriveMovement->getMaxForce();
            if (ImGui::SliderFloat("evade target: Max Acceleration", &maxForce, 0.0f, 10000.0f)) {
                evadeTargetCharacterArriveMovement->setMaxForce(maxForce);
            }

            float maxSpeed = evadeTargetCharacterArriveMovement->getMaxSpeed();
            if (ImGui::SliderFloat("evade target: Max Speed", &maxSpeed, 0.0f, 20000.0f)) {
                evadeTargetCharacterArriveMovement->setMaxSpeed(maxSpeed);
            }
        }

        if (std::dynamic_pointer_cast<Flee>(currentMovement)) {
            auto fleeMovement = std::dynamic_pointer_cast<Flee>(currentMovement);

        }

        if (std::dynamic_pointer_cast<Pursue>(currentMovement)) {
            auto purseMovement = std::dynamic_pointer_cast<Pursue>(currentMovement);

            // purse target
            float slowingRadius = purseTargetCharacterArriveMovement->getSlowingRadius();
            if (ImGui::SliderFloat("purse target: Slowing Radius", &slowingRadius, 0.0f, 100.0f)) {
                purseTargetCharacterArriveMovement->setSlowingRadius(slowingRadius);
            }

            float stopRadius = purseTargetCharacterArriveMovement->getStopRadius();
            if (ImGui::SliderFloat("purse target: Stop Radius", &stopRadius, 0.0f, 50.0f)) {
                purseTargetCharacterArriveMovement->setStopRadius(stopRadius);
            }

            float maxForce = purseTargetCharacterArriveMovement->getMaxForce();
            if (ImGui::SliderFloat("purse target: Max Acceleration", &maxForce, 0.0f, 10000.0f)) {
                purseTargetCharacterArriveMovement->setMaxForce(maxForce);
            }

            float maxSpeed = purseTargetCharacterArriveMovement->getMaxSpeed();
            if (ImGui::SliderFloat("purse target: Max Speed", &maxSpeed, 0.0f, 20000.0f)) {
                purseTargetCharacterArriveMovement->setMaxSpeed(maxSpeed);
            }

        }

        if (std::dynamic_pointer_cast<Seek>(currentMovement)) {
            auto seekMovement = std::dynamic_pointer_cast<Seek>(currentMovement);

        }
        // 其他Movement特有参数...

        ImGui::End();
    }
    void keyPressed(int key) override {
        // 实现 MovementDemo 的键盘事件处理
    }

    void mousePressed(int x, int y, int button) override {

        if (std::dynamic_pointer_cast<Arrive>(currentMovement)) {
            auto arriveMovement = std::dynamic_pointer_cast<Arrive>(currentMovement);
            arriveMovement->setTargetPosition(ofVec2f(x, y));
        }

        if (std::dynamic_pointer_cast<Evade>(currentMovement)) {
            evadeTargetCharacterArriveMovement->setTargetPosition(ofVec2f(x, y));
        }

        if (std::dynamic_pointer_cast<Flee>(currentMovement)) {
            auto fleeMovement = std::dynamic_pointer_cast<Flee>(currentMovement);
            fleeMovement->setThreatPosition(ofVec2f(x, y));
        }

        if (std::dynamic_pointer_cast<Pursue>(currentMovement)) {
            purseTargetCharacterArriveMovement->setTargetPosition(ofVec2f(x, y));
        }

        if (std::dynamic_pointer_cast<Seek>(currentMovement)) {
            auto seekMovement = std::dynamic_pointer_cast<Seek>(currentMovement);
            seekMovement->setTargetPosition(ofVec2f(x, y));
        }

    }
};