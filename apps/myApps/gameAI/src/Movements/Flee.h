#pragma once
#include "BasicMovement.h"

class Flee : public BasicMovement {
private:
    ofVec2f threatPosition;

public:
    Flee(Character* character, float maxSpeed, float maxForce, const ofVec2f& threatPosition)
        : BasicMovement(character, maxSpeed, maxForce), threatPosition(threatPosition) {}

    void updatePosition(float deltaTime) override {
        // 计算远离威胁的期望速度
        ofVec2f desiredVelocity = (character->position - threatPosition).getNormalized() * maxSpeed;

        // 计算操控力
        ofVec2f steeringForce = desiredVelocity - character->velocity;
        steeringForce.limit(maxForce);

        // 更新速度
        character->velocity += steeringForce * deltaTime;
        character->velocity = limitSpeed(character->velocity);

        // 更新位置
        character->position += character->velocity * deltaTime;

        // Update orientation based on velocity
        if (character->velocity.length() > 0.0f) {
            character->orientation = atan2(character->velocity.y, character->velocity.x);
        }
        wrapAroundScreen();
    }

    void setThreatPosition(const ofVec2f& threat) { threatPosition = threat; }
    ofVec2f getThreatPosition() const { return threatPosition; }
};
