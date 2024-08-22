#pragma once
#include "BasicMovement.h"

class Evade : public BasicMovement {
private:
    Character* targetCharacter;
    float evadeRadius; // 逃离半径

public:
    Evade(Character* character, float maxSpeed, float maxForce, Character* targetCharacter, float evadeRadius)
        : BasicMovement(character, maxSpeed, maxForce), targetCharacter(targetCharacter), evadeRadius(evadeRadius) {}

    void updatePosition(float deltaTime) override {
        ofVec2f distance = targetCharacter->position - character->position;

        // 如果威胁在逃离半径之外，则不需要逃离
        if (distance.length() > evadeRadius) {
            return;
        }

        // 计算威胁的未来位置
        float updatesNeeded = 1.0f;
        if (targetCharacter->velocity.length() != 0.0f) {
            updatesNeeded = distance.length() / targetCharacter->velocity.length();
        }
        ofVec2f futurePosition = targetCharacter->position + targetCharacter->velocity * updatesNeeded;

        // 执行反向的 Pursue 行为，逃离未来位置
        ofVec2f desiredVelocity = (character->position - futurePosition).getNormalized() * maxSpeed;
        ofVec2f steeringForce = desiredVelocity - character->velocity;
        steeringForce.limit(maxForce);

        // 更新角色的速度和位置
        character->velocity += steeringForce * deltaTime;
        character->velocity = limitSpeed(character->velocity);
        character->position += character->velocity * deltaTime;

        // Update orientation based on velocity
        if (character->velocity.length() > 0.0f) {
            character->orientation = atan2(character->velocity.y, character->velocity.x);
        }
        wrapAroundScreen();
    }

    void setTargetCharacter(Character* target) { targetCharacter = target; }
    Character* getTargetCharacter() const { return targetCharacter; }

    void setEvadeRadius(float radius) { evadeRadius = radius; }
    float getEvadeRadius() const { return evadeRadius; }
};
