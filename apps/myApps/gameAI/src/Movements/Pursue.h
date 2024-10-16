#pragma once
#include "BasicMovement.h"

class Pursue : public BasicMovement {
private:
    Character* targetCharacter;

public:
    Pursue(Character* character, float maxSpeed, float maxForce, Character* targetCharacter)
        : BasicMovement(character, maxSpeed, maxForce), targetCharacter(targetCharacter) {}

    void updatePosition(float deltaTime) override {
        ofVec2f distance = targetCharacter->position - character->position;

        // 计算追上目标所需的时间
        float updatesNeeded = 1.0f;
        if (character->velocity.length() != 0.0f) {
            updatesNeeded = distance.length() / character->velocity.length();
        }

        // 预测目标的未来位置
        ofVec2f futurePosition = targetCharacter->position + targetCharacter->velocity * updatesNeeded;

        // 执行 Seek 行为，追逐预测的未来位置
        ofVec2f desiredVelocity = (futurePosition - character->position).getNormalized() * maxSpeed;
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
};
