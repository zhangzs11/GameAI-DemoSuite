#pragma once
#include "BasicMovement.h"

class Seek : public BasicMovement {
public:
	ofVec2f targetPosition;

public:
    Seek(Character* character, float maxSpeed, float maxForce, const ofVec2f& targetPosition)
        : BasicMovement(character, maxSpeed, maxForce), targetPosition(targetPosition) {}

    void updatePosition(float deltaTime) override {
        ofVec2f desiredVelocity = (targetPosition - character->position).getNormalized() * maxSpeed;
        ofVec2f steeringForce = desiredVelocity - character->velocity;
        steeringForce.limit(maxForce);

        character->velocity += steeringForce * deltaTime;
        character->velocity = limitSpeed(character->velocity);

        character->position += character->velocity * deltaTime;

        // Update orientation based on velocity
        if (character->velocity.length() > 0.0f) {
            character->orientation = atan2(character->velocity.y, character->velocity.x);
        }
        wrapAroundScreen();
        //std::cout << "Seek: character->velocity * deltaTime: " << character->velocity * deltaTime << std::endl;
    }

    void setTargetPosition(const ofVec2f& target) { targetPosition = target; }
    ofVec2f getTargetPosition() const { return targetPosition; }
};