#pragma once
#include "BasicMovement.h"

class Arrive : public BasicMovement {
public:
    ofVec2f targetPosition;
    float slowingRadius;
    float stopRadius;

public:
    Arrive(Character* character, float maxSpeed, float maxForce, float slowingRadius, float stopRadius)
        : BasicMovement(character, maxSpeed, maxForce), targetPosition(character->position), slowingRadius(slowingRadius), stopRadius(stopRadius){}

    void updatePosition(float deltaTime) override {
        ofVec2f desiredVelocity = targetPosition - this->character->position;
        float distance = desiredVelocity.length();

        if (distance > 0.0f)
        {
            float speed;
            if (distance > slowingRadius)
            {
                speed = maxSpeed;
            }
            else if (distance > stopRadius)
            {
                speed = maxSpeed * (distance - stopRadius) / (slowingRadius - stopRadius);
            }
            else
            {
                speed = 0.0f;
            }

            // Calculate the desired velocity
            desiredVelocity *= speed / distance;

            // Calculate the force
            ofVec2f force = desiredVelocity - this->character->velocity;

            // Limit the force
            force.limit(maxForce);

            character->velocity += force * deltaTime;

            character->velocity = limitSpeed(character->velocity);

            character->position += character->velocity * deltaTime;

            // Update orientation based on velocity
            if (character->velocity.length() > 0.0f) {
                character->orientation = atan2(character->velocity.y, character->velocity.x);
            }
            wrapAroundScreen();
            std::cout << "arrive: character->velocity * deltaTime: " << character->velocity * deltaTime << std::endl;
            return;
        }
        else
        {
            std::cout << "no move on arrive" << std::endl;
            return;
        }
    }

    void setTargetPosition(const ofVec2f& target) { targetPosition = target; }
    ofVec2f getTargetPosition() const { return targetPosition; }

    void setSlowingRadius(const float& radius) { slowingRadius = radius; }
    float getSlowingRadius() const { return slowingRadius; }

    void setStopRadius(const float& radius) { stopRadius = radius; }
    float getStopRadius() const { return stopRadius; }
};