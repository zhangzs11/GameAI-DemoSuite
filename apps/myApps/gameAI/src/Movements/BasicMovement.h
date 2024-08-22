#pragma once
#include "..\Objects\Character.h"

class BasicMovement {
public:
    Character* character; 
    float maxSpeed;       
    float maxForce;      

public:
    BasicMovement(Character* character, float maxSpeed, float maxForce)
        : character(character), maxSpeed(maxSpeed), maxForce(maxForce) {}

    virtual ~BasicMovement() {}

    virtual void updatePosition(float deltaTime) = 0;

    ofVec2f limitSpeed(const ofVec2f& velocity) {
        if (velocity.length() > maxSpeed) {
            return velocity.getNormalized() * maxSpeed;
        }
        return velocity;
    }

    void wrapAroundScreen() {
        if (character->position.x < 0) {
            character->position.x += ofGetWidth();
        }
        if (character->position.x > ofGetWidth()) {
            character->position.x -= ofGetWidth();
        }
        if (character->position.y < 0) {
            character->position.y += ofGetHeight();
        }
        if (character->position.y > ofGetHeight()) {
            character->position.y -= ofGetHeight();
        }
    }

    void setMaxSpeed(float speed) { maxSpeed = speed; }
    float getMaxSpeed() const { return maxSpeed; }

    void setMaxForce(float force) { maxForce = force; }
    float getMaxForce() const { return maxForce; }

    void setCharacter(Character* character) { this->character = character; }
    Character* getCharacter() const { return character; }
};