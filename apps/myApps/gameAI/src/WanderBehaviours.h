#pragma once
#include "ofMain.h"
#include "DynamicSteeringBehaviours.h"

class RigidbodyForWander {

//Face part
public:
    Character* character;

	void setTarget(const ofVec2f& targetPosition) {
        character->target = targetPosition;
	}

    // diff between oriention and direction2Target
    float angleToTarget() {
        ofVec2f direction = character->target - character->position;
        float angleToTarget = atan2(direction.y, direction.x);
        float angleDiff = angleToTarget - character->orientation;

        while (angleDiff > PI) angleDiff -= TWO_PI;
        while (angleDiff < -PI) angleDiff += TWO_PI;

        return angleDiff;
    }


    SteeringOutput getFaceSteering() {
        SteeringOutput steering;
        float angleDiff = angleToTarget();

        float angularSpeed = 2.0f; // can adjust
        steering.angular = angleDiff * angularSpeed;

        steering.linear = ofVec2f(0, 0);

        return steering;
    }

//Wander part
public:

    RigidbodyForWander() {
    }
    float randomBinomial() {
        return ofRandom(-1, 1);
    }

    ofVec2f angleToVector(float angle) {
        return ofVec2f(cos(angle), sin(angle));
    }
    SteeringOutput getSteering() {
        SteeringOutput steering;

        // calculate face target
        character->wanderOrientation += randomBinomial() * character->wanderRate;

        float targetOrientation = character->wanderOrientation + character->orientation;

        //float targetOrientation = wanderOrientation;

        character->target = character->position + character->wanderOffset * angleToVector(character->orientation);
        character->target += character->wanderRadius * angleToVector(targetOrientation);

        steering = getFaceSteering();

        steering.linear = character->maxAcceleration * angleToVector(character->orientation);

        return steering;
    }

    //more simple way
    SteeringOutput getSteering2() {
        SteeringOutput steering;

        float maxAngleChange = PI / 4;
        float angleChange = ofRandom(-maxAngleChange, maxAngleChange);  //[-maxAngleChange, maxAngleChange]

        character->orientation += angleChange;

        if (character->orientation > TWO_PI) {
            character->orientation -= TWO_PI;
        }
        else if (character->orientation < 0) {
            character->orientation += TWO_PI;
        }

        steering.linear = character->maxAcceleration * angleToVector(character->orientation);
 
        steering.angular = 0;

        return steering;
    }

    void update(const SteeringOutput& steering, float deltaTime) {

        character->orientation += steering.angular * deltaTime;

        while (character->orientation > TWO_PI) character->orientation -= TWO_PI;
        while (character->orientation < 0) character->orientation += TWO_PI;


        ofVec2f orientationVector = ofVec2f(cos(character->orientation), sin(character->orientation));
        character->velocity += steering.linear * deltaTime;
        character->velocity = orientationVector * character->velocity.length();

        if (character->velocity.length() > character->maxSpeed) {
            character->velocity.normalize();
            character->velocity *= character->maxSpeed;
        }

        character->position += character->velocity * deltaTime;
    }

};