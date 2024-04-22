#pragma once
#include "ofMain.h"
#include "Character.h"



class RigidbodyForDynamicSteering {
public:
	Character* character; 

	float targetRadius;
	float slowRadius;
	float timeToTarget;

	float radius;

	RigidbodyForDynamicSteering()
		: targetRadius(5), slowRadius(200), timeToTarget(2), radius(20){}

	void update(const SteeringOutput& steering, float deltaTime) {
		character->velocity += steering.linear * deltaTime;
		if (character->velocity.length() > character->maxSpeed) {
			character->velocity.normalize();
			character->velocity *= character->maxSpeed;
		}
		character->position += character->velocity * deltaTime;
		if (character->velocity.lengthSquared() > 0) {
			character->orientation = atan2(character->velocity.y, character->velocity.x);
		}
	}

	SteeringOutput getSteering(const ofVec2f& target) {
		SteeringOutput steering;
		ofVec2f direction = target - character->position;
		float distance = direction.length();
		if (distance < targetRadius) {
			return steering;//do not need any behaviour
		}

		float targetSpeed;
		if (distance > slowRadius) {
			targetSpeed = character->maxSpeed;
		}
		else {
			targetSpeed = character->maxSpeed * distance / slowRadius;//smooth stop
		}

		ofVec2f targetVelocity = direction.getNormalized() * targetSpeed;
		steering.linear = (targetVelocity - character->velocity) / timeToTarget;
		if (steering.linear.length() > character->maxAcceleration) {
			steering.linear.normalize();
			steering.linear *= character->maxAcceleration;
		}

		return steering;
	}


	// More simple way
	SteeringOutput getSteering2(const ofVec2f& target) {
		SteeringOutput steering;

		ofVec2f direction = target - character->position;
		float distance = direction.length();

		if (distance < targetRadius) {

			float speed = character->velocity.length();

			if (distance > 0 && speed > 0) {
				float requiredDeceleration = -pow(speed, 2) / (2 * distance);

				steering.linear = character->velocity.getNormalized() * requiredDeceleration;
			}
			else {
				steering.linear = ofVec2f(0, 0);
			}
			return steering;
		}

		direction.normalize(); 
		steering.linear = direction * character->maxAcceleration;

		if (distance < slowRadius) {
			steering.linear *= (distance / slowRadius);
		}

		return steering;
	}

};