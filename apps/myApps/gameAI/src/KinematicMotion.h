#pragma once
#include "ofMain.h"

class Rigidbody {
public:
	ofVec2f position;
	ofVec2f velocity;
	float orientation;

	float radius;

	Rigidbody() : orientation(0), radius(20) {}

	void update(float deltaTime) {
		position += velocity * deltaTime;

		if (velocity.lengthSquared() > 0) {
			orientation = atan2(velocity.y, velocity.x);
		}
	}

	void draw() {
		ofDrawCircle(position, radius);

		ofVec2f pointFront = position + ofVec2f(cos(orientation), sin(orientation)) * radius*2;					//Triangle front point
		ofVec2f rightSide = position + ofVec2f(cos(orientation + PI / 2), sin(orientation + PI / 2)) * radius;	// Triangle right point
		ofVec2f leftSide = position + ofVec2f(cos(orientation - PI / 2), sin(orientation - PI / 2)) * radius;	// Triangle left point

		ofDrawTriangle(pointFront, rightSide, leftSide);
	}
};