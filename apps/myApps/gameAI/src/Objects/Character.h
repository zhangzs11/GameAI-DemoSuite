#ifndef CHARACTER_H
#define CHARACTER_H
#include "BasicObject.h"

class Character : public BasicObject {
public:
	float radius;

	ofVec2f velocity;
	float orientation;
	float maxAcceleration;
	float maxSpeed;

	float maxHp;
	float currentHp;
	bool isDead;

	void draw() override {
		if (isDead) return;
		ofSetColor(color);
		ofDrawCircle(position, radius);

		ofVec2f pointFront = position + ofVec2f(cos(orientation), sin(orientation)) * radius * 2;
		ofVec2f rightSide = position + ofVec2f(cos(orientation + PI / 2), sin(orientation + PI / 2)) * radius;
		ofVec2f leftSide = position + ofVec2f(cos(orientation - PI / 2), sin(orientation - PI / 2)) * radius;

		ofDrawTriangle(pointFront, rightSide, leftSide);
	}
};

#endif // CHARACTER_H