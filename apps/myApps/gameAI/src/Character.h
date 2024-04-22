#pragma once
#include "ofMain.h"
class Character {
public:
    ofVec2f position;
    ofVec2f velocity;

    float maxAcceleration;
	float maxSpeed;

	float orientation;

	//Wander
	ofVec2f target;//target to face while wander

	//for draw
	float radius;
	ofColor color;

	//say
	string saySentence = "null";

	//Wander part
	float wanderOffset;
	float wanderRadius;
	float wanderRate;
	float wanderOrientation;

	//flock
	std::vector<Character*> targets;

	//BreadCrumbs
	vector<ofVec2f> breadcrumbs;

	//DecisionTree
	bool isPathFinding;
	float findTime;
	bool ifHaveGotPath;

	//item
	std::string tag;
	bool itemHasPickUp;

	float shootTime = 0;
	ofVec2f shootTarget = ofVec2f(0, 0);

	bool ifDie = false;
	Character() {
		wanderOffset = 10.0f;
		wanderRadius = 300.0f;//radius of the wandering circle
		wanderRate = 2.0f;
		wanderOrientation = 0.0f;
		maxAcceleration = 100.0f;

		orientation = 0.0f;
		maxSpeed = 300.0f;

		radius = 10.0f;
		color = ofColor(255, 0, 0);
	}
	void toroidalMap() {
		// Handle boundary violations (toroidal map)
		if (position.x < 0) position.x = ofGetWidth();
		if (position.x > ofGetWidth()) position.x = 0;
		if (position.y < 0) position.y = ofGetHeight();
		if (position.y > ofGetHeight()) position.y = 0;
	}
	
	void draw() {
		if (shootTime > 0) {
			ofSetColor(0, 255, 0);
			ofDrawLine(position, shootTarget);
			shootTime -= 0.1;
		}
		if (tag == "gun" && !itemHasPickUp) {
			ofSetColor(255, 255, 0);

			ofDrawBitmapString("This is a gun", position.x, position.y + 20);
			ofVec2f pointFront = position + ofVec2f(cos(orientation), sin(orientation)) * radius * 2;					//Triangle front point
			ofVec2f rightSide = position + ofVec2f(cos(orientation + PI / 2), sin(orientation + PI / 2)) * radius;		// Triangle right point
			ofVec2f leftSide = position + ofVec2f(cos(orientation - PI / 2), sin(orientation - PI / 2)) * radius;		// Triangle left point

			ofDrawTriangle(pointFront, rightSide, leftSide);

			return;
		}
		if (tag == "gun" && itemHasPickUp) {
			return;
		}
		else
		{
			if (ifDie) {
				return;
			}
			ofSetColor(color);
			ofDrawCircle(position, radius);

			ofVec2f pointFront = position + ofVec2f(cos(orientation), sin(orientation)) * radius * 2;					//Triangle front point
			ofVec2f rightSide = position + ofVec2f(cos(orientation + PI / 2), sin(orientation + PI / 2)) * radius;		// Triangle right point
			ofVec2f leftSide = position + ofVec2f(cos(orientation - PI / 2), sin(orientation - PI / 2)) * radius;		// Triangle left point

			ofDrawTriangle(pointFront, rightSide, leftSide);
		}
		if (saySentence != "null") {
			ofSetColor(ofColor::green);
			ofDrawBitmapString(saySentence, position.x, position.y-20);
		}
	}
	void drawBreadcrumbs() {
		ofSetColor(ofColor::white);
		
		for (auto& pos : breadcrumbs) {
			ofDrawCircle(pos, 5);
		}
	}

	void drawToroidalMap() {
		ofSetColor(color);
		ofVec2f DrawPosition;
		DrawPosition.x = fmod(position.x, ofGetWidth());
		if (DrawPosition.x < 0) {
			DrawPosition.x += ofGetWidth();
		}
		DrawPosition.y = fmod(position.y, ofGetHeight());
		if (DrawPosition.y < 0) {
			DrawPosition.y += ofGetHeight();
		}
		ofDrawCircle(DrawPosition, radius);

		ofVec2f pointFront = DrawPosition + ofVec2f(cos(orientation), sin(orientation)) * radius * 2;					//Triangle front point
		ofVec2f rightSide = DrawPosition + ofVec2f(cos(orientation + PI / 2), sin(orientation + PI / 2)) * radius;		// Triangle right point
		ofVec2f leftSide = DrawPosition + ofVec2f(cos(orientation - PI / 2), sin(orientation - PI / 2)) * radius;		// Triangle left point

		ofDrawTriangle(pointFront, rightSide, leftSide);
	}
};

class SteeringOutput {
public:
	ofVec2f linear; //acceleration
	float angular;

	SteeringOutput() :linear(0, 0), angular(0) {}
	SteeringOutput(ofVec2f linear, float angular) :linear(linear), angular(angular) {}
};

ofVec2f mapToScreenCoordinates(const ofVec2f& normalizedCoords) {
	float screenWidth = ofGetWidth();  // 获取当前窗口的宽度
	float screenHeight = ofGetHeight(); // 获取当前窗口的高度

	// 将归一化坐标转换为屏幕坐标
	ofVec2f screenCoords;
	screenCoords.x = normalizedCoords.x * screenWidth;
	screenCoords.y = normalizedCoords.y * screenHeight;

	return screenCoords;
}