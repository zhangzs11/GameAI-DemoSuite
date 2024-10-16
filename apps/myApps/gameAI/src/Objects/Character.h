#ifndef CHARACTER_H
#define CHARACTER_H
#include <vector>
#include "BasicObject.h"
#include "Bullet.h"

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

	//say
	string saySentence = "null";

	//// Bullet
	//// ---------------
	//std::vector<Bullet> bullets;

	//void fireBullet(float speed, float maxDistance) {
	//	if (isDead) return;
	//	Bullet newBullet(position, orientation, speed, maxDistance);
	//	bullets.push_back(newBullet);
	//}

	//void updateBullets(float deltaTime, std::vector<Character>& characters, const std::vector<PolygonCollision>& obstacles) {
	//	for (auto& bullet : bullets) {
	//		bullet.update(deltaTime);
	//		for (const auto& obstacle : obstacles) {
	//			if (bullet.checkCollision(obstacle)) {
	//				break;  // 如果子弹碰撞到了障碍物，就不再检查其他障碍物
	//			}
	//		}
	//	}

	//	// 移除已经失效的子弹
	//	bullets.erase(std::remove_if(bullets.begin(), bullets.end(), [](const Bullet& bullet) {
	//		return !bullet.isActive;
	//		}), bullets.end());
	//}

	void draw() override {
		if (isDead) return;
		ofSetColor(color);
		ofDrawCircle(position, radius);

		ofVec2f pointFront = position + ofVec2f(cos(orientation), sin(orientation)) * radius * 2;
		ofVec2f rightSide = position + ofVec2f(cos(orientation + PI / 2), sin(orientation + PI / 2)) * radius;
		ofVec2f leftSide = position + ofVec2f(cos(orientation - PI / 2), sin(orientation - PI / 2)) * radius;

		ofDrawTriangle(pointFront, rightSide, leftSide);

		if (saySentence != "null") {
			ofSetColor(ofColor::green);
			ofDrawBitmapString(saySentence, position.x, position.y - 20);
		}

		//// Draw Bullet
		////
		//for (auto& bullet : bullets) {
		//	bullet.draw();
		//}
	}


};

#endif // CHARACTER_H