#ifndef BULLET_H
#define BULLET_H
#include "BasicObject.h"
#include "../Pathfinding/Map.h"

class Bullet : public BasicObject {
public:
    ofVec2f velocity;
    float orientation;
    float maxDistance;
    float currentDistance;
    bool isActive;

    Bullet(ofVec2f position, float orientation, float speed, float maxDistance)
        : velocity(ofVec2f(cos(orientation), sin(orientation))* speed),
        orientation(orientation),
        maxDistance(maxDistance),
        currentDistance(0),
        isActive(true)
    {
        this->position = position;
        this->color = ofColor::yellow;  // color
    }

    void update(float deltaTime) {
        if (!isActive) return;

        ofVec2f deltaPosition = velocity * deltaTime;
        position += deltaPosition;
        currentDistance += deltaPosition.length();

        if (currentDistance >= maxDistance) {
            isActive = false;  // 如果达到最大距离，子弹失效
        }
    }

    void draw() override {
        if (!isActive) return;

        ofSetColor(color);
        ofDrawCircle(position, 5);  // 用一个小圆形表示子弹
    }

    bool checkCollision(const PolygonCollision& polygon) {
        // 简单的碰撞检测，可以根据需要扩展
        if (isPointInsidePolygon(position, polygon)) {
            isActive = false;
            return true;
        }
        return false;
    }

    //bool checkCollision(Character& target) {
    //    float distance = position.distance(target.position);

    //    if (distance < target.radius && !target.isDead) {
    //        isActive = false;
    //        target.currentHp -= 10;  // 每次击中减10点生命值
    //        if (target.currentHp <= 0) {
    //            target.isDead = true;
    //        }
    //        return true;
    //    }

    //    return false;
    //}

};




# endif
