#pragma once
#include "Character.h"
#include "Graph.h"
#include "GoapStatus.h"

class GoapSensor {
public:
    Character* player; // 指向拥有此传感器的AI角色
    Character* monster;
    const std::vector<PolygonCollision>* collisionList;
    GoapStatus& status;

    GoapSensor(Character* player, Character* monster, const std::vector<PolygonCollision>* collisionList, GoapStatus& status)
        : player(player), monster(monster), collisionList(collisionList), status(status) {}

    void updateSensoryInformation() {
        status.set("hearMonster", CanHearMonster());
        status.set("seeMonster", CanSeeMonster());
    }

    bool CanHearMonster() {
        float distance = player->position.distance(monster->position);
        return distance < 100;
    }

    bool CanSeeMonster() {
        return !willCollide(player->position, monster->position, *collisionList);
    }
};