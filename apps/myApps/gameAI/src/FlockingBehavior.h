#pragma once
#include "ofMain.h"
#include "DynamicSteeringBehaviours.h"
#include "Character.h"

class FlockBehavior {
public:
    Character* character;
    Character* leader;
    std::vector<Character*>* targets;
    float threshold; //the distance when start to sperate
    float decayCoefficient;
    float maxAcceleration;

    float collisionRadius;// use to detect if close enough to avoid immediately

    FlockBehavior(Character* character, std::vector<Character*>* targets, Character* leader, float threshold, float decayCoefficient, float maxAcceleration, float collisionRadius)
        : character(character), targets(targets), leader(leader), threshold(threshold), decayCoefficient(decayCoefficient), maxAcceleration(maxAcceleration), collisionRadius(collisionRadius) {}

    void draw() {
        ofDrawCircle(character->position, character->radius);

        ofVec2f pointFront = character->position + ofVec2f(cos(character->orientation), sin(character->orientation)) * character->radius * 2;					//Triangle front point
        ofVec2f rightSide = character->position + ofVec2f(cos(character->orientation + PI / 2), sin(character->orientation + PI / 2)) * character->radius;		// Triangle right point
        ofVec2f leftSide = character->position + ofVec2f(cos(character->orientation - PI / 2), sin(character->orientation - PI / 2)) * character->radius;		// Triangle left point

        ofDrawTriangle(pointFront, rightSide, leftSide);
    }

    ofPoint calculateMassCenter() {
        float sumX = character->position.x, sumY = character->position.y;
        for (const auto& point : *targets) {
            sumX += point->position.x;
            sumY += point->position.y;
        }

        float averageX = sumX / targets->size() + 1;
        float averageY = sumY / targets->size() + 1;

        return ofPoint(averageX, averageY);
    }

//Sepration
public:
    SteeringOutput getSteeringForSepration() {
        SteeringOutput steering;
        for (Character* target : *targets) {
            ofVec2f direction = character->position - target->position;
            float distance = direction.length();

            if (distance < threshold && distance > 0) {
                float strength = std::min(decayCoefficient / (distance * distance), maxAcceleration);
                direction.normalize();
                steering.linear += direction * strength;
            }
        }
        return steering;
    }


//CollisionAvoidance
public:
    SteeringOutput getSteeringForCollisionAvoidance() {
        SteeringOutput steering;

        float shortestTime = std::numeric_limits<float>::infinity();
        Character* firstTarget = nullptr;
        float firstMinSeparation, firstDistance;
        ofVec2f firstRelativePos, firstRelativeVel;

        for (Character* target : *targets) {
            ofVec2f relativePos = target->position - character->position;
            ofVec2f relativeVel = target->velocity - character->velocity;
            float relativeSpeed = relativeVel.length();
            float timeToCollision = -(relativePos.dot(relativeVel)) / (relativeSpeed * relativeSpeed);

            float distance = relativePos.length();
            float minSeparation = distance - relativeSpeed * shortestTime;
            if (minSeparation > 2 * collisionRadius) continue;

            if (timeToCollision > 0 && timeToCollision < shortestTime) {
                shortestTime = timeToCollision;
                firstTarget = target;
                firstMinSeparation = minSeparation;
                firstDistance = distance;
                firstRelativePos = relativePos;
                firstRelativeVel = relativeVel;
            }
        }
        if (!firstTarget) return steering;

        ofVec2f relativePos;
        if (firstMinSeparation <= 0 || firstDistance < 2 * collisionRadius) {
            relativePos = firstTarget->position - character->position;
        }
        else {
            relativePos = firstRelativePos + firstRelativeVel * shortestTime;
        }

        relativePos.normalize();
        steering.linear = relativePos * maxAcceleration;
        return steering;
    }

    SteeringOutput getSteeringForLeaderFollowing() {
        SteeringOutput steering;
        if (leader != nullptr) {
            
            ofVec2f direction = leader->position - character->position;
            direction.normalize();
           
            steering.linear = direction * maxAcceleration;
        }
        return steering;
    }

    SteeringOutput getSteeringForCohesion() {
        SteeringOutput steering;
        if (!targets->empty()) {

            ofPoint massCenter = calculateMassCenter();
            ofVec2f direction = massCenter - character->position;
            direction.normalize();
            steering.linear = direction * maxAcceleration;
        }
        return steering;
    }

    SteeringOutput getSteering() {
        SteeringOutput steering;


        SteeringOutput separationSteering = getSteeringForSepration();

        SteeringOutput avoidanceSteering = getSteeringForCollisionAvoidance();

        SteeringOutput leaderSteering = getSteeringForLeaderFollowing();

        SteeringOutput cohesionSteering = getSteeringForCohesion();

        steering.linear += separationSteering.linear ;
        steering.linear += avoidanceSteering.linear * 1.5;
        steering.linear += leaderSteering.linear * 2;
        steering.linear += cohesionSteering.linear;

        if (steering.linear.length() > maxAcceleration) {
            steering.linear.normalize();
            steering.linear *= maxAcceleration;
        }

        return steering;
    }

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

};