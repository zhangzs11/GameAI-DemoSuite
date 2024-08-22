#pragma once
#include "ofMain.h"

class BasicObject {
public:
    ofVec2f position;
    ofColor color;
    std::string tag;

    BasicObject()
        : position(0, 0),
        color(ofColor(255, 0, 0)), tag(""){}

    virtual void draw() = 0;
};