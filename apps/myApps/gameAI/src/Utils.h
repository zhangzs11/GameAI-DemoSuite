#pragma once
#include "ofMain.h"

ofVec2f mapToScreenCoordinates(const ofVec2f& normalizedCoords) {
    float screenWidth = ofGetWidth();
    float screenHeight = ofGetHeight();

    // 将归一化坐标转换为屏幕坐标
    ofVec2f screenCoords;
    screenCoords.x = normalizedCoords.x * screenWidth;
    screenCoords.y = normalizedCoords.y * screenHeight;

    return screenCoords;
}