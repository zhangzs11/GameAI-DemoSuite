#include "ofApp.h"
// Heuristics 
//float manhattanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
//	auto& currentVertex = vertices.at(current);
//	auto& goalVertex = vertices.at(goal);
//	return std::abs(currentVertex.x * ofGetWidth() - goalVertex.x * ofGetWidth()) + std::abs(currentVertex.y * ofGetHeight() - goalVertex.y * ofGetHeight());
//}
//
//float euclideanDistance(int current, int goal, const std::unordered_map<int, Node>& vertices) {
//	auto& currentVertex = vertices.at(current);
//	auto& goalVertex = vertices.at(goal);
//	return std::sqrt(std::pow(currentVertex.x * ofGetWidth() - goalVertex.x * ofGetWidth(), 2) + std::pow(currentVertex.y * ofGetHeight() - goalVertex.y * ofGetHeight(), 2));
//}

//--------------------------------------------------------------
void ofApp::setup(){
	// frame rate setup
	ofSetFrameRate(120);

	// font setup
	font.load("verdana.ttf", 200);

	// gui setup
	gui.setup();

	// demo setup
	movementDemo.setup();
	pathfindingDemo.setup();
	behaviorTreeDemo.setup();
	goapDemo.setup();

	currentDemo = &behaviorTreeDemo;

}

//--------------------------------------------------------------
void ofApp::update(){
	if (currentDemo) {
		currentDemo->update(1.0f / 120.0f);
	}
}

void ofApp::renderMainImGui() {
	// 选择不同的 Demo
	if (ImGui::CollapsingHeader("Demo Selection")) {
		if (ImGui::Button("Movement Demo")) {
			currentDemo = &movementDemo;
		}
		if (ImGui::Button("Pathfinding Demo")) {
			currentDemo = &pathfindingDemo;
		}
		if (ImGui::Button("BehaviorTree Demo")) {
			currentDemo = &behaviorTreeDemo;
		}
		if (ImGui::Button("Goap Demo")) {
			currentDemo = &goapDemo;
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	gui.begin();
	if (currentDemo) {
		currentDemo->draw();
	}
	renderMainImGui();
	gui.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (currentDemo) {
		currentDemo->keyPressed(key);
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	if (currentDemo) {
		currentDemo->mousePressed(x, y, button);
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
