#pragma once
#include <cmath>
#include <chrono>

#include "ofMain.h"
#include "KinematicMotion.h"
#include "WanderBehaviours.h"
#include "FlockingBehavior.h"
#include "Graph.h"
#include "BehaviorTree.h"

#include "GoapAction.h"
#include "GoapAgent.h"
#include "GoapSensor.h"
#include "GoapStatus.h"
#include "GoapDanceAction.h"
#include "GoapFindMonsterAction.h"
#include "GoapGoRandomAction.h"
#include "GoapGoToTargetAction.h"
#include "GoapShootAction.h"
class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
	public:
		ofTrueTypeFont font;

		enum DemoType {
			Kinematic_Motion,
			Dynamic_Steering_Behaviours,
			Wander_Steering_Behaviours,
			Flocking_Behavior_and_Blending,
			//Path_finding_small_Map,
			//Path_finding_large_Map,
			//Path_finding_indoor_Environment,
			Path_finding_small_Map_Dijkstra,
			Path_finding_small_Map_AStar_manhattanDistance,
			Path_finding_small_Map_AStar_euclideanDistance,
			Path_finding_large_Map_Dijkstra,
			Path_finding_large_Map_AStar_manhattanDistance,
			Path_finding_indoor_Environment,
			Decision_Trees,
			Behavior_Trees,
			Goal_Oriented_Action_Planning
		};

		DemoType currentDemo;

	//Kinematic_Motion Demo
	public:
		Rigidbody rigidbody;
		vector<ofVec2f> breadcrumbs;
		float lastBreadcrumbTime;
	
	//DynamicSteeringBehaviour
	public:
		Character characterForDynamicSteering;
		RigidbodyForDynamicSteering rigidbody2;
		ofVec2f mouseTarget;
		bool isSeeking;

	//WanderBehaviour
	public:
		Character characterForWanderBehaviour;
		RigidbodyForWander rigidbody3;

	//FlockingBehaviour
	public:
		std::vector<Character> characters;
		Character leader;
		RigidbodyForWander leaderWander;
		std::vector<Character*> targets;
		std::vector<std::unique_ptr<FlockBehavior>> flockBehaviors; //FlockBehavior Instance of everyone

	//Path_finding_small_Map_Dijkstra
	public:
		DirectedGraph graphSmall;
		std::vector<int> path1;


	//Path_finding_small_Map_AStar_manhattanDistance
	public:
		//DirectedGraph graphSmall;
		std::vector<int> path12;

	//Path_finding_small_Map_AStar_euclideanDistance
	public:
		//DirectedGraph graphSmall;
		std::vector<int> path13;

	//Path_finding_large_Map_Dijkstra
	public:
		DirectedGraph graphLarge;
		std::vector<int> path2;

	//Path_finding_large_Map_AStar_manhattanDistance
	public:
		//DirectedGraph graphLarge;
		std::vector<int> path22;

	//Path_finding_Indoor_environment
	public:
		bool isPathFinding = false;
		DirectedGraph graphIndoor;
		std::vector<PolygonCollision> collisionList;
		std::vector<int> path3;

		PathFinder pathFind;
		Character characterForPathFinding;

		float lastBreadcrumbTime1;

	//Decision_Trees
	public:
		Character characterForDecisionTrees;

		bool isPathFinding2 = false;
		std::vector<int> path4;

		PathFinder* pathfinder;
		RigidbodyForWander* wanderer;
		RigidbodyForDynamicSteering* dynamicer;

		StopFindingAction* A_stopFinding;
		FindingAction* A_finding;
		ResetFindTargetAction* A_resetFindTarget;
		WanderAction* A_wander;
		GobackAction* A_goBack;
		SpeedUpAction* A_speedUp;

		DistanceWithPathTargetDecision* D_distanceWithTarget;
		FindTimeDecision* D_findTime;
		SpeedDecision* D_speed_will_collide;
		SpeedDecision* D_speed_not_collide;
		IfCollisionForwardDecision* D_ifCollisionForward;
		IfFindingDecision* D_ifFind;
	//Behavior_Trees
	public:
		Blackboard blackboard;

		Character player;
		bool isPlayerPathFinding = false;
		std::vector<int> playerPath;
		PathFinder pathFinderForPlayer;

		Character monsterAI;
		bool isMonsterPathFinding = false;
		std::vector<int> monsterPath;
		PathFinder pathFinderForMonster;
		RigidbodyForWander wandererForMonster;
		RigidbodyForDynamicSteering dynamicerForMonster;

		RootTask* rootTask;
		BehaviourTree* monsterAIBT;

		float monsterSayFindTime = 0.0f;
		float monsterSayHearTime = 0.0f;
		float monsterWanderTime = 0.0f;

	//------------------------------------------------------------------------------------
	//Goal_Oriented_Action_Planning
	public:
		Character playAIforGoap;
		PathFinder pathFinderForplayAIforGoap;
		std::vector<int> playerPathGoap;
		GoapAgent playerAgentforGoap;
		GoapSensor* sensorforGoap;


		Character monsterAIforBT2;
		Blackboard BBforBT2;
		BehaviourTree* monsterAIBT2;
		PathFinder pathFinderForMonsterforBT2;
		std::vector<int> monsterPathBT2;
		RigidbodyForDynamicSteering dynamicerForMonsterforBT2;
		float monsterSayFindTimeforBT2 = 0.0f;
		float monsterSayHearTimeforBT2 = 0.0f;
		float monsterWanderTimeforBT2 = 0.0f;

		Character gun;
};