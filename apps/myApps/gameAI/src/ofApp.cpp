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
	font.load("verdana.ttf", 200);

	currentDemo = Goal_Oriented_Action_Planning;
	ofSetFrameRate(60);

	//Kinematic_Motion
	rigidbody.position.set(100, ofGetHeight() - 100);// start at the bottom left corner of the screen
	rigidbody.velocity.set(500, 0);//move to right
	lastBreadcrumbTime = ofGetElapsedTimef();

	//DynamicSteeringBehaviour

	characterForDynamicSteering.position.set(ofGetWidth() - 100, ofGetHeight() - 100);
	isSeeking = false;
	rigidbody2.character = &characterForDynamicSteering;

	//WanderBehaviour
	characterForWanderBehaviour.position.set(ofGetWidth() - 200, ofGetHeight() - 200);
	characterForWanderBehaviour.color.set(ofColor(255, 0, 0));
	rigidbody3.character = &characterForWanderBehaviour;

	//FlockBehavior
	leader.position = ofVec2f(ofGetWidth() / 2, ofGetHeight() / 2);
	leader.color.set(ofColor(0, 100, 100));
	leader.maxSpeed = 100;
	leaderWander.character = &leader;

	for (int i = 0; i < 10; i++) {
		Character character;
		character.position = ofVec2f(ofRandomWidth(), ofRandomHeight());
		character.maxSpeed = 120;
		characters.push_back(character);
	}
	for (auto& character : characters) {
		targets.push_back(&character);
	}
	flockBehaviors.resize(characters.size());
	for (size_t i = 0; i < characters.size(); i++) {
		flockBehaviors[i] = std::make_unique<FlockBehavior>(&characters[i], &targets, &leader, 200, 90.0f, 100, 100);
		
	}
	//Path_finding_small_Map
	graphSmall.generateSmallGraph();
	
	//Path_finding_large_Map
	graphLarge.generateLargeGraph(1000, 2000, 1.0f);

	//PathFinding_Indoor_Environment

	graphIndoor.generateIndoorEnvironmentGraph();

	collisionList.push_back(PolygonCollision(ofVec2f(0.0, 0.3), ofVec2f(0.15, 0.3), ofVec2f(0.15, 0.17), ofVec2f(0.17, 0.17), ofVec2f(0.17, 0.33), ofVec2f(0.0, 0.33)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.15, 0.13), ofVec2f(0.15, 0), ofVec2f(0.17, 0), ofVec2f(0.17, 0.13)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.22, 0.3), ofVec2f(0.3, 0.3), ofVec2f(0.3, 0), ofVec2f(0.32, 0), ofVec2f(0.32, 0.33), ofVec2f(0.22, 0.33)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.0, 0.63), ofVec2f(0.17, 0.63), ofVec2f(0.17, 0.87), ofVec2f(0.14, 0.87), ofVec2f(0.14, 0.65), ofVec2f(0.0, 0.65)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.23, 1), ofVec2f(0.23, 0.57), ofVec2f(0.14, 0.57), ofVec2f(0.14, 0.54), ofVec2f(0.25, 0.54), ofVec2f(0.25, 1)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.1, 0.47), ofVec2f(0.1, 0.45), ofVec2f(0.27, 0.45), ofVec2f(0.27, 0.47)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.33, 1), ofVec2f(0.33, 0.53), ofVec2f(0.35, 0.53), ofVec2f(0.35, 1)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.77, 1), ofVec2f(0.77, 0.53), ofVec2f(0.79, 0.53), ofVec2f(0.79, 1)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.77, 0.35), ofVec2f(0.77, 0.13), ofVec2f(0.79, 0.13), ofVec2f(0.79, 0.27)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.79, 0.25), ofVec2f(0.85, 0.4), ofVec2f(0.88, 0.22), ofVec2f(0.88, 0.18), ofVec2f(0.85, 0.36), ofVec2f(0.79, 0.22)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.8, 0.36), ofVec2f(0.85, 0.56), ofVec2f(0.88, 0.42), ofVec2f(0.88, 0.46), ofVec2f(0.85, 0.6), ofVec2f(0.8, 0.42)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.77, 0.53), ofVec2f(0.8, 0.42), ofVec2f(0.82, 0.42), ofVec2f(0.79, 0.53)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.47, 0.3), ofVec2f(0.47, 0.47), ofVec2f(0.4, 0.47)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.53, 0.3), ofVec2f(0.53, 0.47), ofVec2f(0.67, 0.47)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.4, 0.53), ofVec2f(0.47, 0.53), ofVec2f(0.47, 0.67)));
	collisionList.push_back(PolygonCollision(ofVec2f(0.53, 0.53), ofVec2f(0.53, 0.67), ofVec2f(0.67, 0.53)));

	graphIndoor.collisionList = collisionList;

	characterForPathFinding.position.set(mapToScreenCoordinates(graphIndoor.vertices[1].getPosition()));
	characterForPathFinding.color.set(ofColor(0, 255, 0));
	characterForPathFinding.maxAcceleration = 200;
	characterForPathFinding.maxSpeed = 60;
	characterForPathFinding.radius = 10.0f;

	pathFind.character = &characterForPathFinding;
	pathFind.graph = graphIndoor;
	pathFind.collisionList = collisionList;

	int start = 1;
	int goal = 10;

	auto result = graphSmall.dijkstra(start, goal);
	auto came_from = result.first;
	path1 = graphSmall.getPath(goal, came_from);

	auto result1 = graphSmall.aStar(start, goal, manhattanDistance);
	came_from = result1.first;
	path12 = graphSmall.getPath(goal, came_from);

	auto result2 = graphSmall.aStar(start, goal, euclideanDistance);
	came_from = result2.first;
	path13 = graphSmall.getPath(goal, came_from);

	start = 1;
	goal = 150;

	auto result3 = graphLarge.dijkstra(start, goal);
	came_from = result3.first;
	path2 = graphLarge.getPath(goal, came_from);

	auto result4 = graphLarge.aStar(start, goal, manhattanDistance);
	came_from = result4.first;
	path22 = graphLarge.getPath(goal, came_from);

	//Desision Tree
	pathfinder = new PathFinder();
	wanderer = new RigidbodyForWander();
	dynamicer = new RigidbodyForDynamicSteering();

	A_stopFinding = new StopFindingAction();
	A_finding = new FindingAction();
	A_resetFindTarget = new ResetFindTargetAction();
	A_wander = new WanderAction();
	A_goBack = new GobackAction();
	A_speedUp = new SpeedUpAction();

	D_distanceWithTarget = new DistanceWithPathTargetDecision(A_resetFindTarget, A_stopFinding, pathfinder, 100);
	D_findTime = new FindTimeDecision(D_distanceWithTarget, A_finding, pathfinder, 5);
	D_speed_will_collide = new SpeedDecision(A_goBack, A_resetFindTarget, &characterForDecisionTrees, 50);
	D_speed_not_collide = new SpeedDecision(A_wander, A_speedUp, &characterForDecisionTrees, 50);
	D_ifCollisionForward = new IfCollisionForwardDecision(D_speed_will_collide, D_speed_not_collide, pathfinder, 100);
	D_ifFind = new IfFindingDecision(D_findTime, D_ifCollisionForward, pathfinder);

	characterForDecisionTrees.position.set(mapToScreenCoordinates(graphIndoor.vertices[1].getPosition()));
	characterForDecisionTrees.color.set(ofColor(0, 255, 0));
	characterForDecisionTrees.maxAcceleration = 200;
	characterForDecisionTrees.maxSpeed = 60;
	characterForDecisionTrees.radius = 10.0f;

	pathfinder->character = &characterForDecisionTrees;
	pathfinder->graph = graphIndoor;
	pathfinder->path = path4;
	pathfinder->collisionList = collisionList;

	wanderer->character = &characterForDecisionTrees;

	dynamicer->character = &characterForDecisionTrees;

	A_stopFinding->character = &characterForDecisionTrees;
	A_stopFinding->wanderer = wanderer;

	A_finding->character = &characterForDecisionTrees;
	A_finding->pathfinder = pathfinder;

	A_resetFindTarget->character = &characterForDecisionTrees;
	A_resetFindTarget->pathfinder = pathfinder;
	A_resetFindTarget->path = path4;
	A_resetFindTarget->graphIndoor = graphIndoor;

	A_wander->character = &characterForDecisionTrees;
	A_wander->wanderer = wanderer;

	A_goBack->character = &characterForDecisionTrees;
	A_goBack->dynamicer = dynamicer;

	A_speedUp->character = &characterForDecisionTrees;
	A_speedUp->dynamicer = dynamicer;

	//Behavior Tree
	player.position.set(mapToScreenCoordinates(graphIndoor.vertices[1].getPosition()));
	player.color.set(ofColor(255, 255, 255));
	player.maxAcceleration = 200;
	player.maxSpeed = 60;
	player.radius = 10.0f;

	pathFinderForPlayer.character = &player;
	pathFinderForPlayer.graph = graphIndoor;
	pathFinderForPlayer.path = playerPath;
	pathFinderForPlayer.collisionList = collisionList;

	monsterAI.position.set(mapToScreenCoordinates(graphIndoor.vertices[5].getPosition()));
	monsterAI.color.set(ofColor(0, 255, 0));
	monsterAI.maxAcceleration = 200;
	monsterAI.maxSpeed = 60;
	monsterAI.radius = 10.0f;

	pathFinderForMonster.character = &monsterAI;
	pathFinderForMonster.graph = graphIndoor;
	pathFinderForMonster.path = monsterPath;
	pathFinderForMonster.collisionList = collisionList;

	wandererForMonster.character = &monsterAI;
	dynamicerForMonster.character = &monsterAI;

	//
	// 树的构建
	// 
	// CanSeePlayer 分支
	auto canSeePlayerSequence = new BTSequenceNode();
	canSeePlayerSequence->addChild(new BTActionNode(new BTMonsterChangeColorAction(ofColor::red)));
	canSeePlayerSequence->addChild(new BTActionNode(new BTMonsterSayFindUAction("I find you")));
	canSeePlayerSequence->addChild(new BTActionNode(new BTMonsterGoTowardsPlayerAction()));

	//auto closeEnoughToEat = new IsCloseToPlayer(new BTActionNode(new BTMonsterEatAction()), 10.0f);  // 假设10单位距离为吃的范围
	//canSeePlayerSequence->addChild(closeEnoughToEat);

	auto canSeePlayer = new CanSeePlayer(canSeePlayerSequence);

	// IsCloseToPlayer 分支
	auto isCloseToPlayerSequence = new BTSequenceNode();
	isCloseToPlayerSequence->addChild(new BTActionNode(new BTMonsterChangeColorAction(ofColor::blue)));
	isCloseToPlayerSequence->addChild(new BTActionNode(new BTMonsterSayHearUAction("I hear you")));
	isCloseToPlayerSequence->addChild(new BTActionNode(new BTCalculatePathAction()));
	isCloseToPlayerSequence->addChild(new BTActionNode(new BTFollowPathAction()));

	//auto closeEnoughToEat2 = new IsCloseToPlayer(new BTActionNode(new BTMonsterEatAction()), 10.0f);
	//isCloseToPlayerSequence->addChild(closeEnoughToEat2);

	auto isCloseToPlayer = new IsCloseToPlayer(isCloseToPlayerSequence, 200.0f);  // 假设30单位距离为听到的范围

	// IsWanderTimeExceeded 分支
	auto isWanderTimeExceededSequence = new BTSequenceNode();
	isWanderTimeExceededSequence->addChild(new BTActionNode(new BTMonsterChangeColorAction(ofColor::green)));
	isWanderTimeExceededSequence->addChild(new BTActionNode(new BTMonsterSayFindUAction("Where are you?")));
	isWanderTimeExceededSequence->addChild(new BTActionNode(new BTResetFindTargetAction()));
	isWanderTimeExceededSequence->addChild(new BTActionNode(new BTFollowPathAction()));

	auto isWanderTimeExceeded = new IsWanderTimeExceeded(isWanderTimeExceededSequence, 5.0f);  // 假设300秒为徘徊时间阈值

	// 根选择器
	auto rootSelector = new BTSelectorNode();
	rootSelector->addChild(canSeePlayer);
	rootSelector->addChild(isCloseToPlayer);
	rootSelector->addChild(isWanderTimeExceeded);
	rootSelector->addChild(new BTActionNode(new BTFollowPathAction()));  // 直接执行跟随路径

	// 根任务
	rootTask = new RootTask(rootSelector);
	BehaviourTree monsterBehaviourTree(rootTask, &blackboard);

	//
	//Blackboard 的构建初始化
	//
	// 初始化Blackboard数据
	blackboard.set("monster", &monsterAI);
	blackboard.set("player", &player);
	blackboard.set("obstacles", &collisionList);
	blackboard.set("graphIndoor", &graphIndoor);
	blackboard.set("pathFinder", &pathFinderForMonster);
	blackboard.set("monsterWanderTime", &monsterWanderTime);
	blackboard.set("monsterBirthPlace", monsterAI.position);
	blackboard.set("playerBirthPlace", player.position);
	blackboard.set("dynamicer", &dynamicerForMonster);
	blackboard.set("monsterSayingFindUTime", &monsterSayFindTime);
	blackboard.set("monsterSayingHearUTime", &monsterSayHearTime);

	monsterAIBT = new BehaviourTree(rootTask, &blackboard);


	// ---------------------------------------------------------------------------------------
	// 
	//Goal_Oriented_Action_Planning
	// 
	// ---------------------------------------------------------------------------------------

	// GoapAgent
	GoapStatus initialState;
	initialState.set("hasGun", false);  // 假设初始状态角色没有枪
	initialState.set("findMonster", false);
	initialState.set("hearMonster", false);
	initialState.set("seeMonster", false);

	GoapStatus goalState;
	goalState.set("success", true);  // 目标是获取枪

	playerAgentforGoap.setCurrentState(initialState);
	playerAgentforGoap.setGoalState(goalState);
	//

	//
	sensorforGoap = new GoapSensor(&playAIforGoap, &monsterAIforBT2, &collisionList, playerAgentforGoap.currentState);
	//
	
	//
	pathFinderForMonsterforBT2.character = &monsterAIforBT2;
	pathFinderForMonsterforBT2.graph = graphIndoor;
	pathFinderForMonsterforBT2.path = monsterPathBT2;
	pathFinderForMonsterforBT2.collisionList = collisionList;
	dynamicerForMonsterforBT2.character = &monsterAIforBT2;
	//

	BBforBT2.set("monster", &monsterAIforBT2);
	BBforBT2.set("player", &playAIforGoap);
	BBforBT2.set("obstacles", &collisionList);
	BBforBT2.set("graphIndoor", &graphIndoor);
	BBforBT2.set("pathFinder", &pathFinderForMonsterforBT2);
	BBforBT2.set("monsterWanderTime", &monsterWanderTimeforBT2);
	BBforBT2.set("monsterBirthPlace", monsterAIforBT2.position);
	BBforBT2.set("playerBirthPlace", playAIforGoap.position);
	BBforBT2.set("dynamicer", &dynamicerForMonsterforBT2);
	BBforBT2.set("monsterSayingFindUTime", &monsterSayFindTimeforBT2);
	BBforBT2.set("monsterSayingHearUTime", &monsterSayHearTimeforBT2);

	monsterAIBT2 = new BehaviourTree(rootTask, &BBforBT2);

	//playAIforGoap
	// ---------------------------------------------------------------------------------------
	playAIforGoap.position.set(mapToScreenCoordinates(graphIndoor.vertices[1].getPosition()));
	playAIforGoap.color.set(ofColor(255, 255, 255));
	playAIforGoap.maxAcceleration = 200;
	playAIforGoap.maxSpeed = 60;
	playAIforGoap.radius = 10.0f;

	pathFinderForplayAIforGoap.character = &playAIforGoap;
	pathFinderForplayAIforGoap.graph = graphIndoor;
	pathFinderForplayAIforGoap.path = playerPathGoap;
	pathFinderForplayAIforGoap.collisionList = collisionList;
	// ---------------------------------------------------------------------------------------
	
	//MonsterAIforBT2
	// ---------------------------------------------------------------------------------------
	monsterAIforBT2.position.set(mapToScreenCoordinates(graphIndoor.vertices[10].getPosition()));
	monsterAIforBT2.color.set(ofColor(0, 255, 0));
	monsterAIforBT2.maxAcceleration = 200;
	monsterAIforBT2.maxSpeed = 60;
	monsterAIforBT2.radius = 10.0f;
	// ---------------------------------------------------------------------------------------

	//Gun
	gun.itemHasPickUp = false;
	gun.tag = "gun";
	gun.position.set(100, ofGetHeight() - 100);
	//
	
	//ActionforGoap
	// ---------------------------------------------------------------------------------------
	
	GoapGoToTargetAction* goToGetGun = new GoapGoToTargetAction("goToGetGun", 10.0f, "Gun", &pathFinderForplayAIforGoap, &gun);
	goToGetGun->addPrecondition("hasGun", false);
	goToGetGun->addPrecondition("findMonster", true);
	goToGetGun->addEffect("hasGun", true);
	playerAgentforGoap.addAction(goToGetGun);

	GoapGoRandomAction* smallRangeRandom = new GoapGoRandomAction("smallRangeRandom", 10.0f, 150.0f, &pathFinderForplayAIforGoap);
	smallRangeRandom->addPrecondition("hasGun", false);
	smallRangeRandom->addPrecondition("seeMonster", false);
	smallRangeRandom->addPrecondition("hearMonster", false);
	smallRangeRandom->addEffect("seeMonster", true);
	smallRangeRandom->addEffect("hearMonster", true);
	playerAgentforGoap.addAction(smallRangeRandom);

	GoapFindMonsterAction* findMonster = new GoapFindMonsterAction("findMonster", 10.0f);
	findMonster->addPrecondition("findMonster", false);
	findMonster->addPrecondition("seeMonster", true);
	findMonster->addEffect("findMonster", true);
	playerAgentforGoap.addAction(findMonster);

	GoapGoToTargetAction* goToMonster = new GoapGoToTargetAction("goToMonster", 10.0f, "Monster", &pathFinderForplayAIforGoap, &monsterAIforBT2);
	goToMonster->addPrecondition("hasGun", true);
	goToMonster->addPrecondition("findMonster", true);
	goToMonster->addEffect("seeMonsterWithGun", true);
	playerAgentforGoap.addAction(goToMonster);

	GoapShootAction* shootMonster = new GoapShootAction("shootMonster", 10.0f, &playAIforGoap, &monsterAIforBT2);
	shootMonster->addPrecondition("seeMonsterWithGun", true);
	shootMonster->addPrecondition("hasGun", true);
	shootMonster->addEffect("MonsterDie", true);
	playerAgentforGoap.addAction(shootMonster);

	GoapDanceAction* dance = new GoapDanceAction("dance", 10.0f, &playAIforGoap);
	dance->addPrecondition("MonsterDie", true);
	dance->addEffect("success", true);
	playerAgentforGoap.addAction(dance);
	// ---------------------------------------------------------------------------------------
	


}

//--------------------------------------------------------------
void ofApp::update(){
	switch (currentDemo) {
		case Kinematic_Motion: {
			float currentTime = ofGetElapsedTimef();
			if (currentTime - lastBreadcrumbTime > 0.5) {
				breadcrumbs.push_back(rigidbody.position);
				lastBreadcrumbTime = currentTime;
			}

			//update position
			rigidbody.update(1.0 / 60.0);

			//change the direction
			if (rigidbody.position.x > ofGetWidth() - 100 && rigidbody.velocity.x > 0) {
				rigidbody.velocity.set(0, -500); // up
			}
			else if (rigidbody.position.y < 100 && rigidbody.velocity.y < 0) {
				rigidbody.velocity.set(-500, 0); // left
			}
			else if (rigidbody.position.x < 100 && rigidbody.velocity.x < 0) {
				rigidbody.velocity.set(0, 500); // down
			}
			else if (rigidbody.position.y > ofGetHeight() - 100 && rigidbody.velocity.y > 0) {
				rigidbody.velocity.set(500, 0); // right
			}
			break;
		}

		case Dynamic_Steering_Behaviours:
			if (isSeeking) {
				SteeringOutput steering = rigidbody2.getSteering(mouseTarget);
				rigidbody2.update(steering, 1.0 / 60.0);
			}
			break;

		case Wander_Steering_Behaviours: {
			SteeringOutput steering = rigidbody3.getSteering();
			rigidbody3.update(steering, 1.0f / 60.0f); // Update wanderer with steering output

			characterForWanderBehaviour.toroidalMap();
			break;
		}
		
		case Flocking_Behavior_and_Blending: {
			SteeringOutput steering = leaderWander.getSteering();
			leaderWander.update(steering, 1.0f / 60.0f); // Update wanderer with steering output
			//leader.toroidalMap();

			for (size_t i = 0; i < characters.size(); i++) {
				SteeringOutput steering = flockBehaviors[i]->getSteering();
				flockBehaviors[i]->update(steering, 1.0f / 60.0f);
				//characters[i].toroidalMap();
			}
			break;
		}
		
		case Path_finding_small_Map_Dijkstra: {
			break;
		}
		case Path_finding_small_Map_AStar_manhattanDistance: {
			break;
		}
		case Path_finding_small_Map_AStar_euclideanDistance: {
			break;
		}
		case Path_finding_large_Map_Dijkstra: {
			break;
		}
		case Path_finding_large_Map_AStar_manhattanDistance: {
			break;
		}
		case Path_finding_indoor_Environment: {
			
			float currentTime = ofGetElapsedTimef();
			if (currentTime - lastBreadcrumbTime1 > 0.5) {
				characterForPathFinding.breadcrumbs.push_back(characterForPathFinding.position);
				lastBreadcrumbTime1 = currentTime;
			}

			if (isPathFinding) {
				SteeringOutput steering = pathFind.getSteering();
				pathFind.update(steering, 1.0f / 60.0f);
			}

			break;
		}
		case Decision_Trees: {
			DecisionTreeNode* currentNode = D_ifFind;
			DecisionTreeNode* actionNode = currentNode->makeDecision();
			if (Action* action = dynamic_cast<Action*>(actionNode)) {
				action->performAction();
			}
			break;
		}
		case Behavior_Trees: {
			if (isPlayerPathFinding) {
				SteeringOutput steering = pathFinderForPlayer.getSteering();
				pathFinderForPlayer.update(steering, 1.0f / 60.0f);
			}
			Status result = monsterAIBT->makeDecision();
			break;
		}
		case Goal_Oriented_Action_Planning: {
			playerAgentforGoap.update();
			Status result = monsterAIBT2->makeDecision();
			sensorforGoap->updateSensoryInformation();
			break;
		}
	}

}

//--------------------------------------------------------------
void ofApp::draw(){
	
	switch (currentDemo) {

		//case Kinematic_Motion:
		//	ofBackground(0);
		//	for (auto& pos : breadcrumbs) {
		//		ofDrawCircle(pos, 5);
		//	}

		//	rigidbody.draw();
		//	break;
		case Dynamic_Steering_Behaviours:
			ofBackground(0);
			characterForDynamicSteering.draw();
			ofSetColor(0, 255, 0);
			ofDrawBitmapString("Click mouse to set the position of target", 50, 100);
			break;
		//case Wander_Steering_Behaviours:
		//	ofBackground(0);
		//	characterForWanderBehaviour.draw();
		//	break;
		//case Flocking_Behavior_and_Blending:
		//	ofBackground(0);

		//	ofSetColor(leader.color);
		//	leader.drawToroidalMap();

		//	ofSetColor(ofColor::white);
		//	for (auto& character : characters) {
		//		character.drawToroidalMap();
		//	}
		//	break;
		case Path_finding_small_Map_Dijkstra: 
			ofBackground(0);
			graphSmall.drawMap();
			graphSmall.drawPath(path1);
			break;
		case Path_finding_small_Map_AStar_manhattanDistance:
			graphSmall.drawMap();
			graphSmall.drawPath(path12);
			break;
		case Path_finding_small_Map_AStar_euclideanDistance:
			graphSmall.drawMap();
			graphSmall.drawPath(path13);
			break;
		case Path_finding_large_Map_Dijkstra:
			graphLarge.drawMap();
			graphLarge.drawPath(path2);
			break;
		case Path_finding_large_Map_AStar_manhattanDistance:
			graphLarge.drawMap();
			graphLarge.drawPath(path22);
			break;
		case Path_finding_indoor_Environment:
			graphIndoor.drawMap();
			if(isPathFinding) graphIndoor.drawPath(path3);

			//Draw collision
			for (const auto& polygon : collisionList) {
				polygon.drawPolygon();
			}
			characterForPathFinding.draw();
			characterForPathFinding.drawBreadcrumbs();

			ofSetColor(0, 255, 0);
			ofDrawBitmapString("Click mouse to set the position of target", 50, 70);

			break;
		case Decision_Trees:
			//Draw collision
			ofBackground(0);
			for (const auto& polygon : collisionList) {
				polygon.drawPolygon();
			}
			characterForDecisionTrees.draw();

			//ofSetColor(0, 255, 0);
			break;
		case Behavior_Trees:
			ofBackground(0);
			for (const auto& polygon : collisionList) {
				polygon.drawPolygon();
			}
			player.draw();
			monsterAI.draw();

			ofSetColor(ofColor::yellow);
			ofDrawBitmapString("Click to move the player!\n", 30, 60);

			break;
		case Goal_Oriented_Action_Planning:
			ofBackground(0);
			for (const auto& polygon : collisionList) {
				polygon.drawPolygon();
			}
			gun.draw();
			monsterAIforBT2.draw();
			playAIforGoap.draw();
			break;
	}

	ofSetColor(ofColor::green);
	string introText = "Press '1' for Decision_Trees\n";
	introText += "Press '2' for Behavior_Trees\n";
	introText += "Press '3' for Goal_Oriented_Action_Planning\n";
	ofDrawBitmapString(introText, 10, 15);
	
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
		//case '1':
		//	currentDemo = Kinematic_Motion;
		//	break;
		//case '9':
		//	currentDemo = Dynamic_Steering_Behaviours;
		//	break;
		//case '3':
		//	currentDemo = Wander_Steering_Behaviours;
		//	break;
		//case '4':
		//	currentDemo = Flocking_Behavior_and_Blending;
		//	break;
		/*case '1':
			currentDemo = Path_finding_small_Map_Dijkstra;
			break;
		case '2':
			currentDemo = Path_finding_small_Map_AStar_manhattanDistance;
			break;
		case '3':
			currentDemo = Path_finding_small_Map_AStar_euclideanDistance;
			break;
		case '4':
			currentDemo = Path_finding_large_Map_Dijkstra;
			break;
		case '5':
			currentDemo = Path_finding_large_Map_AStar_manhattanDistance;
			break;
		case '6':
			currentDemo = Path_finding_indoor_Environment;
			break;*/
		case '1':
			currentDemo = Decision_Trees;
			break;
		case '2':
			currentDemo = Behavior_Trees;
			break;
		case '3':
			currentDemo = Goal_Oriented_Action_Planning;
			break;

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
	switch (currentDemo) {
	case Kinematic_Motion:

		break;
	case Dynamic_Steering_Behaviours:
		mouseTarget.set(x, y);
		isSeeking = true;
		break;
	case Wander_Steering_Behaviours:

		break;
	case Flocking_Behavior_and_Blending:

		break;
	case Path_finding_indoor_Environment: {

		ofVec2f targetPoint = pathFind.onMouseClick(x, y);

		float normalizedX = static_cast<float>(targetPoint.x) / ofGetWidth();
		float normalizedY = static_cast<float>(targetPoint.y) / ofGetHeight();
		float normalizedCharacterX = static_cast<float>(characterForPathFinding.position.x) / ofGetWidth();
		float normalizedCharacterY = static_cast<float>(characterForPathFinding.position.y) / ofGetHeight();

		pathFind.targetPosition.set(targetPoint.x, targetPoint.y);
		pathFind.startNode = graphIndoor.findNearestReachableNodeToClick(characterForPathFinding.position.x, characterForPathFinding.position.y);
		pathFind.goalNode = graphIndoor.findNearestReachableNodeToClick(targetPoint.x, targetPoint.y);
		pathFind.currentNodeIndex = 0;
		pathFind.nextNodeIndex = 1;

		auto result = pathFind.graph.aStar(pathFind.startNode, pathFind.goalNode, manhattanDistance);
		auto came_from = result.first;
		path3 = graphLarge.getPath(pathFind.goalNode, came_from);
		pathFind.path = path3;
		isPathFinding = true;
		break;
	}
	case Behavior_Trees: {
		ofVec2f targetPoint = pathFinderForPlayer.onMouseClick(x, y);

		float normalizedX = static_cast<float>(targetPoint.x) / ofGetWidth();
		float normalizedY = static_cast<float>(targetPoint.y) / ofGetHeight();
		float normalizedCharacterX = static_cast<float>(player.position.x) / ofGetWidth();
		float normalizedCharacterY = static_cast<float>(player.position.y) / ofGetHeight();

		pathFinderForPlayer.targetPosition.set(targetPoint.x, targetPoint.y);
		pathFinderForPlayer.startNode = graphIndoor.findNearestReachableNodeToClick(player.position.x, player.position.y);
		pathFinderForPlayer.goalNode = graphIndoor.findNearestReachableNodeToClick(targetPoint.x, targetPoint.y);
		pathFinderForPlayer.currentNodeIndex = 0;
		pathFinderForPlayer.nextNodeIndex = 1;

		auto result = pathFinderForPlayer.graph.aStar(pathFinderForPlayer.startNode, pathFinderForPlayer.goalNode, manhattanDistance);
		auto came_from = result.first;
		playerPath = graphLarge.getPath(pathFinderForPlayer.goalNode, came_from);
		pathFinderForPlayer.path = playerPath;
		isPlayerPathFinding = true;
		break;
	}
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
