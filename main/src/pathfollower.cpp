#include "pathfollower.h"
#include "path.h"

#include <stdio.h>
#include <math.h>

#include "SimPID.h"
#include "shiftlib.h"

using namespace shiftlib;


PathFollower::PathFollower(float nDistanceError, float nMaxTurnError, SimPID *nDrivePID, SimPID *nTurnPID, SimPID *nFinalTurnPID)
{
	posX = posY = 0;
	lastX = lastY = 0;
	nextPoint = 0;
	leftSpeed = rightSpeed = 0;
	distanceError = nDistanceError;
	maxTurnError = nMaxTurnError;
	drivePID = nDrivePID;
	turnPID = nTurnPID;
	finalTurnPID = nFinalTurnPID;
	
	invertDrive = 1;
	invertTurn = 1;
	isDegrees = false;
	drivePID->setDesiredValue(0);
}

void PathFollower::initPath(Path *nPath, PathDirection nDirection, float nFinalAngleDegrees){

	path = nPath;
	direction = nDirection;
	nextPoint = 1;
	finalAngle = deg2rad(nFinalAngleDegrees);

	done = false;
	driveDone = false;
}

void PathFollower::driveToPoint(void){
	//drives to a given point
	//calculates drive output for distance and turn
	//turns towards given point and drives
	//total speed given by PID to endpoint, not target point

	int* nextCoordinate = path->getPoint(nextPoint);
	float desiredAngle = atan2((float)(nextCoordinate[1] - posY), (float)(nextCoordinate[0] - posX));

	if (direction == PathBackward)
	{
		desiredAngle += PI;
	}

	turnPID->setDesiredValue(normalizeRad(desiredAngle));
	turnSpeed = invertTurn*turnPID->calcPID(angle);

	printf("driving to point (%d,%d,%d) at (%d,%d)\n", nextCoordinate[0], nextCoordinate[1], nextPoint, posX, posY);
	printf("das-ang: %f Err: %f out: %f\n", desiredAngle, turnPID->getError(), turnSpeed);
	printf("das-drvv Err: %f out: %f\n", drivePID->getError(), driveSpeed);
}


void PathFollower::setSpeed(float nMaxSpeed){
	maxSpeed = nMaxSpeed;
}


void PathFollower::pickNextPoint(void){
	distanceToPoint = sqrt((float)((SQ(path->getPoint(nextPoint)[0]-posX) + SQ(path->getPoint(nextPoint)[1]-posY))));

	if(distanceToPoint < distanceError && nextPoint == path->size - 1)
			driveDone = true;
	if(distanceToPoint < distanceError && nextPoint + 1 != path->size)
	{
		nextPoint = nextPoint + 1;

	}
}

int PathFollower::followPathByPos(int nX, int nY, float nAngle, float &nLeftSpeed, float &nRightSpeed){
	angle = (isDegrees ? deg2rad(nAngle) : nAngle); 
	posX  = nX;
	posY  = nY;

	return followPath(nLeftSpeed, nRightSpeed);
}

int PathFollower::followPathByEnc(int32_t leftEncoder, int32_t rightEncoder, float nAngle, float &nLeftSpeed, float &nRightSpeed){
	//following is acheived by using the drive to point function
	//the robot attempts to drive to a point and when it gets close we move the point
	//to the next point on the path
	//speed is determined by distance from the ENDPOINT not intermediate points
	//use drive to point to do the calculations, this function just controls the targets

	angle = (isDegrees ? deg2rad(nAngle) : nAngle); 


	//get new position
	updatePos(leftEncoder, rightEncoder, nAngle);

	return followPath(nLeftSpeed, nRightSpeed);
}

int PathFollower::followPath(float &nLeftSpeed, float &nRightSpeed){
	driveError = sqrt((float)((SQ(path->getEndPoint()[0]-posX) + SQ(path->getEndPoint()[1]-posY))));
	pickNextPoint();

	//get drive speed and turn speed
	if(path->getPathDistance(nextPoint) > driveError)
	{
		driveError = path->getPathDistance(nextPoint);
	}

	
	//set left drive and right drive variables
	//this automatically returns them

	if(driveDone == false)
	{
		driveSpeed = invertDrive*direction*drivePID->calcPID(driveError) * (1 - limit(fabs(turnPID->getError())/maxTurnError, 1.f));
		driveToPoint();

		nLeftSpeed = -turnSpeed + driveSpeed;
		nRightSpeed = -turnSpeed - driveSpeed;
	}
	else if(done == false){
		turnSpeed = invertTurn*driveToAngle();

		nLeftSpeed = -turnSpeed;
		nRightSpeed	= -turnSpeed;

		printf("das-ang: %f Err: %f out: %f\n", finalAngle, turnPID->getError(), turnSpeed);
		printf("das-drvv Err: %f out: %f\n", drivePID->getError(), driveSpeed);
		
		if(finalTurnPID->isDone())
			done = true;
	}
	else
	{
		nLeftSpeed = 0;
		nRightSpeed = 0;
		printf("Drive is done!");
	}

	//printf("angle: %f\tPosX: %d\tPosY: %d\n", rad2deg(angle), posX, posY);
	//printf("turn: %f\tdrive: %f\n", turnSpeed, driveSpeed);

	//ScaleDrive(nLeftSpeed, nRightSpeed);
	//return 1 if follow is complete, else 0
	return 0;
}

void PathFollower::ScaleDrive(float &left, float &right){
	float top = 1;
	if (fabs(left) > maxSpeed)
		top = fabs(left);
	if (fabs(right) > maxSpeed && fabs(right) > fabs(left))
		top = fabs(right);
	
	left  = left  / top * maxSpeed;
	right = right / top * maxSpeed;
}

void PathFollower::setInvertTurn(bool set){
	if(set)
		invertTurn = -1;
	else
		invertTurn = 1;
}

void PathFollower::setInvertDrive(bool set){
	if(set)
		invertDrive = -1;
	else
		invertDrive = 1;
}

void PathFollower::updatePos(int leftEncoder, int rightEncoder, float heading){
	//this function will update the robots position, is called periodically by followPath()
	int d = ((leftEncoder - lastLeftEncoder) + (rightEncoder - lastRightEncoder) ) / 2;

	posX += d * cos(deg2rad(heading));
	posY += d * sin(deg2rad(heading));

	lastLeftEncoder = leftEncoder;
	lastRightEncoder = rightEncoder;
}

void PathFollower::reset(void)
{
	posX = 0;
	posY = 0;
	nextPoint = 1;
	driveDone = false;
	done = false;
}

int PathFollower::getXPos(void){
	return posX;
}

int PathFollower::getYPos(void){
	return posY;
}

float PathFollower::getPathDistance(void){
	return path->getPathDistance(nextPoint);
}

float PathFollower::getLinearDistance(void){
	return driveError;
}

float PathFollower::driveToAngle(void){
	finalTurnPID->setDesiredValue(finalAngle);
	return finalTurnPID->calcPID(angle);
}

bool PathFollower::isDone()
{
	return done;
}

void PathFollower::setIsDegrees(bool set) { isDegrees = set; }
