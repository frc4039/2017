#ifndef __PATH_FOLLOWER__
#define __PATH_FOLLOWER__

#include "path.h"
#include "pathline.h"
#include "pathcurve.h"

#include "SimPID.h"

#include <stdint.h>

#define PI 3.141592653589793f

typedef enum PathFollower_directions_enum{
	PathForward = 1,
	PathBackward = -1,
} PathDirection;

class PathFollower
{
private:
	int posX, posY;
	int lastX, lastY;
	int lastLeftEncoder, lastRightEncoder;
	int nextPoint;
	bool driveDone;
	bool done;
	bool isDegrees;
	float angle;
	float finalAngle;
	PathDirection direction;
	float distanceToEnd;
	float distanceToPoint;
	float distanceError;
	float turnError;
	float driveError;
	float maxSpeed;
	Path *path;
	float leftSpeed, rightSpeed;
	void driveToPoint(void);
	float driveToAngle(void);
	float distanceP;
	float turnP;
	float maxTurnError;
	float turnSpeed, driveSpeed;
	float normalize(float normalAngle);
	char invertTurn, invertDrive;
	int followPath(float &nLeftSpeed, float &nRightSpeed);

public:
	PathFollower(float nDistanceError, float nMaxTurnError, SimPID *nDrivePID, SimPID *nTurnPID, SimPID *nFinalTurnPID);
	void initPath(Path *nPath, PathDirection nDirection, float nFinalAngleDegrees);
	int followPathByEnc(int32_t leftEncoder, int32_t rightEncoder, float nAngle, float &nLeftSpeed, float &nRightSpeed);
	int followPathByPos(int nX, int nY, float nAngle, float &nLeftSpeed, float &nRightSpeed);
	void setSpeed(float nMaxSpeed);
	void updatePos(int leftEncoder, int rightEncoder, float heading);
	void reset(void);
	int getXPos(void);
	int getYPos(void);
	float getPathDistance(void);
	float getLinearDistance(void);
	void pickNextPoint(void);
	bool isDone(void);
	SimPID *turnPID;
	SimPID *drivePID;
	SimPID *finalTurnPID;
	void setInvertTurn(bool set);
	void setInvertDrive(bool set);
	void setIsDegrees(bool set);
	void ScaleDrive(float &left, float &right);
};

#endif
