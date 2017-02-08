#include "motion/path.h"
#include "motion/pathcurve.h"
#include "motion/pathfollower.h"
#include "motion/pathline.h"

#include <math.h>
#include <stdio.h>

#define PI 3.141592653589793f

PathCurve createPath(int targetX, int targetY, float targetNormal){
	PathCurve *curve;

	int start[2] = {0,0};
	int end[2] = {targetX, targetY};

	int C = 0.5*sqrt(pow(end[0], 2) + pow(end[1], 2));

	int c2[2] = {(int)(0.5*C), 0};
	int c3[2] = {(int)(end[0]+(C*cos(targetNormal))), (int)(end[1]+(C*sin(targetNormal)))};

	curve = new PathCurve(start,c2,c3,end,200);
	curve->show();
	return *curve;
}

int main(){
	int x(500), y(600);
	float normal = -PI;

	createPath(x, y, normal);

	return 0;
}
