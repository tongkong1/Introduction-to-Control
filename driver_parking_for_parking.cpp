/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Description: User Module for CyberParking							█
█	Author: Chenxi Yang													█
█	Email: yangchenxi@sjtu.edu.cn										█
█	Create: 2022.04.03							    					█
█	Salute to Xuangui Huang												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/


#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*/
//Do NOT modify the code below!

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	printf("OK!\n");
	return 0;
}

//Do Not modify the code above!
/*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	printf("carX %.6f  carY %.6f", _caryaw, _caryaw);
	printf("lotX %.6f  lotY %.6f", _lotAngle, _lotAngle);
}

double constrain(double lowerBoundary, double upperBoundary, double input) {
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	return input;
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█ Define your variables here											█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
static int nCrtStage = 0;	//Current stage

enum ListCrtStage {			//Stage list
	StageApproaching,
	StageKeepRight,	//车辆靠右行驶
	StageTurnLeft,
	StageEnterLot,
	StageLeaveLot,
	StageBackToRoad,
	StageLeave
};

static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█ Write your own code here												█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	float fDis2Lot = (_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY);//车辆到车位的距离Distance to the parking lot车辆到车位的距离
	float fVerticalDis2Lot;		//车辆到车位的垂线距离Vertical distance to the parking lot

	if (abs(_lotAngle) > (PI / 2 - 0.05) && abs(_lotAngle) < (PI / 2 + 0.05)) {
		fVerticalDis2Lot = abs(_carX - _lotX);
	}
	else {
		float k = tan(_lotAngle);
		float b = -k * _lotX + _lotY;
		fVerticalDis2Lot = abs(k * _carX - _carY + b) / sqrt(k * k + 1);
	}

	printf("Stage: %d\n", nCrtStage);		//You can see the stage in the Console
	switch (nCrtStage)
	{
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Approaching the parking lot when far from it							█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageApproaching://初态
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14);
		*cmdGear = 1;
		*cmdAcc = 1.0; *cmdBrake = 0.0;
		if (fDis2Lot < 4000)	//5000->4000
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Keep right to get more place to turn									█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageKeepRight: //车辆靠右行驶
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - atan2(_midline[6][0] + _width / 3, _midline[6][1])) / 3.14);
		*cmdGear = 1;
		if (_speed > 20) {
			*cmdAcc = 0.0; *cmdBrake = 1.0;
		}
		else {
			*cmdAcc = 0.1; *cmdBrake = 0.0;
		}
		if (fVerticalDis2Lot < 7.2)	//讨论改的7.2
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Turn left into the parking lot										█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageTurnLeft://车向左转
		*cmdSteer = 1.0;
		*cmdGear = 1;
		if (_speed > 15) {
			*cmdAcc = 0.0; *cmdBrake = 0.3;
		}
		else {
			*cmdAcc = 0.2; *cmdBrake = 0.0;
		}
		if (fDis2Lot < 30)
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Drive straight into the parking lot									█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageEnterLot://车头驶入车位
		*cmdSteer = constrain(-1.0, 1.0, -50 * (_lotAngle + _caryaw) / 3.14);
		*cmdGear = 1;
		if (_speed > 5) {
			*cmdAcc = 0.01; *cmdBrake = 0.2;
		}
		else if (_speed > 0.5) {
			*cmdAcc = 0.02; *cmdBrake = 0.01;	//以下讨论改的
		}
		//else if (_speed > 0.05) {
			//*cmdAcc = 0.01; *cmdBrake = 0.0;
		//}
		else {
			if (fDis2Lot < 2 && _speed < 0.05) {
				//*cmdBrake = 1.0;
				//for (int i = 0; i < 100; i++) {

				//}
				*bFinished = true;
				nCrtStage++;
				//*cmdBrake = 0.1;
			}

		}
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Drive straight out of the parking lot									█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageLeaveLot://出车位
		*cmdSteer = constrain(-1.0, 1.0, -3 * (_lotAngle + _caryaw) / 3.14);	//30->3 刚出来少转弯
		*cmdGear = -1;
		if (_speed > 5) {
			*cmdAcc = 0.1; *cmdBrake = 0.1;
		}
		else {
			*cmdAcc = 0.2; *cmdBrake = 0.0;
		}
		if (fDis2Lot > 10)
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Turn left back to the main road										█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageBackToRoad://车转回来返回道路
		*cmdSteer = 1;
		*cmdGear = -1;
		if (_speed > 5) {
			*cmdAcc = 0; *cmdBrake = 0.1;
		}
		else {
			*cmdAcc = 0.1; *cmdBrake = 0;
		}
		if (fVerticalDis2Lot > 5)	//7.2->5 早点停下准备冲过终点
			nCrtStage++;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Leave the parking lot													█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	case StageLeave://冲过终点
		*cmdSteer = constrain(-1.0, 1.0, (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14);
		*cmdGear = 1;
		*cmdAcc = 1; *cmdBrake = 0;
		break;
		/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
		█ Enjoy!																█
		\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	default:
		break;
	}
}



