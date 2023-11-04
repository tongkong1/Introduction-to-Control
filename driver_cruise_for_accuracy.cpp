/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	DLL接口部分，您可以跳过这部分不阅读											█
█	不要修改这个 #pragma region 中的任何代码!									█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< 不要修改这个 region 中的任何代码!
#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <ostream>
#include <fstream>

#include "class_Visualization.h"
#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

//Global variables for vehicle states
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm)
{
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}
#pragma endregion >>>

/*
	约束函数
*/
#pragma region <<< Boundaries of control	
double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}
#pragma endregion

/*
	曲率半径计算函数
*/
#pragma region <<< Radius of curvature
//		Given three points ahead, outputs a struct circle.				
//		{radius:[1,1000], sign{-1:left,1:right}							
typedef struct Circle
{
	double r;
	int sign;
}circle;

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b * f - e * c) / (b * d - e * a);
	y = (d * c - a * f) / (b * d - e * a);
	r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	return { r,sign };
}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	以下是核心控制程序													█
█	主要输入: _midline, _speed											█
█	次要输入: _yaw, _yawrate, _acc, _width, _rpm,	_gearbox			█
█	主要输出: *cmdAcc, *cmdBrake, *cmdSteer  							█
█	次要输出: *cmdGear 【本样例中已实现】									█
█	详细信息请参见用户手册												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*
	基于OpenCV的可视化工具，详情请见文档
*/
cls_VISUAL cls_visual;

/*
	换挡程序
*/
#pragma region <<< Manual Gear
const float fGearShift[2][7] = //0 for downshift, 1 for upshift
{
	0,92,128,167,208,233,255,
	0,105,145,182,224,249,272
};
void updateGear(int* cmdGear)
{

	if (_speed > fGearShift[1][_gearbox] && _gearbox < 7) //upshift
	{
		*cmdGear = _gearbox + 1;
	}
	else if (_speed < fGearShift[0][_gearbox - 1] && _gearbox > 1) //downshift
	{
		*cmdGear = _gearbox - 1;
	}
	else
	{
		*cmdGear = _gearbox;
	}
}
#pragma endregion >>>

/*
	PID控制器
*/
class PIDController
{
private:
	double kp, ki, kd;		// PID控制器的参数
	double targetValue;		// 目标值
	double lastError;		// 上一次误差值
	double errorIntegral;	// 误差积分值

public:
	void initial(double p, double i, double d, double target)
	{
		kp = p;
		ki = i;
		kd = d;
		targetValue = target;
		lastError = 0;
		errorIntegral = 0;
	}

	void reset(double p, double i, double d)
	{
		kp = p;
		ki = i;
		kd = d;
	}

	double calculate(double input)
	{
		double error = targetValue - input;
		double derivative = error - lastError;
		errorIntegral += error;
		lastError = error;
		return kp * error + ki * errorIntegral + kd * derivative;
	}
};

PIDController speedController;	//速度PID控制
PIDController angleController;	//航向偏差PID控制
PIDController offsetController; //横向偏差PID控制
double lastTargetSpeed = 999.0;	//上一帧目标速度
bool isFirstFrame = true;		//第一帧标志
bool isRoadDetermined = false; //是否确定路面类型标志
bool isDefined = false; //是否更新参数
bool isDirtRoad = false; //是否是难走的路
bool stopAcc = false; // 停止加速
int counter = 0; //计时器
float fError = 0; //Error值

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	double currentoffsetError = _midline[0][0]; //当前横向偏差
	double targetAngleError = 0.0; //角度目标误差
	double targetOffsetError = 0.0; //目标横向偏差
	double currentAngleError = atan2(_midline[2][0], _midline[2][1]); //角度当前误差，默认为前方五米
	double targetSpeed = 999;  //目标车速
	double currentSpeed = _speed;	//当前车速

	/*
		初始化角度控制器
	*/
	if (isFirstFrame)
	{
		isFirstFrame = false;
		angleController.initial(9.7, 0, 0.1, targetAngleError); // 5.5, 0.0004, 50
		offsetController.initial(14, 0.9, 5.5, targetOffsetError); //18:165 60:165
		speedController.initial(10, 0.2, 50, 60);
		counter = 0;
	}


	/*
		曲率判断
	*/
	circle myCurve;
	float minCruve = 500.0;
	for (int fStep = 0; fStep < 65; fStep++)
	{
		myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 2][0], _midline[fStep + 2][1],
			_midline[fStep + 4][0], _midline[fStep + 4][1]);
		if (myCurve.r < minCruve)
		{
			minCruve = myCurve.r;
		}
	}

	/*
		舵角控制
	*/
	if (abs(_yawrate) > 0.94)
	{
		//打滑时稳住方向盘
		*cmdSteer = 0.07 * constrain(-1.0, 1.0, angleController.calculate(currentAngleError))
			+ 0.03 * constrain(-1.0, 1.0, offsetController.calculate(currentoffsetError));
	}
	else
	{
		//正常行驶
		*cmdSteer = 0.7 * constrain(-1.0, 1.0, angleController.calculate(currentAngleError))
			+ 0.3 * constrain(-1.0, 1.0, offsetController.calculate(currentoffsetError));
	}

	double test1 = 0.7 * constrain(-1.0, 1.0, angleController.calculate(currentAngleError));
	double test2 = 0.3 * constrain(-1.0, 1.0, offsetController.calculate(currentoffsetError));

	/*
		速度控制
	*/
	if (minCruve > 300)
		targetSpeed = 165;
	else if (minCruve > 180)
		targetSpeed = 140;
	else if (minCruve > 160)
		targetSpeed = 121;
	else if (minCruve > 140)
		targetSpeed = 106;
	else if (minCruve > 120)
		targetSpeed = 98;
	else if (minCruve > 100)
		targetSpeed = 90;
	else if (minCruve > 80)
		targetSpeed = 80;
	else if (minCruve > 60)
		targetSpeed = 65;
	else if (minCruve > 25)
		targetSpeed = 63;
	else if (minCruve > 14)
		targetSpeed = 43.5;
	else
		targetSpeed = 30;

	//每当目标速度变化时初始化PID控制器，重设参数，清空积分器和微分器
	if (counter < 200)
	{
		speedController.initial(10, 0.2, 50, 60);
		//起步pid
	}
	else if (targetSpeed != lastTargetSpeed)
	{
		//正常行驶pid
		speedController.initial(0.02, 0.75, 13, targetSpeed);
		lastTargetSpeed = targetSpeed;
	}


	//根据当前速度和目标速度关系，控制油门刹车以改变速度
	if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
	{
		/*
			起步阶段
		*/
		if (counter < 300 && !stopAcc)
		{
			*cmdAcc = constrain(0.3, 0.6, speedController.calculate(currentSpeed));
			if (minCruve < 45)
			{
				stopAcc = true;
			}
		}
		else
		{
			//根据方向盘和角速度综合考虑油门
			if (minCruve < 18)
			{
				*cmdAcc = constrain(0.1, 0.2, speedController.calculate(currentSpeed));
			}
			else if (abs(*cmdSteer) < 0.1)
			{
				if (abs(_yawrate) > 0.22)
				{
					*cmdAcc = constrain(0.0, 0.25, speedController.calculate(currentSpeed));
				}
				else
				{
					*cmdAcc = constrain(0.0, 0.39, speedController.calculate(currentSpeed));
				}
			}
			else if (abs(*cmdSteer) < 0.4)
			{
				if (abs(_yawrate) > 0.22)
				{
					*cmdAcc = constrain(0.0, 0.15, speedController.calculate(currentSpeed));
				}
				else
				{
					*cmdAcc = constrain(0.0, 0.25, speedController.calculate(currentSpeed));
				}
			}
			else
			{
				*cmdAcc = constrain(0.0, 0.08, speedController.calculate(currentSpeed));
			}
		}
		*cmdBrake = 0.0;
	}
	else
	{
		//根据曲率综合考虑刹车
		*cmdAcc = 0;
		if (minCruve > 250)
			*cmdBrake = 0.22;
		else if (minCruve > 200)
			*cmdBrake = 0.23;
		else if (minCruve > 150)
			*cmdBrake = 0.24;
		else if (minCruve > 100)
			*cmdBrake = 0.25;
		else if (minCruve > 70)
			*cmdBrake = 0.26;
		else if (minCruve > 45)
			*cmdBrake = 0.27;
		else
			*cmdBrake = 0.29;

		/*if (minCruve > 250)
			*cmdBrake = 0.32;
		else if (minCruve > 200)
			*cmdBrake = 0.31;
		else if (minCruve > 150)
			*cmdBrake = 0.3;
		else if (minCruve > 100)
			*cmdBrake = 0.29;
		else if (minCruve > 70)
			*cmdBrake = 0.28;
		else if (minCruve > 45)
			*cmdBrake = 0.27;
		else
			*cmdBrake = 0.26;*/

		/**cmdBrake = 0.3;*/
	}

	//更新档位
	updateGear(cmdGear);

	counter++;

	/*
		误差显示
	*/
	double lfToMiddle = sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);
	fError += abs(lfToMiddle) + abs((abs((4.8 / 2.0) * tan(_yaw)) + (1.8 / 2.0)) / (cos(_yaw))) - (1.8 / 2.0);
	printf("Error term: %lf", fError);

	////窗口可视化
	cls_visual.Fig2Y(3, 0, 300, 0, 500, 10, "Target V", targetSpeed, "Curvature", minCruve, "Current V", _speed);
	//cls_visual.Fig2Y(2, -3, 7, -3, 7, 1, "Offset", currentoffsetError, "Angle", currentAngleError);
	cls_visual.Fig2Y(4, 0, 500, -30, 70, 10, "fError", fError, "Angle", currentAngleError, "Offset", currentoffsetError);
	//cls_visual.Fig2Y(1, -1, 1, -1, 1, 10,"Angletest", test1, "Offsettest", test2, "Cmdsteer", *cmdSteer);
	//cls_visual.Fig2Y(5, -0.5, 0.5, -0.5, 0.5, 10, "cmdSteer", *cmdSteer, "_yawrate", _yawrate);
}
