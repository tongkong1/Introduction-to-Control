/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Description: User Module for CyberCruise							█
█	作者: 杨辰兮 & ChatGPT												█
█	联系方式: yangchenxi@sjtu.edu.cn										█
█	日期: 2023.02.13				    									█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	贴士:	您可以折叠 #pragma region 和	#pragma endregion 之间的代码		█
█	这可以使您获得一次性折叠完成的程序块而不是一个函数的能力					█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	DLL接口部分，您可以跳过这部分不阅读									█
█	不要修改这个 #pragma region 中的任何代码!								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< 不要修改这个 region 中的任何代码!
#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <ostream>
#include <fstream>
#include <algorithm>

#include "class_Visualization.h"
using namespace std;
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

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	上下确界约束函数									 					█
█	您需要理解它的功能，建议不要修改										█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
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

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	赛道曲率半径计算函数													█
█	您需要理解它的功能，建议不要修改										█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
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

//基于OpenCV的可视化工具，详情请见文档
cls_VISUAL cls_visual;

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	手动换挡程序															█
█	可以不用看懂，建议不要修改，除非您是学(Juan)霸(Wang) :P					█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< Manual Gear
const float fGearShift[2][7] = //0 for downshift, 1 for upshift
{
	0,75,115,170,205,220,235,
	0,105,150,180,220,249,272
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

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	PID控制器，由ChatGPT生成												█
█	可选择性修改，需要完全理解												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
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

	double calculate(double input)
	{
		double error = targetValue - input;
		double derivative = error - lastError;
		errorIntegral += error;
		lastError = error;
		return kp * error + ki * errorIntegral + kd * derivative;
	}
};


//竞速相关：
double Gradient_1 = 99999;
double Gradient_2 = 99999;
double Gradient = 99999;
double Lf = 0;




/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	车辆控制主程序，由ChatGPT自动生成助教完善								█
█	样例代码仅供参考，请在下方设计实现您的算法								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
PIDController speedController;	//速度PID控制
PIDController angleController;	//舵角PID控制
////////////////////PIDController hengController;   //横向PID控制

double lastTargetSpeed = 999.0;	//上一帧目标速度
bool isFirstFrame = true;		//第一帧标志
bool isCementRoad = true;      //路面判断
int timecounter = 0;           //计时器用来判断路面
bool flag = false;             //
double length(float x1, float y1, float x2, float y2) {   //
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	//先判断是否是城市路面
	timecounter++;
	if (timecounter < 68)
	{
		*cmdAcc = 1.0;
		*cmdBrake = 0.0;
	}
	if (timecounter == 68 && _speed < 40)
	{
		isCementRoad = false;             //若初始加速度很低，判断为dirt路面
		flag = true;
	}
	else if (timecounter == 68 && _speed >= 40)
	{
		isCementRoad = true;			//若初始加速度高，判断为城市路面
		flag = true;
	}


	//全局变量
	float fError = 0;
	double last_angle_error = 0.0;
	//以下程序放在userDriverSetParam函数中的任意位置，其中4.8为车长，1.8为车宽
	double lfToMiddle = sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);
	fError += abs(lfToMiddle) + abs((abs((4.8 / 2.0) * tan(_yaw)) + (1.8 / 2.0)) / (cos(_yaw))) - (1.8 / 2.0);
	printf("Error term: %lf", fError);

	//计算前方是直道还是弯道
	circle myCurve;
	float minCruve = 500.0;
	int fStep;
	int pin;               
	int pin_dirt;
	if (0.4 * _speed <= 35) pin = 35;
	else if (0.5 * _speed >= 150) pin = 150;	
	else pin = 0.45 * _speed;
	int step_high = 0.1 * _speed;
	int step_high_dirt = 0.1 * _speed;
	int step_low = 0.05 * _speed;
	int step_low_dirt = 0.05 * _speed;

	if (!isCementRoad)		//dirt路面
	{
		for (fStep = 0; fStep < 92; fStep++)
		{
			myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 3][0], _midline[fStep + 3][1], _midline[fStep + 4][0], _midline[fStep + 4][1]);
			if (myCurve.r < minCruve)
			{
				minCruve = myCurve.r;
			}
		}
	}
	else   //城市路面
	{
		for (fStep = 0; fStep < pin; fStep++)
		{
			myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + step_low][0], _midline[fStep + step_low][1], _midline[fStep + step_high][0], _midline[fStep + step_high][1]);
			if (myCurve.r < minCruve)
			{
				minCruve = myCurve.r;
			}
		}
	}

	
	/*预瞄控制切弯部分*/

	//double double_sight = 0.8 * _speed * _speed / (minCruve*3.6*3.6);
	//int sight = int(double_sight);

	/*int sight = 20;

	if (_midline[sight][0] - _midline[sight - 1][0])
	{
		Gradient_1 = (_midline[sight][1] - _midline[sight - 1][1]) / (_midline[sight][0] - _midline[sight - 1][0]);
	}
	else
	{
		Gradient_1 = 99999;
	}
	if (_midline[sight + 1][0] - _midline[sight][0])
	{
		Gradient_2 = (_midline[sight + 1][1] - _midline[sight][1]) / (_midline[sight + 1][0] - _midline[sight][0]);
	}
	else
	{
		Gradient_2 = 99999;
	}

	Gradient = 0.5 * abs(Gradient_1) + 0.5 * abs(Gradient_2);*/

	/* int sight = 0;
   if (speed > 120)
	   sight = 30;
   else if (speed > 70)sight = floor(speed / 5 + 6);
   else if (speed > 50)
	   sight = floor(speed / 2 - 15);
   else
	   sight = 10; */

	 /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	 █	预瞄控制切弯部分													   █
	 \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

	int sight = 20;
	if (!isCementRoad) {	//dirt路面
		sight = 26.5 + _width + 0.0033 * minCruve * minCruve;
	}
	else {		//城市路面
		sight = 21 + _width + 0.0030 * minCruve * minCruve;
	}

	double x1 = _midline[sight][0];
	double y1 = _midline[sight][1];
	double x2 = 0; // assuming _pos_x contains the current x coordinate of the car
	double y2 = 0; // assuming _pos_y contains the current y coordinate of the car

	circle direction = getR(_midline[sight - 1][0], _midline[sight - 1][1], _midline[sight][0], _midline[sight][1], _midline[sight + 1][0], _midline[sight + 1][1]);

	double slope1 = (_midline[sight][1] - _midline[sight - 1][1]) / (_midline[sight][0] - _midline[sight - 1][0]);
	double slope2 = (_midline[sight + 1][1] - _midline[sight][1]) / (_midline[sight + 1][0] - _midline[sight][0]);
	double slope = 0.5 * abs(slope1) + 0.5 * abs(slope2);
	double deltawidth = 1.5 * _width;
	double sinn = slope / sqrt(1 + slope * slope);
	double coss = 1 / sqrt(1 + slope * slope);

	if (direction.sign == -1) {
		if (x1 - x2 != 0)
		{
			Gradient_1 = (y1 - y2 /* + sinn * deltawidth*/) / (x1 - x2 /*- coss * deltawidth*/);
		}
		else
		{
			Gradient_1 = 99999;
		}
	}
	else {
		if (x1 - x2 != 0)
		{
			Gradient_1 = (y1 - y2 /* - sinn * deltawidth*/) / (x1 - x2 /*+ coss * deltawidth*/);
		}
		else
		{
			Gradient_1 = 99999;
		}
	}

	Gradient = abs(Gradient_1);


	double new_targetAngle;
	double theta = atan(Gradient);


	if (direction.sign == -1)
	{
		new_targetAngle = 0.8 * (90 / PI * (PI / 2 - atan(Gradient)));
	}
	else
	{
		new_targetAngle = 0.8 * (0 - 90 / PI * (PI / 2 - atan(Gradient)));
	}





	/*
	if (direction.sign == -1)
	{
		for (int i = 2; i < 5; i++)
		{
			_midline[i][0] = _midline[i][0] + 0.1 * cos(theta) * _width;
			_midline[i][1] = _midline[i][1] + 0.1 * sin(theta) * _width;
		}

	}
	else
	{
		for (int i = 2; i < 5; i++)
		{
			_midline[i][0] = _midline[i][0] - 0.1 * cos(theta) * _width;
			_midline[i][1] = _midline[i][1] - 0.1 * sin(theta) * _width;
		}
	}

	*/





	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█	舵角控制																█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	double targetAngleError = new_targetAngle; //目标误差
	int sight1 = 20;
	if (!isCementRoad) {
		sight1 = 24;
	}
	else {
		sight1 = 18;
	}
	double currentAngleError = atan2(_midline[sight1][0], _midline[sight1][1]); //当前误差
	

															
	//第一帧初始化舵角控制参数，清空积分器和微分器，因为控制目标为恒零，所以只需要初始化一次
	if (isFirstFrame)
	{
		isFirstFrame = false;
		angleController.initial(6.0, 0, 12.0, targetAngleError);
	}
	if (timecounter == 69 && !isCementRoad)       //dirt路面舵角的PID参数
	{
		angleController.initial(9.0, 0, 1, targetAngleError);//0.3,5.0,,1.5  
	}
	else if (timecounter == 69 && isCementRoad) //城市路面舵角的PID参数
	{
		angleController.initial(8.0, 0, 12.0, targetAngleError);
	}


	//舵角PID控制
	*cmdSteer = constrain(-1.0, 1.0, 1.0 * angleController.calculate(currentAngleError));


	if (!isCementRoad)		//dirt沙地
	{
		*cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));
	}
	else             //城市路面
	{
		*cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));		
	}


	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█	速度控制																█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	double targetSpeed;  //目标车速
	double currentSpeed = _speed;	//当前误差


	//设定目标速度，如果前方弯道就设定低，直道就设定高

	if (!isCementRoad)       //dirt沙地路面
	{       
		if (minCruve > 300)
			targetSpeed = 375;
		else if (minCruve > 220)
			targetSpeed = constrain(310, 365, minCruve + 90);
		else if (minCruve > 160)
			targetSpeed = constrain(250, 310, minCruve + 90);
		else if (minCruve > 80)
			targetSpeed = constrain(190, 250, minCruve + 90);
		else if (minCruve > 60)
			targetSpeed = constrain(140, 190, 1.25 * minCruve + 65);
		else if (minCruve > 20)
			targetSpeed = constrain(95, 140, 1.125 * minCruve + 72.5);
		else targetSpeed = 65;
	}

	else if (isCementRoad)         //城市路面
	{
		if (minCruve > 300)
			targetSpeed = 385; 
		else if (minCruve > 240)
			targetSpeed = constrain(335, 380, minCruve + 95);
		else if (minCruve > 160)
			targetSpeed = constrain(305, 345, 0.75 * minCruve + 175);
		else if (minCruve > 100)
			targetSpeed = constrain(230, 305, 1.1667 * minCruve + 113.333);
		else if (minCruve > 60)
			targetSpeed = constrain(165, 230, 1.625 * minCruve + 67.5);
		else if (minCruve > 20)
			targetSpeed = constrain(85, 165, 2 * minCruve + 45);

		else targetSpeed = 70;
	}

	//每当目标速度变化时初始化PID控制器，重设参数，清空积分器和微分器
	if (targetSpeed != lastTargetSpeed)
	{
		if (isCementRoad)		//城市路面
		{
			speedController.initial(4.2, 0.01, 22.0, targetSpeed);
			lastTargetSpeed = targetSpeed;
		}
		else if (!isCementRoad)		//dirt沙地路面
		{
			speedController.initial(2, 0, 0.2, targetSpeed);
			lastTargetSpeed = targetSpeed;
		}
	}

	//控制油门刹车来调节速度
	if (!isCementRoad && flag)    //dirt沙地
	{
		//根据当前速度和目标速度关系，控制油门刹车以改变速度
		if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
		{

			if (_speed > 120 && abs(*cmdSteer) < 0.05)
			{
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = 0.7;
			}
			else if (_speed > 66 && abs(*cmdSteer) < 0.1)
			{
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = 0.40 + 0.04 * (minCruve / _speed);
			}


			else if (abs(*cmdSteer) > 0.5)
			{
				if (fError > 0.01)
					*cmdAcc = 0;
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = 0.05 + 0.01 * (minCruve / _speed);
			}

			else if (abs(*cmdSteer) > 0.3)
			{
				if (fError > 0.01)
					*cmdAcc = 0;
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = 0.2 + 0.04 * (minCruve / _speed);
			}
			else
			{
				if (fError > 0.01)
					*cmdAcc = 0;
				//除此之外的情况进一步限定油门
				*cmdAcc = 0.26 + 0.01 * (minCruve / _speed);
			}
			//加速情况下，刹车为0
			*cmdBrake = 0;
		}

		else
		{
			//减速情况下，刹车
			*cmdAcc = 0;
			*cmdBrake = 0.25 + 0.001 * (minCruve / _speed);
		}
	}


	else if (isCementRoad && flag)		//城市路面
	{
		if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
		{
			if (_speed > 250 && abs(*cmdSteer) < 0.1)
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.65, speedController.calculate(currentSpeed));
			}
			if (_speed > 200 && abs(*cmdSteer) < 0.1)
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.8, speedController.calculate(currentSpeed));
			}
			if (_speed > 130 && abs(*cmdSteer) < 0.1)
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.95, speedController.calculate(currentSpeed));
			}
			else if (_speed > 60 && abs(*cmdSteer) < 0.2)
			{
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = constrain(0.0, 0.65, speedController.calculate(currentSpeed));
			}
			else if (_speed > 20 && abs(*cmdSteer) < 0.2)
			{
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = constrain(0.0, 0.4, speedController.calculate(currentSpeed));
			}

			/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
			█	为了过急弯															█
			\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
			//根据方向盘和角速度综合考虑油门
			else if (minCruve < 18)
			{
				*cmdAcc = constrain(0.1, 0.2, speedController.calculate(currentSpeed));
			}

			else
			{
				//除此之外的情况进一步限定油门
				*cmdAcc = 0.2;
			}

			//加速情况下，刹车为0
			*cmdBrake = 0;
		}

		else
		{
			//减速情况下，刹车
			*cmdAcc = 0;
			if (currentSpeed - targetSpeed > 200 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.7;
			}
			else if (currentSpeed - targetSpeed > 140 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.5;
			}
			else if (currentSpeed - targetSpeed > 80 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.3;
			}
			else
			{
				*cmdBrake = 0.26;
			}
		}
	}

	//更新档位
	updateGear(cmdGear);

	//窗口可视化
	//cls_visual.Fig2Y(1, 0, 400, 0, 500, 10, "_yaw", 0, "sign", 100 * direction.sign, "targetAngelError", targetAngleError);
	//cls_visual.Fig2Y(2, -0.3, 0.3, -0.5, 0.5, 10, "yaw", _yaw, "yawrate", _yawrate);
	//cls_visual.Fig2Y(1, -1, 1, -1, 1, 10, "Acc", *cmdAcc, "Brake", *cmdBrake);
}
