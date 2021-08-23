#pragma once
#ifndef PID_H
#define PID_H
#define hardcodeMap(x,f1,f2,t1,t2) ((x-(f1))/((f2)-(f1))*((t2)-(t1))+(t1))
#include <Arduino.h>
typedef double (*pidArgFunction)(double, double);
class PID {
public:
	double target, input, output;						//目标值,反馈值，输出
	double Kp, Ki, Kd;									//比例系数,积分系数,微分系数
	pidArgFunction KpFunction, KiFunction, KdFunction;	//变 比例/积分/微分 参数 函数
	double sT;											//离散化系统的采样周期
	double e0, e1, e2;									//e(k),(k-1),e(k-2)
	double o0, o1;										//u(k),u(k-1)
	double inputScalerMin, inputScalerMax, outputScalerMin, outputScalerMax;
	PID(double kp, double ki, double kd, uint32_t sampTime);
	void setPIDArguments(double kp, double ki, double kd);
	void setInputScaler(double inputMin, double inputMax);
	void setOutputScaler(double outputMin, double outputMax);
	void update();
};
#endif
