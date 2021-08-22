#pragma once
using namespace std;
#define hardcodeMap(x,f1,f2,t1,t2) ((x-(f1))/((f2)-(f1))*((t2)-(t1))+(t1))

typedef double (*pidkFunction)(double, double);
class PID {
public:
	//uint8_t dForward : 1;
	double target, input;								//目标值,反馈值
	double Kp, Ki, Kd;									//比例系数,积分系数,微分系数
	pidkFunction KpFunction, KiFunction, KdFunction;	//
	double sT;											//离散化系统的采样周期
	double e0, e1, e2;									//e(k),(k-1),e(k-2)
	double o0, o1;										//u(k),u(k-1)
	double inputScalerMin, inputScalerMax, outputScalerMin, outputScalerMax;
	PID(double kp, double ki, double kd, uint32_t sampTime) {
		sT = sampTime;
		inputScalerMin = outputScalerMin = -1;
		inputScalerMax = outputScalerMax = 1;
		e0 = e1 = e2 = 0;
		o0 = o1 = 0;
		Kp = kp, Ki = ki, Kd = kd;
		target = input = 0;
		KpFunction = 0, KiFunction = 0, KdFunction = 0;
		//dForward = false;
	}
	void setPIDArguments(double kp, double ki, double kd) {
		Kp = kp, Ki = ki, Kd = kd;
	}
	void setInputScaler(double inputMin, double inputMax) {
		inputScalerMin = inputMin, inputScalerMax = inputMax;
	}
	void setOutputScaler(double outputMin, double outputMax) {
		outputScalerMin = outputMin, outputScalerMax = outputMax;
	}
	void update() {
		e2 = e1;
		e1 = e0;
		e0 = hardcodeMap((target - input), inputScalerMin, inputScalerMax, -1.0, 1.0);
		double kp = KpFunction == 0 ? Kp : KpFunction(e0, Kp), kd = KdFunction == 0 ? Kd : KdFunction(e0, Kd), ki = KiFunction == 0 ? Ki : KiFunction(e0, Ki);
		double a2 = kd / sT;
		double a1 = kp + 2 * a2;
		double a0 = kp + ki * sT + a2;
		o1 = o0;
		o0 = o1 + a0 * e0 + a1 * e1 + a2 * e2;
		if (o0 > 1) o0 = 1;
		if (o0 < -1) o0 = -1;
	}
};
