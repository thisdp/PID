#include "PID.h"
PID::PID(double kp, double ki, double kd, uint32_t sampTime) {
	sT = sampTime;
	inputScalerMin = outputScalerMin = -1;
	inputScalerMax = outputScalerMax = 1;
	e0 = e1 = e2 = 0;
	o0 = o1 = 0;
	output = 0;
	Kp = kp, Ki = ki, Kd = kd;
	target = input = 0;
	KpFunction = 0, KiFunction = 0, KdFunction = 0;
}

void PID::setPIDArguments(double kp, double ki, double kd) {
	Kp = kp, Ki = ki, Kd = kd;
}

void PID::setInputScaler(double inputMin, double inputMax) {
	inputScalerMin = inputMin, inputScalerMax = inputMax;
}

void PID::setOutputScaler(double outputMin, double outputMax) {
	outputScalerMin = outputMin, outputScalerMax = outputMax;
}

void PID::update() {
	e2 = e1;
	e1 = e0;
  double targetTransformed = hardcodeMap(target, inputScalerMin, inputScalerMax, -1.0, 1.0);
  double inputTransformed = hardcodeMap(input, inputScalerMin, inputScalerMax, -1.0, 1.0);
  e0 = targetTransformed-inputTransformed;
	double kp = KpFunction == 0 ? Kp : KpFunction(e0, Kp), kd = KdFunction == 0 ? Kd : KdFunction(e0, Kd), ki = KiFunction == 0 ? Ki : KiFunction(e0, Ki);
	double a2 = kd / sT;
	double a1 = kp + 2.0 * a2;
	double a0 = kp + ki * sT + a2;
	o1 = o0;
	o0 = o1 + a0 * e0 + a1 * e1 + a2 * e2;
	if (o0 > 1) o0 = 1;
	if (o0 < -1) o0 = -1;
	output = hardcodeMap(o0, -1.0, 1.0, outputScalerMin, outputScalerMax);
}
