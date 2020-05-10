#pragma once
#include <Peripheral.h>
#include <GPIO/Pin.h>
#include <BiSS/BiSS.h>

#include <math.h>
#include <MathUtilities/AngleCalculator.h>
#include "MathUtilities/LowPassFilter.h"
#include "MathUtilities/PiController.h"
#include "MathUtilities/ClarkeTransformation.h"
#include "MathUtilities/SpaceVectorGenerator.h"
#include "MathUtilities/RampController.h"
#include "MathUtilities/VoltageCalculator.h"
#include <MathUtilities/ParkTransformation.h>
#include <MathUtilities/RampGenerator.h>
#include <MathUtilities/SpeedCalculator.h>




class BLDC {
public:
	Pin enable;

	Pin enableChannelA;
	Pin enableChannelB;
	Pin enableChannelC;

	TIM_HandleTypeDef *pwmTimer;
	uint32 pwmChannelA;
	uint32 pwmChannelB;
	uint32 pwmChannelC;


	enum class MotorStateType { Stop, Run };
	enum class MotorRunType { Type6 };


	struct Parameters {
		uint16 baseFrequency = 200; 		// Base electrical frequency (Hz)
		uint16 resolverStepsPerTurn = 4096;
		uint16 motorPoles = 8;
		uint16 speedLoopPrescaler = 10;
		uint16 halfPeriodMax = 100;
		uint32 alignCountStart = 20000;
		float sampingPeriod = 0.001 / 100;  // Samping period (sec) (0.001 / ISR_FREQUENCY (kHz))
		float positionLowKi = 0.0001;
		float positionMediumKi = 0.001;
		float positionhighKi  = 0.01;
		float offsetFilterK1 = 0.998; 		// Offset filter coefficient K1: 0.05 / (T + 0.05);
		float offsetFilterK2 = 0.001999;	// Offset filter coefficient K2: T / (T + 0.05);
		float dAxisReferenceStart = 0.1;
		float dAxisReferenceRun = 0.0;
		MotorRunType motorRunType = MotorRunType::Type6;
	};




private:
	Parameters settings;

	MotorStateType motorState = MotorStateType::Stop;

	BiSS encoder = Peripheral::hspi1;


	// Instance a few transform objects
	ClarkeTransformation clarkeTransform;
	ParkTransformation parkTransform;
	InverseParkTransformation inverseParkTransform;

	// Instance PI regulators to regulate the d and q  axis currents, and speed
	PiController speedController;
	PiController dAxisController;
	PiController qAxisController;
	PiController positionController;

	// Instance a Space Vector PWM modulator. This modulator generates a, b and c
	// phases based on the d and q stationery reference frame inputs
	SpaceVectorGenerator spaceVectorGenerator;

	// Instance a ramp controller to smoothly ramp the frequency
	RampController rampController;

	// Instance a speed calculator based on QEP
	SpeedCalculator speedCalculator;

	// Instance a AngleCalculator
	AngleCalculator angleCalculator;


	uint32 offsetA = 0;
	uint32 offsetB = 0;
	uint32 offsetC = 0;

	float dAxisReference = 0.0;
	bool lockRotor = true;
	uint16 speedLoopCount = 1;
	uint32 alignCount = 0;


public:
	BLDC();
	BLDC(const Parameters &init);
	void Init();
	void InitOffset(uint16 adcRawA, uint16 adcRawB, uint16 adcRawC);
	void SetState(MotorStateType newState);
	void Tick(uint16 adcRawA, uint16 adcRawB, uint16 adcRawC);
	void UpdateEncoderAngle();

private:
	void SetPwmCmpare(uint16 ChannelA, uint16 ChannelB, uint16 ChannelC);
	void MotorRunType6(uint16 adcRawA, uint16 adcRawB, uint16 adcRawC);
	int32 PositionReferenceGeneration(int32 out);
};



