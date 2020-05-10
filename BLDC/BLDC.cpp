#include <BLDC/BLDC.h>



BLDC::BLDC() {

}

BLDC::BLDC(const Parameters &init) {
	settings = init;
}





void BLDC::Init() {
	// Initialize QEP module
	angleCalculator.mechanicalScaler = 1.0 / settings.resolverStepsPerTurn;
	angleCalculator.polePairs = settings.motorPoles / 2;


	// Initialize the Speed module for QEP based speed calculation
	speedCalculator.K1 = 1 / (settings.baseFrequency * settings.sampingPeriod);
	speedCalculator.K2 = 1 / (1 + settings.sampingPeriod * 2 * M_PI * 5); // Low-pass cut-off frequency
	speedCalculator.K3 = 1 - speedCalculator.K2;
	speedCalculator.baseSpeedRpm = 120 * (settings.baseFrequency / settings.motorPoles);


	// Initialize the PI module for position
	positionController.Kp = 10.0;
	positionController.Ki = settings.positionLowKi;
	positionController.Umax = 1.0;
	positionController.Umin = -1.0;


	// Initialize the PI module for Id
	qAxisController.Kp = 0.3;
	qAxisController.Ki = 0.0003;
	qAxisController.Umax = 0.95;
	qAxisController.Umin = -0.95;


	// Initialize the PI module for speed
	speedController.Kp = 1.0;
	speedController.Ki = settings.sampingPeriod * settings.speedLoopPrescaler / 0.2;
	speedController.Umax = 0.95;
	speedController.Umin = -0.95;


	// Initialize the PI module for Iq
	dAxisController.Kp = 0.3;
	dAxisController.Ki = 0.0003;
	dAxisController.Umax = 0.6;
	dAxisController.Umin = -0.6;
}





void BLDC::InitOffset(uint16 adcRawA, uint16 adcRawB, uint16 adcRawC) {
    offsetA = (settings.offsetFilterK1 * offsetA) + (settings.offsetFilterK2 * adcRawA);
    offsetB = (settings.offsetFilterK1 * offsetB) + (settings.offsetFilterK2 * adcRawB);
    offsetC = (settings.offsetFilterK1 * offsetC) + (settings.offsetFilterK2 * adcRawC);
}





void BLDC::SetState(MotorStateType newState) {
	bool enablePwm = newState != MotorStateType::Stop;

	enable = enablePwm;

	enableChannelA = enablePwm;
	enableChannelB = enablePwm;
	enableChannelC = enablePwm;

    HAL_TIM_PWM_Start(pwmTimer, pwmChannelA);
    HAL_TIM_PWM_Start(pwmTimer, pwmChannelB);
    HAL_TIM_PWM_Start(pwmTimer, pwmChannelC);

	motorState = newState;
}





void BLDC::SetPwmCmpare(uint16 ChannelA, uint16 ChannelB, uint16 ChannelC) {
	__HAL_TIM_SET_COMPARE(pwmTimer, pwmChannelA, ChannelA);
	__HAL_TIM_SET_COMPARE(pwmTimer, pwmChannelB, ChannelB);
	__HAL_TIM_SET_COMPARE(pwmTimer, pwmChannelC, ChannelC);
}





void BLDC::UpdateEncoderAngle() {
	auto data = encoder.ReceiveData();
	angleCalculator.rawAngle = data.position;
}





void BLDC::Tick(uint16 adcRawA, uint16 adcRawB, uint16 adcRawC) {
	switch (settings.motorRunType) {
		case MotorRunType::Type6:
			MotorRunType6(adcRawA, adcRawB, adcRawC);
		break;
	}
}




void BLDC::MotorRunType6(uint16 adcRawA, uint16 adcRawB, uint16 adcRawC) {
	if (lockRotor) {
		// alignment curretnt
		dAxisReference = settings.dAxisReferenceStart;

		// for restarting from (RunMotor = false)
		rampController.Reset();

		// during alignment, assign the current shaft position as initial position
		angleCalculator.rawAngleOffset = angleCalculator.rawAngle;

		// set up an alignment and hold time for shaft to settle down
		if (dAxisController.GetReference() >= dAxisReference) {
			if (alignCount < settings.alignCountStart) {
				alignCount++;
			} else {
				lockRotor = false;
				alignCount = 0;
				dAxisReference = settings.dAxisReferenceRun;
			}
		}
	}


	// ------------------------------------------------------------------------------
	//  Read resolver data, get position and speed feedback
	// ------------------------------------------------------------------------------
	angleCalculator.Resolve();


	// ------------------------------------------------------------------------------
	//    Connect inputs of the SPEED_FR module and call the speed calculation macro
	// ------------------------------------------------------------------------------
	speedCalculator.Set(angleCalculator.Get().electricalAngle).Resolve();


	// ------------------------------------------------------------------------------
	//  Measure phase currents, subtract the offset and normalize from (-0.5, +0.5) to (-1, +1).
	//	Connect inputs of the CLARKE module and call the clarke transformation macro
	// ------------------------------------------------------------------------------
	clarkeTransform.Set(
		(adcRawA - offsetA) * 2,
		(adcRawB - offsetB) * 2
	).Resolve();


	// ------------------------------------------------------------------------------
	//  Connect inputs of the PARK module and call the park trans. macro
	// ------------------------------------------------------------------------------
	parkTransform.Set(
		angleCalculator.Get().electricalAngle,
		clarkeTransform.Get().alpha,
		clarkeTransform.Get().beta
	).Resolve();


	// ------------------------------------------------------------------------------
	//    Connect inputs of the PI module and call the PI speed controller macro
	// ------------------------------------------------------------------------------
	if (++speedLoopCount >= settings.speedLoopPrescaler) {
		speedLoopCount = 0;

		if (lockRotor) {
			rampController.Reset();
			positionController.Clear();
			speedController.Clear();
		} else {

			// ========== reference position setting =========
			rampController.Set(PositionReferenceGeneration(rampController.GetTargetPosition()));

			float empty;
			rampController.SetReferencePosition(modf(rampController.GetTargetPosition(), &empty));

			// Rolling in angle within 0 to 1pu
			if (rampController.Get().position < 0) {
				rampController.SetReferencePosition(rampController.Get().position + 1);
			}
			// ===============================================

			// PI coefficient adjustment for position loop
			auto speed = speedCalculator.Get().speed;
			if (speed < 0) {
				speed = -speed;
			}

			if (speed < 0.03) {
				positionController.Ki = settings.positionLowKi;
			} else if ((speed > 0.06) && (speed < 0.5)) {
				positionController.Ki = settings.positionMediumKi;
			} else if (speed > 0.7) {
				positionController.Ki = settings.positionhighKi;
			}
			// ===============================================

			// position PI regulator
			positionController.Set(
				rampController.Get().position,
				angleCalculator.Get().mechanicalAngle
			).ResolvePosition();

			// speed PI regulator
			speedController.Set(
				positionController.GetOutput(),
				speedCalculator.Get().speed
			).Resolve();
		}
	}


	// ------------------------------------------------------------------------------
	//    Connect inputs of the PID module and call the PID IQ controller macro
	// ------------------------------------------------------------------------------
	if(lockRotor) {
		qAxisController.SetReference(0);
	} else {
		qAxisController.SetReference(speedController.GetOutput());
	}
	qAxisController.SetFeedback(parkTransform.Get().qAxis).Resolve();


	// ------------------------------------------------------------------------------
	//    Connect inputs of the PID module and call the PID ID controller macro
	// ------------------------------------------------------------------------------
	dAxisController.Set(
		RampController::Ramper(dAxisReference, dAxisController.GetReference(), 0.0001), // ramprate = 1pu/s
		parkTransform.Get().dAxis
	).Resolve();


	// ------------------------------------------------------------------------------
	//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
	// ------------------------------------------------------------------------------
	inverseParkTransform.Set(
		dAxisController.GetOutput(),
		qAxisController.GetOutput(),
		angleCalculator.Get().electricalAngle
	).Resolve();


	// ------------------------------------------------------------------------------
	//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
	// ------------------------------------------------------------------------------
	spaceVectorGenerator.Set(
		inverseParkTransform.Get().alpha,
		inverseParkTransform.Get().beta
	).Resolve();


	// ------------------------------------------------------------------------------
	//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
	// ------------------------------------------------------------------------------

	SetPwmCmpare(
		(settings.halfPeriodMax * spaceVectorGenerator.Get().phaseA) + settings.halfPeriodMax,
		(settings.halfPeriodMax * spaceVectorGenerator.Get().phaseB) + settings.halfPeriodMax,
		(settings.halfPeriodMax * spaceVectorGenerator.Get().phaseC) + settings.halfPeriodMax
	);
}






// position reference generation test module
int32 BLDC::PositionReferenceGeneration(int32 out) {
	static const float positionArray[8] = { 1.5, -1.5, 2.5, -2.5 };
	int32 ptrMax = 2;
	int32 cntr1 = 0;
	int32 ptr1 = 0;
	float positionSlewRate = 0.001;
	int32 in = positionArray[ptr1];

	out = RampController::Ramper(in, out, positionSlewRate);

	if (in == out) {
		if (++cntr1 > 1000) {
			cntr1 = 0;
			if (++ptr1 >= ptrMax) {
				ptr1 = 0;
			}
		}
	}

	return out;
}
