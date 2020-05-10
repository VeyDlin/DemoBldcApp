#pragma once
#include <algorithm>
#include <cmath>


class VoltageCalculator {
private:
	static constexpr float ONE_THIRD = 0.33333333333333;
	static constexpr float TWO_THIRD = 0.66666666666667;
	static constexpr float INV_SQRT3 = 0.57735026918963;

	struct {
		float dcBusVoltage = 0;			// DC-bus voltage (pu)
		float modulationVoltageA = 0;  	// Modulation voltage phase A (pu)
		float modulationVoltageB = 0;	// Modulation voltage phase B (pu)
		float modulationVoltageC = 0;	// Modulation voltage phase C (pu)
	} in;


public:
	bool outputOfPhase = true; 	// Out of Phase adjustment

	struct Out {
		float phaseA = 0;		// Phase voltage phase A (pu)
		float phaseB = 0;		// Phase voltage phase B (pu)
		float phaseC = 0;		// Phase voltage phase C (pu)
		float alpha = 0;		// Stationary d-axis phase voltage (pu)
		float beta = 0;  		// Stationary q-axis phase voltage (pu)
	} out;


	VoltageCalculator& Set(float dcBusVoltage, float modulationVoltageA, float modulationVoltageB, float modulationVoltageC) {
		in.dcBusVoltage = dcBusVoltage;
		in.modulationVoltageA = modulationVoltageA;
		in.modulationVoltageB = modulationVoltageB;
		in.modulationVoltageC = modulationVoltageC;
		return *this;
	}


	// outputOfPhase = true for the out of phase correction if
	//	* modulationVoltageA is out of phase with PWM1,
	//	* modulationVoltageB is out of phase with PWM3,
	//	* modulationVoltageC is out of phase with PWM5,
	// otherwise, set 0 if their phases are correct.
	VoltageCalculator& Resolve() {

		// Scale the incomming Modulation functions with the DC bus voltage value and calculate the 3 Phase voltages
		float dcBusVoltage = in.dcBusVoltage * ONE_THIRD;
		out.phaseA = dcBusVoltage * ((in.modulationVoltageA * 2) - in.modulationVoltageB - in.modulationVoltageC);
		out.phaseB = dcBusVoltage * ((in.modulationVoltageB * 2) - in.modulationVoltageA - in.modulationVoltageC);

		if(outputOfPhase == false) {
			out.phaseA = -out.phaseA;
			out.phaseB = -out.phaseB;
		}

		// Voltage transformation (a, b, c)  ->  (Alpha, Beta)
		out.alpha = out.phaseA;
		out.beta = (out.phaseA + (out.phaseB * 2)) * INV_SQRT3;

		return *this;
	}


	Out Get() {
		return out;
	}
};





























