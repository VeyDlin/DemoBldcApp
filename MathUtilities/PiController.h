#pragma once
#include <cmath>
#include <algorithm>


class PiController {
private:
	float i1 = 0; // integrator storage: ui(k-1)
	float v1 = 0; // pre-saturated controller output

    float piOutput = 0;

	struct {
		float reference = 0;
		float feedback = 0;
	} in;

public:
	float Kp = 1.0; 	// proportional loop gain
	float Ki = 0.0; 	// integral gain
	float Umax = 1.0; 	// upper saturation limit
	float Umin = -1.0; 	// lower saturation limit



	PiController& SetReference(float reference) {
        in.reference = reference;
        return *this;
    }

	PiController& SetFeedback(float feedback) {
        in.feedback = feedback;
        return *this;
    }

	PiController& Set(float reference, float feedback) {
        in.reference = reference;
        in.feedback = feedback;
        return *this;
    }


	PiController& Clear() {
		i1 = 0;
		v1 = 0;
		return *this;
	}


	PiController& Resolve() {
		// proportional term
		float up = Kp *  (in.reference - in.feedback);

		// integral term
		float ui = piOutput == v1 ? ((Ki * up) + i1) : i1;
		ui = std::min(std::max(ui, Umin), Umax);
		i1 = ui;

		// control output
		v1 = up + ui;
		piOutput = std::min(std::max(v1, Umin), Umax);

		return *this;
	}


	// This macro works with angles as inputs, hence error is rolled within -pi to +pi
	PiController& ResolvePosition() {
		// proportional term
		float up = in.reference - in.feedback;

		if(up >= 0.5){
			up -= 1.0; // roll in the error
		} else if (up <= -0.5) {
			up += 1.0; // roll in the error
		}

		// integral term
		up = Kp * up;
		float ui = piOutput == v1 ? ((Ki * up) + i1) : i1;
		i1 = ui;

		// control output
		v1 = up + ui;
		piOutput = std::min(std::max(v1, Umin), Umax);

		return *this;
	}

	float GetOutput() {
		return piOutput;
	}

	float GetFeedback() {
		return in.feedback;
	}

	float GetReference() {
		return in.reference;
	}
};




