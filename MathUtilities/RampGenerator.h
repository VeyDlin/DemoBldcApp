#pragma once
#include <algorithm>
#include <cmath>


class RampGenerator {
private:
	struct {
		float rampFrequency = 0;
		float gain = 1;
		float offset = 1;
	} in;

	float outputRampSignal = 0;
	float angle = 0;
public:
	float stepAngleMax = 0;


	RampGenerator& Set(float rampFrequency, float gain, float offset) {
		in.rampFrequency = rampFrequency;
		in.gain = gain;
		in.offset = offset;
		return *this;
	}


	RampGenerator& Resolve() {
		// Compute the angle rate
		angle += stepAngleMax * in.rampFrequency;

		// Saturate the angle rate within (-1, 1)
		if (angle > 1.0) {
			angle -= 1.0;
		} else if (angle < -1.0) {
			angle += 1.0;
		}

		// Compute the ramp output
		outputRampSignal = (angle * in.gain) + in.offset;

		// Saturate the ramp output within (-1, 1)
		if (outputRampSignal > 1.0) {
			outputRampSignal -= 1.0;
		} else if (outputRampSignal < -1.0) {
			outputRampSignal += 1.0;
		}

		return *this;
	}


	float Get() {
		return outputRampSignal;
	}
};




