#pragma once
#include <algorithm>
#include <cmath>


class RampController {
private:
	float targetPosition = 0;
	uint32 delayCount = 0;

public:
	uint32 delayMax = 5;
	float amplitudeMin = -1;
	float amplitudeMax = 1;
	float resolution = 0.0000305;

	struct Out {
		float position = 0;
		bool equalFlag = false;
	} out;


	RampController& Set(float target) {
		targetPosition = target;
		return *this;
	}

	RampController& SetReferencePosition(float position) {
		out.position = position;
		return *this;
	}

	RampController& Reset() {
		targetPosition = 0;
		delayCount = 0;
		out.position = 0;
		out.equalFlag = 0;
		return *this;
	}


	RampController& Resolve() {
		float dlt = targetPosition - out.position;

		if (abs(dlt) >= resolution) {
			delayCount++;

			if (delayCount >= delayMax) {
				if (targetPosition >= out.position) {
					out.position += resolution;
				} else {
					out.position -= resolution;
				}

				out.position = std::min(std::max(out.position , amplitudeMin),  amplitudeMax);

				delayCount = 0;
			}
		} else {
			out.equalFlag = true;
			out.position = targetPosition;
		}

		return *this;
	}


	Out Get() {
		return out;
	}

	float GetTargetPosition() {
		return targetPosition;
	}



	// Slew programmable ramper
	static inline float Ramper(float in, float out, float rampDelta) {
		float err = in - out;

		if (err > rampDelta) {
			return out + rampDelta;
		} else if (err < -rampDelta) {
	  		return out - rampDelta;
		}

	    return in;
	}
};








