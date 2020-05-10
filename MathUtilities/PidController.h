#pragma once
#include <cmath>


class PidController {
private:
    float error = 0;     // error between ref and fdb

    float Ui = 0;      // integral output
    float Ud = 0;      // derivative output
    float Up = 0;      // proportional output
    float previousUp = 0;  // previous proportional output

    float saturatedDifference = 0;

    float pidOutput = 0;

	struct {
		float reference;
		float feedback;
	} in;

public:
    float Kp = 1.3;    // proportional gain
    float Ki = 0.02;   // integral gain
    float Kc = 0.5;    // integral correction gain
    float Kd = 1.05;   // derivative gain

    float maximumOutput = 1;
    float minimumOutput = -1;


	PidController& SetReference(float reference) {
        in.reference = reference;
        return *this;
    }

	PidController& SetFeedback(float feedback) {
        in.feedback = feedback;
        return *this;
    }

	PidController& Set(float reference, float feedback) {
        in.reference = reference;
        in.feedback = feedback;
        return *this;
    }


	PidController& Resolve() {
		error = in.reference - in.feedback;

        Up = Kp * error;
        Ui = Ui + Ki * Up + Kc * saturatedDifference;

        float preSaturated = Up + Ui;

        pidOutput = std::min(std::max(preSaturated, minimumOutput), maximumOutput);

        saturatedDifference = pidOutput - preSaturated;
        previousUp = Up;

        return *this;
    }


	float GetOutput() {
        return pidOutput;
    }
};


