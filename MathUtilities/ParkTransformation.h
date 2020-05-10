#pragma once
#include <cmath>


class ParkTransformation {
private:
	struct {
		float alpha; // stationary d-axis stator variable
		float beta;  // stationary q-axis stator variable
		float angle; // rotating angle (pu)
	} in;

public:
	struct Out {
		float dAxis = 0; // rotating d-axis stator variable
		float qAxis = 0; // rotating q-axis stator variable
	} out;


	ParkTransformation& Set(float alpha, float beta, float angle) {
		in.alpha = alpha;
		in.beta = beta;
		in.angle = angle;

		return *this;
	}

	ParkTransformation& Resolve() {
		out.dAxis = in.alpha * cos(in.angle) + in.beta  * sin(in.angle);
		out.qAxis = in.beta * cos(in.angle) - in.alpha * sin(in.angle);

		return *this;
	}

	Out Get() const {
		return out;
	}
};


class InverseParkTransformation {
private:
	struct {
		float dAxis;
		float qAxis;
		float angle;
	} in;

public:
	struct Out {
		float alpha = 0;
		float beta = 0;
	} out;

	InverseParkTransformation& Set(float dAxis, float qAxis, float angle) {
		in.dAxis = dAxis;
		in.qAxis = qAxis;
		in.angle = angle;

		return *this;
	}

	InverseParkTransformation& Resolve() {
		out.alpha = in.dAxis * cos(in.angle) - in.qAxis * sin(in.angle);
		out.beta = in.dAxis * sin(in.angle) + in.qAxis * cos(in.angle);

		return *this;
	}

	Out Get() {
		return out;
	}
};


