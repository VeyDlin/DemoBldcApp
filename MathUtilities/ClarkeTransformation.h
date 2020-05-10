#pragma once
#include <cmath>


class ClarkeTransformation {
private:
	struct {
		float phaseA; // phase-a stator variable
		float phaseB; // phase-b stator variable
	} in;

public:
	struct Out {
		float alpha = 0; // stationary d-axis stator variable
		float beta = 0;  // stationary q-axis stator variable
	} out;

	ClarkeTransformation& Set(float phaseA, float phaseB) {
		in.phaseA = phaseA;
		in.phaseB = phaseB;

		return *this;
	}

	ClarkeTransformation& Resolve() {
		out.alpha = in.phaseA;
		out.beta = 1 / sqrt(3) * (in.phaseA + 2 * in.phaseB);

		return *this;
	}

	Out Get() {
		return out;
	}
};


class InverseClarkeTransformation {
private:
	struct {
		float alpha; // stationary d-axis stator variable
		float beta;  // stationary q-axis stator variable
	} in;

public:
	struct Out {
		float phaseA = 0; // phase-a stator variable
		float phaseB = 0; // phase-b stator variable
		float phaseC = 0; // phase-c stator variable
	} out;


	InverseClarkeTransformation& Set(float alpha, float beta) {
		in.alpha = alpha;
		in.beta = beta;

		return *this;
	}

	InverseClarkeTransformation& Resolve() {
		out.phaseA = in.alpha;
		out.phaseB = (sqrt(3) / 2) * in.beta - (1 / 2) * in.alpha;
		out.phaseC = (-1 / 2) * in.alpha - (sqrt(3) / 2) * in.beta;

		return *this;
	}

	Out Get() {
		return out;
	}
};



