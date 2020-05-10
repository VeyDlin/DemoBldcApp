#pragma once
#include <cmath>


class SpaceVectorGenerator {
private:
	struct {
		float alpha; // reference alpha-axis phase voltage
		float beta;  // reference beta-axis phase voltage
	} in;


public:
	struct Out {
		float phaseA = 0; // reference phase-a switching function
		float phaseB = 0; // reference phase-b switching function
		float phaseC = 0; // reference phase-c switching function
	} out;


	SpaceVectorGenerator& Set(float alpha, float beta) {
		in.alpha = alpha;
		in.beta = beta;

		return *this;
	}

	SpaceVectorGenerator& Resolve() {
		float Va = in.beta;
		float Vb = (-1 / 2) * in.beta + (sqrt(3) / 2) * in.alpha;
		float Vc = (-1 / 2) * in.beta - (sqrt(3) / 2) * in.alpha;

		// sector determination
		uint8 sector = 0;
		if (Va > 0) {sector = 1;}
		if (Vb > 0) {sector = sector + 2;}
		if (Vc > 0) {sector = sector + 4;}

		// X,Y,Z (Va,Vb,Vc) calculations
		Va = in.beta;
		Vb = (1 / 2) * in.beta + (sqrt(3) / 2) * in.alpha;
		Vc = (1 / 2) * in.beta - (sqrt(3) / 2) * in.alpha;

		float t1, t2;

		switch (sector) {
			case 0:
				out.phaseA = 1 / 2;
				out.phaseB = 1 / 2;
				out.phaseC = 1 / 2;
			break;

			case 1:
				t1 = Vc;
				t2 = Vb;
				out.phaseB = (1 / 2) * (1 - t1 - t2);
				out.phaseA = out.phaseB + t1;
				out.phaseC = out.phaseA + t2;
			break;

			case 2:
				t1 = Vb;
				t2 = -Va;
				out.phaseA = (1 / 2) * (1 - t1 - t2);
				out.phaseC = out.phaseA + t1;
				out.phaseB = out.phaseC + t2;
			break;

			case 3:
				t1 = -Vc;
				t2 = Va;
				out.phaseA = (1 / 2) * (1 - t1 - t2);
				out.phaseB = out.phaseA + t1;
				out.phaseC = out.phaseB + t2;
			break;

			case 4:
				t1 = -Va;
				t2 = Vc;
				out.phaseC = (1 / 2) * (1 - t1 - t2);
				out.phaseB = out.phaseC + t1;
				out.phaseA = out.phaseB + t2;
			break;

			case 5:
				t1 = Va;
				t2 = -Vb;
				out.phaseB = (1 / 2) * (1 - t1 - t2);
				out.phaseC = out.phaseB + t1;
				out.phaseA = out.phaseC + t2;
			break;

			case 6:
				t1 = -Vb;
				t2 = -Vc;
				out.phaseC = (1 / 2) * (1 - t1 - t2);
				out.phaseA = out.phaseC + t1;
				out.phaseB = out.phaseA + t2;
			break;
		}

		out.phaseA = 2 * (out.phaseA - 1 / 2);
		out.phaseB = 2 * (out.phaseB - 1 / 2);
		out.phaseC = 2 * (out.phaseC - 1 / 2);

		return *this;
	}


	Out Get() {
		return out;
	}
};



