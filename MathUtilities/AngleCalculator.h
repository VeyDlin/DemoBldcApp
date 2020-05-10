#pragma once
#include <algorithm>
#include <cmath>



class AngleCalculator {
public:
	int32 rawAngle = 0;  // Raw position data from resolver
	uint16 mechanicalScaler = 0;
	uint16 polePairs = 2;
	int32 rawAngleOffset = 0; // Raw angular offset between encoder index and phase a


    struct Out {
		int32 electricalAngle = 0; // Motor Electrical angle
		int32 mechanicalAngle = 0; // Motor Mechanical Angle
    } out;



    AngleCalculator& Resolve() {
    	out.mechanicalAngle = rawAngle - rawAngleOffset;
    	out.mechanicalAngle *= mechanicalScaler;

    	out.electricalAngle = polePairs * out.mechanicalAngle;

		return *this;
	}


    Out Get() {
    	return out;
    }
};


