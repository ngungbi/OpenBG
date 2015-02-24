
#define PI 3.14159265359
//***************************************************************
// contrain function for 32-bit values
//***************************************************************
inline int32_t constrain_int32(int32_t x , int32_t l, int32_t h) {
	if (x <= l) {
		return l;
	} else if (x >= h) {
		return h;
	} else {
		return x;
	}
}

//***************************************************************
// “Efficient approximations for the arctangent function”,
// Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
//***************************************************************
inline float Rajan_FastArcTan(float x) {
	return PI/4.0*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x));
}

// atan2 for all quadrants by A. Hahn
inline float Rajan_FastArcTan2(float y, float x) {

	uint8_t qCode;
	const float pi_2 = PI/2.0;
	float q;
	float z;

	// 6 us
	uint8_t swap45 = (fabs(y) > fabs(x));
	 
	// 22us
	if ((y >= 0) && (x >= 0)) { qCode = 0; }
	if ((y >= 0) && (x <= 0)) { qCode = 1; }
	if ((y <= 0) && (x <= 0)) { qCode = 2; }
	if ((y <= 0) && (x >= 0)) { qCode = 3; }

	// 54 us
	if (swap45) {
	 q = x / y;
	} else {
	 q = y / x;
	}

	// 92 us
	z = Rajan_FastArcTan(q);

	if (swap45) {
		switch (qCode) {
			case 0: z = pi_2 - z;  break;
			case 1: z = pi_2 - z;  break;
			case 2: z = -pi_2 - z; break;
			case 3: z = -pi_2 - z; break;
		}
	} else {
		switch (qCode) {    
			case 0: z = z;         break;
			case 1: z = PI + z;    break;
			case 2: z = -PI + z;   break;
			case 3: z = z;         break;
		}
	}

	return z;
}

// atan2 returnig degrees * 1000
int32_t Rajan_FastArcTan2_deg1000(float y, float x) {
	return 180/PI * 1000 * Rajan_FastArcTan2(y, x);
}
float InvSqrt (float x){ 
	union{  
		int32_t i;  
		float   f; 
	} conv; 
	conv.f = x; 
	conv.i = 0x5f3759df - (conv.i >> 1); 
	return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}