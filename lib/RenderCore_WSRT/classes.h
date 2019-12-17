#pragma once
#include "rendersystem.h"

namespace lh2core {

inline float3 HSVtoRGB(int H, float S, float V) {
	float C = S * V;
	float X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
	float m = V - C;
	float Rs, Gs, Bs;

	if (H >= 0 && H < 60) {
		Rs = C;
		Gs = X;
		Bs = 0;
	}
	else if (H >= 60 && H < 120) {
		Rs = X;
		Gs = C;
		Bs = 0;
	}
	else if (H >= 120 && H < 180) {
		Rs = 0;
		Gs = C;
		Bs = X;
	}
	else if (H >= 180 && H < 240) {
		Rs = 0;
		Gs = X;
		Bs = C;
	}
	else if (H >= 240 && H < 300) {
		Rs = X;
		Gs = 0;
		Bs = C;
	}
	else {
		Rs = C;
		Gs = 0;
		Bs = X;
	}

	return make_float3(Rs + m, Gs + m, Bs + m);
}

//  +-----------------------------------------------------------------------------+
//  |  Ray	                                                                      |
//  |  Center piece of this Ray Tracer                                      LH2'19|
//  +-----------------------------------------------------------------------------+
class Ray {
public:
	float3 origin;		// origin point of the ray
	float3 direction;	// direction the ray is travelling in
	int bounces;		// maximum number of bounces of a ray before it is discarded.
};

//  +-----------------------------------------------------------------------------+
//  |  Texture                                                                    |
//  |  Encapsulates a palettized pixel surface with pre-scaled paletted for fast  |
//	|  shading. Partially copied from Software Rasterizer					LH2'19|
//  +-----------------------------------------------------------------------------+
class Texture {
public:
	int width = 0, height = 0;
	float3* pixels = 0;
};

//  +-----------------------------------------------------------------------------+
//  |  Material                                                                   |
//  |  Minimalistic mesh storage.                                           LH2'19|
//  +-----------------------------------------------------------------------------+
class Material {
public:
	float transmission;
	float specularity;
	float3 diffuse;
	float3 transmittance;
	Texture* texture = 0;
};

enum Side { Front, Back };

class Intersection {
public:
	Side side;
	float t;
	float u, v;
	CoreTri *tri;
	mat4 transform;

	float3 position;
	float3 normal;
};
}
