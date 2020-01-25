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
	float3 origin;		// 12 - origin point of the ray
	float3 direction;	// 12 - direction the ray is travelling in
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

// make power of 2 so we can use cach line opt
class IntersectionTraverse {
public:
	float t;           // 4
	float u, v;        // 8
	int tri;           // 4 - first byte for bvhId last 3 bytes for triId 
					   // total: 16

	__inline void Reset() { t = 1e34f; }
};

class IntersectionShading {
public:
	bool hasIntersection; // 4
	float3 position;      // 12
	float3 normal;        // 12
	float3 diffuse;       // 12
	Side side;            // 4?
	float t;              // 4
						  // total = 48
};

struct Photon {
	float3 position; // 12 - world space position of the photon hit
	float energy;    // 4 - current power level for the photon
	uint lightIndex; // 4 20 total
};

}
