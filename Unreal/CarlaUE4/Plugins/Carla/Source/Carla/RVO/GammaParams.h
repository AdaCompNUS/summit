#ifndef GAMMA_PARAMS_H_
#define GAMMA_PARAMS_H_

#include <compiler/disable-ue4-macros.h>

namespace GammaParams{
	extern bool use_polygon;
	extern bool consider_kinematics;
	extern bool use_dynamic_resp;
	extern bool use_dynamic_att;

	const float GAMMA_PI = 3.14159;
};

#include <compiler/enable-ue4-macros.h>

#endif
