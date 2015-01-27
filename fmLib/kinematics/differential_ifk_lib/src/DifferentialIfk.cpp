/****************************************************************************
 # FroboMind
 # Licence and description in .hpp file
 ****************************************************************************/
#include "differential_ifk_lib/DifferentialIfk.hpp"

DifferentialIfk::DifferentialIfk()
{
	_wheel_dist = 0.0;
}

DifferentialIfk::DifferentialIfk(double wheel_distance)
{
	_wheel_dist = wheel_distance;
}

DifferentialIfk::~DifferentialIfk()
{

}

DifferentialIfk::twist_t DifferentialIfk::forward(double vel_left, double vel_right)
{
	DifferentialIfk::twist_t out;
	out.linear = (vel_right + vel_left)/2.0;
	out.angular = (vel_right - vel_left)/_wheel_dist;
	return out;
}

DifferentialIfk::wheel_t DifferentialIfk::inverse(double vel_lin, double vel_ang)
{
	DifferentialIfk::wheel_t out;
	out.left  = vel_lin - _wheel_dist*vel_ang;
	out.right = vel_lin + _wheel_dist*vel_ang;
	return out;
}
