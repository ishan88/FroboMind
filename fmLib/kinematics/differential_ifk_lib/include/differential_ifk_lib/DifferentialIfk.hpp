/****************************************************************************
 # FroboMind
 # Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #	* Redistributions of source code must retain the above copyright
 #  	notice, this list of conditions and the following disclaimer.
 #	* Redistributions in binary form must reproduce the above copyright
 #  	notice, this list of conditions and the following disclaimer in the
 #  	documentation and/or other materials provided with the distribution.
 #	* Neither the name FroboMind nor the
 #  	names of its contributors may be used to endorse or promote products
 #  	derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************
 # 2013-10-05 Kjeld Jensen: First version
 # 2014-04-06 Leon Bonde Larsen: Translated library to C++
 #
 ****************************************************************************
 # This library contains functions for forward and inverse kinematics of a
 # differentially steered vehicle.
 #
 # The forward kinematic model describes how the speed of the two wheels
 # (vel_right, vel_left) translate into a forward speed (vel_lin) and an angular
 # speed (vel_ang).
 #
 # The inverse kinematic model describes how the speed of the wheels
 # (vel_left, vel_right) must be set for the robot to obtain a forward speed
 # (vel_lin) and an angular speed (vel_ang).
 ****************************************************************************/
#ifndef DIFFERENTIALIFK_HPP_
#define DIFFERENTIALIFK_HPP_

class DifferentialIfk
{
public:
	double _wheel_dist;

	DifferentialIfk();
	DifferentialIfk(double);
	virtual ~DifferentialIfk();

	struct twist_t
	{
		double linear, angular;
	};

	struct wheel_t
	{
		double left, right;
	};

	twist_t forward(double, double);
	wheel_t inverse(double, double);
};

#endif /* DIFFERENTIALIFK_HPP_ */
