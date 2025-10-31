/*
 * force_reactive_robot_companion_learning.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: gferrer
 */
/*
 *
 *  Created on: Dec 22, 2013 by Gonzalo Ferrer. 
 *  Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration)
 *      Author: Initial code of the Robot alone AKP navigation is from Gonzalo Ferrer. 
 *      Author: The Addition of ASPSI and other approaching and accompaniment modifications and codes are of Ely Repiso.
 *
 *      License (for other authors that will not be the original ones): CC BY-NC-ND 4.0 
 *              (Attribution-NonCommercial-NoDerivatives 4.0 International)
 *              https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
 *
 *      This license does not allow other authors to modify or to take profit from these works. 
 *      Then, for modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
 *   collaborations (for journals with other researchers, formal collaborations between UPC and companies, and so on).
 *
 *  Please to only use it cite: Repiso, Ely, Anaís Garrell, and Alberto Sanfeliu. "Adaptive social planner to accompany people in real-life dynamic environments." International Journal of Social Robotics 16.6 (2024): 1189-1221.
 *
 *  Redistribution and use in source and binary forms, without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission. 
 *   * Additionally, remember that the derivatives without collaboration 
 *     Of the original authors are prohibited
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#include "nav/force_reactive_robot_companion_learning.h"
#include "random/rand_gmm.h"
#include "math.h"

Cforce_reactive_robot_companion_learning::Cforce_reactive_robot_companion_learning():
alpha_std_(0.1), beta_std_(0.1), gamma_std_(3.0), delta_std_(0.5), alpha_(0.0),beta_(1.0),cont_(0)
{
	previous_params_.push_back( 0.7 );//alpha
	previous_params_.push_back( 0.3 );//beta
	previous_params_.push_back( 5.0 );//interaction persons
	previous_params_.push_back( 0.5 );//obstacles laser
	previous_params_.push_back( 0.5 );//obstacles maps

	//set random seed for the rng
	rand_seed();

}

Cforce_reactive_robot_companion_learning::~Cforce_reactive_robot_companion_learning()
{

}

std::vector<double> Cforce_reactive_robot_companion_learning::learning_force_companion_params_MC( )
{
	//propose new robot companion parameters using MC
	//settings are ready for new experiments, but has to be activated somewhere

	//5- set next param_previous
	std::vector<double> params = param_force_approach_;//parameters returning the below work


	//4- sample params depending on previous params, only gamma and delta are set
	double gamma, delta;
	//while ( alpha <= 0.0 || alpha > 2.0 )
	//	alpha = rand_normal( previous_params_[0] ,alpha_std_);
	while ( gamma <= 1.0 || gamma > 15.0 )
		gamma = rand_normal( previous_params_[2] ,4*gamma_std_);
	while ( delta <= 0.1 || delta > 1.0 )
		delta = rand_normal( previous_params_[4] ,4*delta_std_);
	//param_force_approach_[0] = alpha;
	//param_force_approach_[1] = beta;
	param_force_approach_[2] = gamma;
	param_force_approach_[4] = delta;

	return params;
}

std::vector<double> Cforce_reactive_robot_companion_learning::learning_force_companion_params_alphabeta( )
{
	std::vector<double> params = param_force_approach_;

	cont_++;
	if ( cont_ > 10 )//5 times per parameters value
	{
		cont_ = 0;
		alpha_ += 0.02;
		if ( alpha_ > 1.0 ) alpha_ = 1.0;
		beta_ = 1 - alpha_;
	}
	params[0] = alpha_; //alpha
	params[1] = beta_; //beta
	param_force_approach_ = params;
	return params;
}
