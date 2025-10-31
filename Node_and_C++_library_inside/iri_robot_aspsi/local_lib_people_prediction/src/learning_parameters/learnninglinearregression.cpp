/*
 * learninglinnearregression.cpp
 *
 *  Created on: May 19, 2017
 *      Unique Author: Ely Repiso (Also, I think that I have this code in another place because it is not complete here.)
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
#include "learnninglinearregression.h"


/*
 * learningleastsquaresregression.cpp
 *
 *  Created on: May 19, 2017
 *      Author: ely7787
 */

//#include "learningleastsquaresregression.h"

learning_least_squares_regression::learning_least_squares_regression():
learnning_rate_(0.001) // parametro a sacar fuera y modificar con el dynamic reconfigure.
{
	// TODO Auto-generated constructor stub

}

learning_least_squares_regression::~learning_least_squares_regression() {
	// TODO Auto-generated destructor stub
}

void learning_least_squares_regression::least_squares_iteration(){
	/* */
	// calculate error:
	least_squares_calc_error();
	//double new_beta0_k=0;
	//double new_beta1_k=0;

	//new_beta0_k=beta0_k_1_-learnning_rate_*act_regression_error;
	//new_beta1_k=beta1_k_1_-learnning_rate_*act_regression_error*real_x_value_;

}

void learning_least_squares_regression::least_squares_initialice(){
	beta0_k_1_=0;
	beta1_k_1_=0;

}

void learning_least_squares_regression::least_squares_calc_error(){
	// we use the squared error.

}
/*
 * learnningBayesianregression.cpp
 *
 *  Created on: May 19, 2017
 *      Author: ely7787
 */

//#include "learnningBayesianregression.h"

learnning_Bayesian_regression::learnning_Bayesian_regression() {
	// TODO Auto-generated constructor stub

}

learnning_Bayesian_regression::~learnning_Bayesian_regression() {
	// TODO Auto-generated destructor stub
}



/*
 * learningleastsquarespercentageregression.cpp
 *
 *  Created on: May 19, 2017
 *      Author: ely7787
 */

//#include "learningleastsquarespercentageregression.h"

learning_least_squares_percentage_regression::learning_least_squares_percentage_regression() {
	// TODO Auto-generated constructor stub

}

learning_least_squares_percentage_regression::~learning_least_squares_percentage_regression() {
	// TODO Auto-generated destructor stub
}

/*
 * learnningleastabsolutedeviationregression.cpp
 *
 *  Created on: May 19, 2017
 *      Author: ely7787
 */

//#include "learnningleastabsolutedeviationregression.h"

learnning_least_absolute_deviation_regression::learnning_least_absolute_deviation_regression() {
	// TODO Auto-generated constructor stub

}

learnning_least_absolute_deviation_regression::~learnning_least_absolute_deviation_regression() {
	// TODO Auto-generated destructor stub
}
