/*
 * learninglinnearregression.h
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

#ifndef SRC_LEARNING_PARAMETERS_LEARNNINGLINEARREGRESSION_H_
#define SRC_LEARNING_PARAMETERS_LEARNNINGLINEARREGRESSION_H_

/*class learning_linnear_regression {
};*/

//#endif /* SRC_LEARNING_PARAMETERS_LEARNINGLINNEARREGRESSION_H_ */

/*
 * learningleastsquaresregression.h
 *
 *  Created on: May 19, 2017
 *      Author: ely7787
 */

//#ifndef SRC_LEARNING_PARAMETERS_LEARNINGLEASTSQUARESREGRESSION_H_
//#define SRC_LEARNING_PARAMETERS_LEARNINGLEASTSQUARESREGRESSION_H_

class learning_least_squares_regression {
public:
	// variables de la clase
	double learnning_rate_; // alpha=0.001 parametro a sacar y modificar con el dynamic reconfigure.
							// es el step de lo que cambia el error el valor de los parametros.
	double beta0_k_1_;
	double beta1_k_1_;
	double act_regression_error;

	double real_y_value_;
	double real_x_value_;

	// funciones de la clase
	learning_least_squares_regression();
	virtual ~learning_least_squares_regression();
	void least_squares_iteration();
	void least_squares_initialice();
	void least_squares_calc_error();
};

//#endif /* SRC_LEARNING_PARAMETERS_LEARNINGLEASTSQUARESREGRESSION_H_ */


/*
 * learnningBayesianregression.h
 *
 *  Created on: May 19, 2017
 *      Author: ely7787
 */

//#ifndef SRC_LEARNING_PARAMETERS_LEARNNINGBAYESIANREGRESSION_H_
//#define SRC_LEARNING_PARAMETERS_LEARNNINGBAYESIANREGRESSION_H_

class learnning_Bayesian_regression {
public:
	learnning_Bayesian_regression();
	virtual ~learnning_Bayesian_regression();
};

//#endif /* SRC_LEARNING_PARAMETERS_LEARNNINGBAYESIANREGRESSION_H_ */


/*
 * learningleastsquarespercentageregression.h
 *
 *  Created on: May 19, 2017
 *      Author: ely7787
 */

//#ifndef SRC_LEARNING_PARAMETERS_LEARNINGLEASTSQUARESPERCENTAGEREGRESSION_H_
//#define SRC_LEARNING_PARAMETERS_LEARNINGLEASTSQUARESPERCENTAGEREGRESSION_H_

class learning_least_squares_percentage_regression {
public:
	learning_least_squares_percentage_regression();
	virtual ~learning_least_squares_percentage_regression();
};

//#endif /* SRC_LEARNING_PARAMETERS_LEARNINGLEASTSQUARESPERCENTAGEREGRESSION_H_ */

/*
 * learnningleastabsolutedeviationregression.h
 *
 *  Created on: May 19, 2017
 *      Author: ely7787
 */

//#ifndef SRC_LEARNING_PARAMETERS_LEARNNINGLEASTABSOLUTEDEVIATIONREGRESSION_H_
//#define SRC_LEARNING_PARAMETERS_LEARNNINGLEASTABSOLUTEDEVIATIONREGRESSION_H_

class learnning_least_absolute_deviation_regression {
public:
	learnning_least_absolute_deviation_regression();
	virtual ~learnning_least_absolute_deviation_regression();
};

#endif /* SRC_LEARNING_PARAMETERS_LEARNNINGLEASTABSOLUTEDEVIATIONREGRESSION_H_ */
