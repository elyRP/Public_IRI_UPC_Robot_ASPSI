/*
 * rand_gmm.cpp
 *
 *  Created on: Dec 22, 2013 by Gonzalo Ferrer. 
 *  Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration)
 *      Author: Initial code of the Robot alone AKP navigation is from Gonzalo Ferrer (2013).
 *      Author: Additions and new codes of ASPSI for people accompaniment code of Ely Repiso (from 2015 and currently).
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
#include "rand_gmm.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>

double rand_uniform (  )
{
	return (double)rand()/(double)RAND_MAX;
}

double rand_uniform ( double min, double max )
{
	return min + (max-min)* rand_uniform();
}

int rand_uniform_discrete ( int min, int max )
{
	//return min + int ( (double)(max-min+1)* rand_uniform());
	return min + rand() % ( max-min+1 );
}

double rand_normal(double mu,double sigma)
{
	double u = rand_uniform();
	double v = rand_uniform();
	double x = sqrt( -2.0*log(u) )*cos( 2*M_PI*v );
	return mu + sigma*x;
}

double rand_gmm ( std::vector<Sgauss> & gmm)
{
		double p = rand_uniform () , p_accumulated = 0.0;
		unsigned int i = 0;
		while ( i < gmm.size() && p_accumulated < p)
		{
			p_accumulated += gmm.at(i).w;
			i++;
		}
		i--;
		return rand_normal ( gmm.at(i).mu , gmm.at(i).sigma );
}

void rand_seed()
{
	srand ( time(NULL) );
}
