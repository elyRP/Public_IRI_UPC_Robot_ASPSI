/*
 * test_geometry.cpp
 *
 * TODO complete the test, it was done after the designe of the Cpersons classes
 *
 *  Created on: Oct 1, 2013
 *      Author: gferrer
 *      Notice: It is an implementation test used by Gonzalo to test the initial functionalities of this code. 
 *              The next tests were done directly in simulation and with a real robot 
 *              (solo navigation by Gonzalo and other accompaniment and approaching by Ely).
 */
/*
 *
 *  Created on: Dec 22, 2013 by Gonzalo Ferrer. 
 *  Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration)
 *      Author: Initial code of the Robot alone AKP navigation is from Gonzalo Ferrer (2013).
 *      Author: Additions and new codes of ASPSI for people accompaniment code of Ely Repiso (from 2015 and currently).
 *      license (for other authors that will not be the original ones): CC BY-NC-ND 4.0 
 *              (Attribution-NonCommercial-NoDerivatives 4.0 International)
 *              https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
 *      This license does not allow other authors to modify or to take profit from these works. 
 *      Then, for modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
 *   collaborations (for journals with other researchers, formal collaborations between UPC and companies, and so on).
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
#define BOOST_TEST_MODULE geometry_test
#include <boost/test/unit_test.hpp>
#include "iri_geometry.h"


BOOST_AUTO_TEST_CASE( point )
{
    // testing if construction creates Spoint by default
	Spoint p;
    BOOST_CHECK( p.x == 0 && p.y == 0 && p.time_stamp == 0 );

    Spoint p2 = Spoint( 1.0, 3.0, 54.3);
	BOOST_CHECK( p2.x == 1.0 && p2.y == 3.0 && p2.time_stamp == 54.3 );

	p = p2;
	BOOST_CHECK( p.x == 1.0 && p.y == 3.0 && p.time_stamp == 54.3 );

	Spoint p3 = p2-p;
    BOOST_CHECK( p3.x == 0 && p3.y == 0 && p3.time_stamp == 54.3 );

    p2 = p2 * (-1);
    p3 = p2 + p;
    BOOST_CHECK( p3.x == 0 && p3.y == 0 && p3.time_stamp == 54.3 );

    p = Spoint( 5,0 );
    BOOST_CHECK( p.distance( Spoint() ) == 5.0  );
    BOOST_CHECK( p.distance( ) == 5.0  );

    p = Spoint( 3,4 );
	BOOST_CHECK( p.distance( Spoint() ) == 5.0  );
	BOOST_CHECK( p.distance( ) == 5.0  );

	p2 = p.propagate(0.1);
	BOOST_CHECK( p2.distance(p) == 0.0  );

	p2 = p.propagate(0.1,Sforce(), 1.0);
	BOOST_CHECK( p2.distance(p) == 0.0  );
}


BOOST_AUTO_TEST_CASE( point_cov )
{
   // testing if construction creates Spoint by default
	Spoint_cov p;
	BOOST_CHECK( p.x == 0 && p.y == 0 && p.time_stamp == 0 );
	BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0.0 &&  p.cov[2] == 0 && p.cov[3] == 1.0);

	p = Spoint_cov(1.0,2.0,0.0);
	BOOST_CHECK( p.x == 1.0 && p.y == 2.0 && p.time_stamp == 0.0 );
	BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0 &&  p.cov[2] == 0.0 && p.cov[3] == 1.0);

	//pass invalid covariance matrix (<4)
	std::vector<double> cov(3,0.0);
	p = Spoint_cov(1.0,2.0,0.0,cov);
	BOOST_CHECK( p.x == 1.0 && p.y == 2.0 && p.time_stamp == 0.0 );
	BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0 &&  p.cov[2] == 0.0 && p.cov[3] == 1.0);

	cov.resize(4,0.0);
	cov[0]=3.0;
	p = Spoint_cov(1.0,1.0,0.0,cov);
	BOOST_CHECK( p.x == 1.0 && p.y == 1.0 && p.time_stamp == 0.0 );
	BOOST_CHECK( p.cov[0] == 3.0 && p.cov[1] == 0 &&  p.cov[2] == 0.0 && p.cov[3] == 0);

	//operators
	Spoint_cov p2 = Spoint_cov(1.0,-1.0,0.0,cov);
    Spoint_cov p3 = p2 + p;
    BOOST_CHECK( p3.x == 2.0 && p3.y == 0.0 && p3.time_stamp == 0.0 );
    BOOST_CHECK( p3.cov[0] == 3.0 && p3.cov[1] == 0 &&  p3.cov[2] == 0.0 && p3.cov[3] == 0);

    p3 = p - p;
    BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 && p3.time_stamp == 0.0 );
    BOOST_CHECK( p3.cov[0] == 3.0 && p3.cov[1] == 0 &&  p3.cov[2] == 0.0 && p3.cov[3] == 0);

    p = p*2.5;
    BOOST_CHECK( p.x == 2.5 && p.y == 2.5 && p.time_stamp == 0.0 );


	//distances
    p = Spoint_cov(2.0,4.0,.0,cov);
	p2 = Spoint_cov( 5,0 );
    BOOST_CHECK( p2.distance( p ) == 5.0  );
    Spoint p4 = (Spoint)p;
    BOOST_CHECK( p2.distance2( p4 ) == 25.0  );
    BOOST_CHECK( p2.distance( ) == 5.0  );

    //covariances distances: default identity
    BOOST_CHECK( p2.cov_dist( p ) == 25.0  );
    BOOST_CHECK( p2.cov_dist( Spoint() ) == 25.0  );
    BOOST_CHECK( p2.cov_dist(  ) == 25.0  );

    //covariances distances: matrix cov
    p2.cov[0]=2.0;
    p2.cov[3]=2.0;
    BOOST_CHECK( p2.cov_dist( p ) == 12.5  );
    BOOST_CHECK( p2.cov_dist( Spoint() ) == 12.5  );
    BOOST_CHECK( p2.cov_dist(  ) == 12.5  );


}

BOOST_AUTO_TEST_CASE( point_velocity )
{
    // testing if construction creates Spoint by default
	SpointV p;
    BOOST_CHECK( p.x == 0 && p.y == 0 && p.vx == 0 && p.vy == 0 && p.time_stamp == 0 );

    SpointV p2 = SpointV( 1.0, 3.0, 54.3, 1.0, -1.0);
	BOOST_CHECK( p2.x == 1.0 && p2.y == 3.0 );
	BOOST_CHECK( p2.vx == 1.0 && p2.vy == -1.0);
	BOOST_CHECK( p2.time_stamp == 54.3 );

	p = p2;
	BOOST_CHECK( p.x == 1.0 && p.y == 3.0 );
	BOOST_CHECK( p.vx == 1.0 && p.vy == -1.0);
	BOOST_CHECK( p.time_stamp == 54.3 );

	SpointV p3 = p2-p;
	BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 );
	BOOST_CHECK( p3.vx == 0.0 && p3.vy == 0.0);
	BOOST_CHECK( p3.time_stamp == 0.0 );

    p2 = p2 * (-1);
    //p2.Spoint::print();
    //p2.print();
    p3 = p2 + p;
	BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 );
	BOOST_CHECK( p3.vx == 0.0 && p3.vy == 0.0);
	BOOST_CHECK( p3.time_stamp == 54.3 );

    p = SpointV( 5,0 );
    BOOST_CHECK( p.distance( SpointV() ) == 5.0  );
    BOOST_CHECK( p.distance( ) == 5.0  );

    p = SpointV( 3,4,0, 1,0 );
	BOOST_CHECK( p.distance( SpointV() ) == 5.0  );
	BOOST_CHECK( p.distance( ) == 5.0  );


	p2 = p.propagate(1);
	BOOST_CHECK( p2.x == 4 && p2.y == 4  );

	p2 = p.propagate(1, Sforce(), 0.5);
	BOOST_CHECK( p2.x == 3.5 && p2.y == 4  );

	p2 = p.propagate(1, Sforce(1,0), 5.0 );
	//p2.print();
	BOOST_CHECK( p2.x == 4.5 && p2.vx == 2  );

    //compatibility to other geometry classes
    BOOST_CHECK( p.distance( Spoint() ) == 5.0  );
}
BOOST_AUTO_TEST_CASE( point_velocity_covariance )
{
	// constructor by default
	SpointV_cov p;
	BOOST_CHECK( p.x == 0 && p.y == 0 && p.time_stamp == 0 && p.vx == 0.0 &&p.vy==0.0);
	BOOST_CHECK( p.cov[0] == 0.4 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
			     p.cov[4] == 0 && p.cov[5] == 0.4 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
			     p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.1 && p.cov[11] == 0 &&
			     p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.1);

	p = SpointV_cov(1.0,3.0,23.8,1.0,2.3);
	BOOST_CHECK( p.x == 1.0 && p.y == 3.0 && p.time_stamp == 23.8 && p.vx == 1.0 &&p.vy==2.3);
	BOOST_CHECK( p.cov[0] == 0.4 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
			     p.cov[4] == 0 && p.cov[5] == 0.4 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
			     p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.1 && p.cov[11] == 0 &&
			     p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.1);

	//invalid covariance
    std::vector<double> cov(3,0.0);
    p = SpointV_cov(1.0,3.0,23.8,1.0,2.3,cov);
    BOOST_CHECK( p.x == 1.0 && p.y == 3.0 && p.time_stamp == 23.8 && p.vx == 1.0 &&p.vy==2.3);
	BOOST_CHECK( p.cov[0] == 0.4 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
			     p.cov[4] == 0 && p.cov[5] == 0.4 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
			     p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.1 && p.cov[11] == 0 &&
			     p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.1);

    //valid covariance
    cov.resize(16,0.0);
    cov[0]=1.0;
    cov[5]=10;
    p = SpointV_cov(1.0,1.0,23.8,1.0,-1.0,cov);
    BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
                 p.cov[4] == 0 && p.cov[5] == 10.0 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
                 p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.0 && p.cov[11] == 0 &&
                 p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.0);

    SpointV pp(1.0,1.0,23.8,1.0,-1.0);
    p = SpointV_cov( pp, cov );
    BOOST_CHECK( p.x == 1.0 && p.y == 1.0 && p.time_stamp == 23.8 && p.vx == 1.0 &&p.vy==-1.0);
    BOOST_CHECK( p.cov[0] == 1.0 && p.cov[1] == 0 &&  p.cov[2] == 0 && p.cov[3] == 0 &&
                     p.cov[4] == 0 && p.cov[5] == 10.0 &&  p.cov[6] == 0 && p.cov[6] == 0 &&
                     p.cov[8] == 0 && p.cov[9] == 0 &&  p.cov[10] == 0.0 && p.cov[11] == 0 &&
                     p.cov[12] == 0 && p.cov[13] == 0 &&  p.cov[14] == 0 && p.cov[15] == 0.0);


    //operators
    SpointV_cov p2 = p;
    SpointV_cov p3 = p2-p;
    BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 );
    BOOST_CHECK( p3.vx == 0.0 && p3.vy == 0.0);
    BOOST_CHECK( p3.time_stamp == 0.0 );

    p2 = p2 * (-1);
    //p2.Spoint::print();
    //p2.print();
    p3 = p2 + p;
    BOOST_CHECK( p3.x == 0.0 && p3.y == 0.0 );
    BOOST_CHECK( p3.vx == 0.0 && p3.vy == 0.0);
    BOOST_CHECK( p3.time_stamp == 23.8 );

    //check propagation
    p = SpointV_cov(1.0,1.0,23.8,1.0,-1.0);
    p = p.propagate( 1.0 );

}
