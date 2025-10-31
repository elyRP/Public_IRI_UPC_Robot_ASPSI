/*
 * test_Cperson.cpp
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
#define BOOST_TEST_MODULE Cperson_test
#include <boost/test/unit_test.hpp>
#include "scene_elements/person_abstract.h"
#include "scene_elements/person_virtual.h"
#include "scene_elements/person_bhmip.h"
#include "scene_elements/person_behavior.h"

//Alias in order to easily validate a new type of person
typedef Cperson_behavior T;
//typedef Cperson_virtual T;

BOOST_AUTO_TEST_CASE( construction_and_setting )
{
	// testing constructor to created a person with id
	unsigned int id = 0;
    Cperson_abstract* person = new T( id );
    BOOST_CHECK( person->get_id() == id );
    delete person;

    // testing if construction creates person of type Person
    person = new T( id, Cperson_abstract::Person );
    BOOST_CHECK( person->get_id() == id && person->get_person_type () == Cperson_abstract::Person);

    // testing if construction creates person of type Robot
    Cperson_abstract* person2 = new T( 1, Cperson_abstract::Robot , Cperson_abstract::Spherical);
    BOOST_CHECK( person2->get_id() == 1 && person2->get_person_type () == Cperson_abstract::Robot);

    // testing if operator != works fine (only for classes, not pointers!): different id
    BOOST_CHECK( *person != *person2 );

    // testing if operator != works fine: same id but different type
	delete person2;
	person2 = new T( id, Cperson_abstract::Robot );
    BOOST_CHECK( *person != *person2 );

    // testing if operator == works fine on itself
    BOOST_CHECK( *person == *person );

    // testing if operator == works fine: same id, same type but different force type
    delete person2;
    person2 = new T( id, Cperson_abstract::Person , Cperson_abstract::Spherical);
    BOOST_CHECK( *person == *person2 );

    delete person;
    delete person2;
}

BOOST_AUTO_TEST_CASE( prediction )
{
    Cperson_abstract* person = new T( 0 );

    // setting of destinations
    std::vector<Sdestination> dests;
    dests.push_back( Sdestination(1,10,0,0.25) );
    dests.push_back( Sdestination(2,10,-10,0.25) );
    dests.push_back( Sdestination(3,-10,-10,0.25) );
    dests.push_back( Sdestination(4,-10,-10,0.25) );
    person->set_destinations( dests );
    BOOST_CHECK( person->get_destinations()->size() == 4 );

    //propagation
    person->add_pointV(SpointV_cov(0,0,0),Cperson_abstract::Linear_regression_filtering);
    person->add_pointV(SpointV_cov(0.1,0,0.1),Cperson_abstract::Linear_regression_filtering);
    person->add_pointV(SpointV_cov(0.2,0,0.2),Cperson_abstract::Linear_regression_filtering);
    BOOST_CHECK( person->get_current_pointV().propagate(0.1).distance( Spoint(0.3,0) ) < 0.05 );


    delete person;
}
