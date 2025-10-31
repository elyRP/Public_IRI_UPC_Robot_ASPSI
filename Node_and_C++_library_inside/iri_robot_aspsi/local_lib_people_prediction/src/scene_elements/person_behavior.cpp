/*
 * person_behavior.cpp
 *
 *  Created on: Dec 17, 2013.  Last Modified on 2025 by Ely
 *      Author: Initial code of the Robot alone AKP navigation is from Gonzalo Ferrer. 
 *      Author: The Addition of ASPSI and other approaching and accompaniment modifications and codes are of Ely Repiso.
 */
/*
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
#include "scene_elements/person_behavior.h"
#include <math.h>
#include <iostream>

Cperson_behavior::Cperson_behavior(unsigned int id, Cperson_abstract::target_type person_target_type,
		Cperson_abstract::force_type person_force_type, double _time_window) :
    Cperson_bhmip(id,person_target_type, person_force_type, _time_window),
	debug_prediction_propagation_(false),
	debug_add_pointV_(false),
	debug_Cperson_bhmip_add_pointV_(false)
{
	prediction_trajectory_.reserve(1000);//default value for the prediction trajectory
}

Cperson_behavior::~Cperson_behavior()
{

}

void Cperson_behavior::add_pointV( SpointV_cov point ,Cperson_abstract::filtering_method filter,  bool robot_or_person)
{
	if(debug_add_pointV_){
		std::cout<<" (Cperson_behavior) entro en add_pointV!"<< std::endl;
	}
	Cperson_bhmip::add_pointV(point,filter ,robot_or_person);
	//calculate expected behavior is done in the estimation methd (in scene)
}

void Cperson_behavior::Cperson_bhmip_add_pointV( SpointV_cov point ,Cperson_abstract::filtering_method filter, bool person_or_robot)
{
	if(debug_Cperson_bhmip_add_pointV_){
		std::cout<<" (Cperson_behavior) entro en add_pointV!"<< std::endl;
	}

	Cperson_bhmip::Cperson_bhmip_add_pointV(point,filter,person_or_robot);
	//calculate expected behavior is done in the estimation methd (in scene)
}

void Cperson_behavior::prediction(double min_v_to_predict)
{
	Cperson_bhmip::prediction( min_v_to_predict);
	//behavior estimation is required to be done jointly with all person, so it is calculated outside the class
}

Sbehavior* Cperson_behavior::find_behavior_estimation( unsigned int id )
{
	std::list<Sbehavior>::iterator iit = expected_behavior_list_.begin();
	Sbehavior *behavior;
	if ( !expected_behavior_list_.empty() )
	{
		for( ; iit != expected_behavior_list_.end(); iit++ )
		{
			if ( iit->related_person_id == id )
			{
				behavior = &(*iit);
				assert( behavior != NULL );
				return  behavior;
			}
			if ( iit->related_person_id > id )
				break;
		}
	}
	//behavior not found for person id, then a new behavior is set
	expected_behavior_list_.insert( iit, Sbehavior( id ) );
	iit--;//element inserted is before iit, so we want the pointer to the inserted element
	behavior = &(*iit);
	assert( behavior != NULL );
	return  behavior;
}

Cperson_abstract::behavior_type
Cperson_behavior::get_best_behavior_to_person( unsigned int interacting_person ) const
{
	Cperson_abstract::behavior_type result = Cperson_abstract::Balanced;
	const Sbehavior * behavior = NULL;
	std::list<Sbehavior>::const_iterator iit = expected_behavior_list_.begin();
	if ( !expected_behavior_list_.empty() )
	{
		for( ; iit != expected_behavior_list_.end(); iit++ )
		{
			if ( iit->related_person_id == interacting_person )
			{
				behavior = &(*iit);
				assert( behavior != NULL );
			}
			if ( iit->related_person_id > interacting_person )
				break;//person not found, so result is a balanced behavior
		}
	}
	if ( behavior == NULL ) return result;
	assert( behavior != NULL );
	double best_expectation =  behavior->expectation[0];
	for ( unsigned int i = 1; i < behavior->expectation.size(); ++i  )
	{
		if( best_expectation < behavior->expectation[i]  )
		{
			best_expectation = behavior->expectation[i];
			result = (Cperson_abstract::behavior_type)i;
		}
	}

	return result;
}

void Cperson_behavior::prediction_propagation( double dt , Sforce force , unsigned int index )
{

	if(debug_prediction_propagation_){
		std::cout << "IN Cperson_behavior::prediction_propagation prediction_trajectory_.emty()"<<prediction_trajectory_.empty()<< std::endl;
		std::cout <<" dt="<<dt<<";desired_velocity_="<<desired_velocity_<<"; force.fx"<<force.fx<<"; force.fy="<<force.fy<<"; prediction_trajectory_.at(index).print():"<< std::endl;
		prediction_trajectory_.at(index).print();
	}
	prediction_trajectory_.push_back( prediction_trajectory_.at(index).propagate(dt,force,desired_velocity_) );
}

void Cperson_behavior::planning_propagation( double dt , Sforce force , unsigned int index )
{
	planning_trajectory_.push_back( planning_trajectory_.at(index).propagate(dt,force,desired_velocity_) );
}

void Cperson_behavior::planning_propagation_copy( unsigned int prediction_index )
{
	planning_trajectory_.push_back( prediction_trajectory_.at(prediction_index) );
}

bool Cperson_behavior::is_needed_to_propagate_person_for_planning( unsigned int parent_index, Spoint robot, unsigned int& new_index_to_be_copied )
{
	bool res;
	//TODO first iteration: only checks distance to target

	double distance=1; // Gonzalo has 1, as distance, I need more distance.
	//std::cout << "has_copied_propagation_="<<has_copied_propagation_<< std::endl;

	if ( robot.distance( planning_trajectory_.at( parent_index ) ) < distance && !has_copied_propagation_ )
	{
	//	std::cout << "robot.distance( planning_trajectory_.at( parent_index ) ) < distance; d="<<robot.distance( planning_trajectory_.at( parent_index ) )<< std::endl;
		res = true;
	}
	else
	{

		has_copied_propagation_ = true;
		unsigned int index(0);
	//	std::cout << "has_copied_propagation_="<<has_copied_propagation_<< std::endl;
		while( robot.time_stamp > prediction_trajectory_.at(index).time_stamp  && index < prediction_trajectory_.size()-1 )
		{
			index++;
		}
		//std::cout << "new_index_to_be_copied="<<index<< std::endl;
		new_index_to_be_copied = index;
		res = false;
	}

	return res;
}

void Cperson_behavior::clear_prediction_trajectory()
{
	prediction_trajectory_.clear();
	prediction_trajectory_.push_back( current_pointV_ );
	planning_trajectory_.clear();
	planning_trajectory_.push_back( current_pointV_ );
	has_copied_propagation_ = false;
}

void Cperson_behavior::clear_planning_trajectory()
{
	planning_trajectory_.clear();
	planning_trajectory_.push_back( current_pointV_ );
	has_copied_propagation_ = false;
}

void Cperson_behavior::reserve_prediction_trajectory( unsigned int n)
{
	prediction_trajectory_.reserve( n );//if n < capacity() then does nothing
}

void Cperson_behavior::reset(  )
{
	Cperson_bhmip::reset();
	expected_behavior_list_.clear();
	prediction_trajectory_.clear();
	planning_trajectory_.clear();
	std::vector<double> reset_cov; // MADE BY Ely (reset covariances person simulated. Augment more with huge velocities, in start, I need to reset!)
	reset_cov.reserve(16);
	reset_cov.resize(16,0.0);
	reset_cov[0] = 0.5;//0.4;
	reset_cov[5] = 0.5;
	reset_cov[10] = 0.1; //0.1
	reset_cov[15] = 0.1;
	current_pointV_.cov=reset_cov;

}


