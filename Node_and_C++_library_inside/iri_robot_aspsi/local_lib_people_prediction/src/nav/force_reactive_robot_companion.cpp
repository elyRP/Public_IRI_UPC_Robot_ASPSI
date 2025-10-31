/*
 * force_reactive_robot_companion.cpp
 *
 *  Created on: Nov 9, 2013
 *      Author: Gonzalo. Deprecated, do mot used. The new companion by Ely Repiso is anticipative not reactive. Then, it not includes this code.
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


#include "nav/force_reactive_robot_companion.h"
#include <math.h>
#include <iostream>

Cforce_reactive_robot_companion::Cforce_reactive_robot_companion ( ) :
	Cprediction_bhmip(),
	target_person_id_(0),
	robot_state_( UNKNOWN_ZONE ),
	v_max_(2.0),
	v_cruise_(1.0),
	time_horizon_(0.2),
	is_target_person_visible_(false)
{
	param_force_approach_.push_back(0.1);//force goal
	param_force_approach_.push_back(1.0);//force persongoal
	param_force_approach_.push_back(7.0);//force interaction persons
	param_force_approach_.push_back(1.0);//force interaction laser
	param_force_approach_.push_back(1.0);//force interaction map

}

Cforce_reactive_robot_companion::~Cforce_reactive_robot_companion()
{
	//parent class do their destructors and no need to do anything free memory in this class
}

SpointV_cov Cforce_reactive_robot_companion::robot_approach_pose(bool stop_if_smn_in_inner)
{  //stop_if_smn_in_inner=> true para anclarse a la persona más cercana. (lo usa para real, darle el id de la persona más cercana!)
	Sforce f1,f2,f3,f4,f5,f;
	return robot_approach_pose(f1,f2,f3,f4,f5,f,stop_if_smn_in_inner );
}

SpointV_cov Cforce_reactive_robot_companion::robot_approach_pose( Sforce& f_goal, Sforce& f_persongoal,
	Sforce& f_int_pers, Sforce& f_int_laser, Sforce& f_int_map, Sforce& f , bool stop_if_smn_in_inner)
{
	std::cout << " IN!!! Cforce_reactive_robot_companion::robot_approach_pose" << std::endl;
	//get initial conditions

	//robot_->print();
	internal_robot_->print();
	std::cout << " desp_robot_print (0.1)" << std::endl;
	//Spose act_robot=robot_->get_current_pose();
	Spose act_robot=internal_robot_->get_current_pose();
	std::cout << " act_robot (0.1)" << std::endl;
	act_robot.print();
	double vx=act_robot.v*cos(act_robot.theta);
	std::cout << " vx="<<vx << std::endl;
	double vy=act_robot.v*sin(act_robot.theta);
	std::cout << " vy="<<vy << std::endl;
	SpointV_cov robot=SpointV_cov(act_robot.x , act_robot.y, act_robot.time_stamp, vx,vy);
	//SpointV_cov robot = robot_->get_current_pointV();
	std::cout << " (0.1)" << std::endl;
	SpointV_cov approach;
	std::cout << " (0.2)" << std::endl;
	approach.x = robot.x;
	std::cout << " (0.3)" << std::endl;
	//approach.x = 1.0/0.0; //sending invalid commands to stop movement
	approach.y = robot.y;
	std::cout << " (0.4)" << std::endl;
	//approach.y = 1.0/0.0;
	approach.time_stamp = robot.time_stamp;
	std::cout << " (0.5)" << std::endl;
	approach.vx = 0.0; //no approach, stays at current position at v=0 TODO
	approach.vy = 0.0;
	std::cout << " (0.6)" << std::endl;
	Cperson_abstract* person_obj;
	std::cout << " (1)" << std::endl;
	//checking if stop_if_smn_in_inner flag is set
	if( stop_if_smn_in_inner ) // por defecto es false!, entonce no pilla la más cercana, sino el caso siguiente.
	{
		std::cout << " (1.1)" << std::endl;
		//Cperson_abstract* nearest = nearest_person( robot_ );
		Cperson_abstract* nearest = nearest_person( internal_robot_ );
		//double phi = robot_->get_current_pointV().angle_heading_point( nearest->get_current_pointV() );
		double phi = internal_robot_->get_current_pointV().angle_heading_point( nearest->get_current_pointV() );
		double lambda = 0.5;
		double anisotropy = (lambda + (1-lambda)*(1 + cos(phi))/2 );
		if( distance_to_nearest_person(internal_robot_) * anisotropy < 0.8  )
		//if( distance_to_nearest_person(robot_) * anisotropy < 0.8  )
		{
			return approach;
		}
	}
	std::cout << " (2)" << std::endl;
	person_obj=internal_person_obj_;
		//if (!find_person(target_person_id_ , &person_obj)  )
	if (!find_person(target_person_id_ , &person_obj)  ) // no tienes cerca a la persona que acompañas, desconoces la zona.
	{													// pero luego al poner is_target_person_visible_=true, creo k no usa esto...
		std::cout << " param_force_approach_[0]="<<param_force_approach_[0] << std::endl;
		std::cout << " param_force_approach_[1]="<<param_force_approach_[1] << std::endl;
		std::cout << " param_force_approach_[2]="<<param_force_approach_[2] << std::endl;
		std::cout << " param_force_approach_[3]="<<param_force_approach_[3] << std::endl;
		std::cout << " param_force_approach_[4]="<<param_force_approach_[4] << std::endl;
		std::cout << " (2.1)" << std::endl;
		person_obj->print();
		std::cout << " after print (2.1)" << std::endl;
		is_target_person_visible_ = false;
		robot_state_ = UNKNOWN_ZONE;
		f_goal = Sforce();
		f_persongoal = Sforce();
		std::cout << " (2.2)" << std::endl;
		//f_int_pers = force_persons_int( robot_  ) * param_force_approach_[2];
		f_int_pers = force_persons_int( internal_robot_ ) * param_force_approach_[2];
		std::cout << " (2.3) f_int_pers:"<< std::endl;
		f_int_pers.print();
		//f_int_laser = force_objects_laser_int( robot_ ) * param_force_approach_[3];
		f_int_laser = force_objects_laser_int( internal_robot_ ) * param_force_approach_[3];
		std::cout << " (2.4) f_int_laser:" << std::endl;
		f_int_laser.print();
		//f_int_map =  get_force_map_robot_position()* param_force_approach_[4];
		f_int_map =   act_force_map_robot_position_* param_force_approach_[4];
		std::cout << " f_int_map:" << std::endl;
		act_force_map_robot_position_.print();
		std::cout << " (2.5) f_int_map:" << std::endl;
		f_int_map.print();
		f = f_int_pers + f_int_map + f_int_laser;
		std::cout << " (2.6) f:" << std::endl;
		f.print();
		return approach;
	}
	std::cout << " (3)" << std::endl;
	is_target_person_visible_ = true;
	SpointV_cov person = person_obj->get_current_pointV(); // coge el punto de la persona.
	std::cout << " (4)" << std::endl;
	//calculate current robot destinations:
	std::vector<Sdestination> dest;
	dest.push_back( person_obj->get_best_dest() ); //en las destinaciones del robot se guarda la destinacion de la persona.
	std::cout << " (5)" << std::endl;
	//calculation of the person destination
	person_destination_ = Sdestination(0, person.x ,person.y,1.0);  // destinacion, para acercarse a la persona.
	//positive difference means on the left hand side of the angle circle thus, positive turn
	//if( diffangle(person.theta, atan2(robot.y-person.y , robot.x-person.x) ) < 0 )
	std::cout << " (6) person.x="<<person.x<<"; person.y"<<person.y << std::endl;
	//a pure geometric approach: makes the destination more stable in time
	double theta = atan2(person_obj->get_best_dest().y-person.y ,
						person_obj->get_best_dest().x-person.x); // angulo entre la destinacion a la que va la persona y la posicion de la persona.
	std::cout << " (7)" << std::endl;
	if( diffangle(theta, atan2(robot.y-person.y , robot.x-person.x) ) < 0 ) // diferendia entre angulo posición robot a persona y el angulo de persona hacia su goal.
	{
		std::cout << " (7.1)" << std::endl;
		//person_destination_.x += r_*cos(person.theta + ro_);
		person_destination_.x += r_*cos(theta + ro_); // intenta ir a un radio al rededor de la persona.
		person_destination_.y += r_*sin(theta + ro_);
	}
	else  // es como si hiciera una circunferencia al rededor de la persona. (se intenta poner a un lado o a otro de la persona. segun esos angulos)
	{
		std::cout << " (7.2)" << std::endl;
		person_destination_.x += r_*cos(theta - ro_);
		person_destination_.y += r_*sin(theta - ro_);
	}
	std::cout << " (8)" << std::endl;
	dest.push_back( person_destination_ );
	//robot_->set_destinations( dest );
	internal_robot_->set_destinations(dest);
	std::cout << " (9)" << std::endl;

	//calculate robot state and if unknown, do not move
	//set_robot_state(person,robot,  get_sfm_params(robot_)->at(1) );
	set_robot_state(person,robot,  get_sfm_params(internal_robot_)->at(1) );
	std::cout << " (10)" << std::endl;

	if ( robot_state_ == UNKNOWN_ZONE)
		return approach;

	std::cout << " (11)" << std::endl;

	//calculate robot forces
	if (robot.v() < 0.2 ){ //just to avoid
		robot.norm_v( v_cruise_);
	}
	std::cout << " (12)" << std::endl;
	//const std::vector<double>* social_forces_param = person_obj->get_social_force_parameters_person_robot();
	//robot_->set_desired_velocty(v_robot_desired_);
	internal_robot_->set_desired_velocty(v_robot_desired_);
	std::cout << " (13)" << std::endl;
	//f_goal = robot_->force_goal( person_obj->get_best_dest(), get_sfm_params(robot_),  &robot  )* param_force_approach_[0];
	f_goal = internal_robot_->force_goal( person_obj->get_best_dest(), get_sfm_params(internal_robot_),  &robot  )* param_force_approach_[0];
	std::cout << " (14)" << std::endl;
	//f_persongoal = robot_->force_goal( person_destination_ , get_sfm_params(robot_),  &robot  )* param_force_approach_[1];
	f_persongoal = internal_robot_->force_goal( person_destination_ , get_sfm_params(internal_robot_),  &robot  )* param_force_approach_[1];
	std::cout << " (15)" << std::endl;
	//f_persongoal = robot_force_persongoal(robot, person, social_forces_param)* param_force_approach_[1];
	//f_int_pers = force_persons_int( robot_  ) * param_force_approach_[2];
	f_int_pers = force_persons_int( internal_robot_  ) * param_force_approach_[2];
	std::cout << " (16)" << std::endl;
	//f_int_laser =   force_objects_laser_int( robot_ )*param_force_approach_[3];
	f_int_laser =   force_objects_laser_int( internal_robot_ )*param_force_approach_[3];
	std::cout << " (17)" << std::endl;
	//f_int_map =   get_force_map_robot_position()* param_force_approach_[4];
	f_int_map =   get_force_map_robot_position()* param_force_approach_[4];
	std::cout << " (18)" << std::endl;
	double w = 1.0;
	do
	{
		std::cout << " (18.1)" << std::endl;
		f_goal = f_goal * w;
		std::cout << " (18.2)" << std::endl;
		f_persongoal = f_persongoal * w;
		std::cout << " (18.3)" << std::endl;
		f_int_pers = f_int_pers *w;
		std::cout << " (18.4)" << std::endl;
		f = f_goal + f_persongoal +	f_int_pers + f_int_map + f_int_laser;
		std::cout << " (18.5)" << std::endl;
		approach = robot_tangential_propagation( f );
		std::cout << " (18.6)" << std::endl;
		w *= 0.9;
	} while( read_force_map_success_ && !is_cell_clear_map( approach.x, approach.y ) && w > 0.1 );
	std::cout << " (19)" << std::endl;
	//approach = get_robot().pose_propagation( dt_, f );
	return approach;
}

bool Cforce_reactive_robot_companion::set_nearest_target_person()
{
	//returns True if the target was set and false if no person was in the scene nearer than 20m
	/*Cperson_abstract* nearest = nearest_person( robot_ );
	if ( nearest->get_current_pointV().distance( robot_->get_current_pointV() )  < 20.0 && *nearest != *robot_ )
	{
		target_person_id_ = nearest->get_id();
		return true;
	}
	else
	{
		return false;
	}*/

	Cperson_abstract* nearest = nearest_person( internal_robot_ );
		if ( nearest->get_current_pointV().distance(internal_robot_->get_current_pointV() )  < 20.0 && *nearest != *internal_robot_ )
		{
			target_person_id_ = nearest->get_id();
			return true;
		}
		else
		{
			return false;
		}
}

Cperson_abstract* Cforce_reactive_robot_companion::nearest_person( Cperson_abstract* person)
{
	Cperson_abstract* nearest_person = person;
	double d = 2000, d_to_person;
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		d_to_person = (*iit)->get_current_pointV().distance( person->get_current_pointV() );
		if ( d_to_person < d && person != *iit )
		{
			d = d_to_person;
			nearest_person = *iit;
		}
	}
	return nearest_person;
}

double Cforce_reactive_robot_companion::distance_to_nearest_person( Cperson_abstract* person )
{
	Cperson_abstract* nearest = nearest_person( person );
	if (nearest == person)
		return -1.0;
	return nearest->get_current_pointV().distance( person->get_current_pointV() );

}


Sforce Cforce_reactive_robot_companion::robot_force_persongoal(SpointV_cov robot, SpointV_cov person, std::vector<double> social_forces_param )
{
	//force calculation
	/*robot_->set_desired_velocty( v_robot_desired_ );
	return robot_->force_goal(  person_destination_, get_sfm_params(robot_) );*/
	internal_robot_->set_desired_velocty( v_robot_desired_ );
	return internal_robot_->force_goal(  person_destination_, get_sfm_params(internal_robot_) );
}

void Cforce_reactive_robot_companion::set_robot_state( SpointV_cov person, SpointV_cov robot , double lambda )
{
	//calculation of the influence zone
	double phi = diffangle( person.orientation() , atan2(robot.y-person.y , robot.x-person.x) );
	double anisotropy = (lambda + (1-lambda)*(1 + cos(phi))/2 );

	if ( anisotropy*1.5 > robot.distance(person)  )
		robot_state_ = INNER_ZONE;
	else if ( anisotropy*3 > robot.distance(person) )
		robot_state_ = MID_ZONE;
	else
		robot_state_ = OUT_ZONE;

	switch(robot_state_)
	{
		case INNER_ZONE:
			//v_robot_desired_ = 1.5*person.v;
			v_robot_desired_ = v_cruise_;
			break;
		case MID_ZONE:
			//v_robot_desired_ = 2 * person.v;
			v_robot_desired_ = 1.5*v_cruise_;
			if (v_robot_desired_ < 0.2)//almost stopped
				v_robot_desired_ = v_cruise_;
			break;
		case OUT_ZONE:
			v_robot_desired_ = v_max_;
			break;
		case UNKNOWN_ZONE:
		default:
			v_robot_desired_ = 0.0;
			break;
	}
	//robot_->set_desired_velocty( v_robot_desired_ );
	internal_robot_->set_desired_velocty( v_robot_desired_ );

}

SpointV_cov Cforce_reactive_robot_companion::robot_tangential_propagation(Sforce f)
{
	//returns the propagated state given a time horizon of propagation
	// and assuming no other force is applied, except for the initial one
	//SpointV_cov robot = robot_->get_current_pointV();
	SpointV_cov robot = internal_robot_->get_current_pointV();
	//limits max acceleration
	//if (f.module() > a_max_)
	//	f = f * (a_max_ / f.module());

	//limits max velocity
	double vx = robot.vx + f.fx * dt_;
	double vy = robot.vy + f.fy * dt_;
	double v = robot.v();
	if (v > v_max_)
	{
		vx *= v_max_/v;
		vy *= v_max_/v;
		v = v_max_;
	}
	//check for destination proximity
	double time_propagated;
	//Sdestination dest = robot_->get_destinations()->at(1);//person destination
	Sdestination dest = internal_robot_->get_destinations()->at(1);//person destination
	double distance = robot.distance( SpointV_cov(dest.x,dest.y) );

	if (distance / v < time_horizon_) //propagated pose would be farther than destination
	{
		//time_propagated = 0.1; //NO FUNCIONA!robot remains in place, to avoid minor position changes that entail huge orientation differences...
		if (v < 0.05 ) v = 0.05;
		time_propagated = distance / v;
	}
	else
	{
		time_propagated = time_horizon_;
	}
	return SpointV_cov(robot.x + vx * time_propagated , robot.y + vy * time_propagated,
                                                                robot.time_stamp , vx, vy );

}
void Cforce_reactive_robot_companion::set_force_params( double force_goal, double force_toperson, double force_interaction)
{
	param_force_approach_[0] = force_goal;
	param_force_approach_[1] = force_toperson;
	param_force_approach_[2] = force_interaction;
}

void Cforce_reactive_robot_companion::set_force_params( double force_goal, double force_toperson)
{
	param_force_approach_[0] = force_goal;
	param_force_approach_[1] = force_toperson;
}

void Cforce_reactive_robot_companion::set_force_params( double force_goal, double force_toperson,
			double force_interaction, double force_laser, double force_map)
{
	param_force_approach_[0] = force_goal;
	param_force_approach_[1] = force_toperson;
	param_force_approach_[2] = force_interaction;
	param_force_approach_[3] = force_laser;
	param_force_approach_[4] = force_map;
}
void Cforce_reactive_robot_companion::robot_parameters_feedback_adjustment( double feedback_sign )
{
	//feedback_sing input is a double float of value 1.0 or -1.0

	switch( robot_state_ )
	{
		case INNER_ZONE:
			param_force_approach_[2] -= feedback_sign * 2.0;
			if( param_force_approach_[2] < 0 )
				param_force_approach_[2] = 0.0;
			break;

		case MID_ZONE:
		case OUT_ZONE:
		case UNKNOWN_ZONE:
			param_force_approach_[0] -=  feedback_sign * 0.1;
			if (param_force_approach_[0] > 1.0)
				param_force_approach_[0] = 1.0;
			if (param_force_approach_[0] < 0.0)
				param_force_approach_[0] = 0.0;

			param_force_approach_[1] += feedback_sign * 0.1;
			if (param_force_approach_[1] > 1.0)
				param_force_approach_[1] = 1.0;
			if (param_force_approach_[1] < 0.0)
				param_force_approach_[1] = 0.0;

			/*v_max_ += feedback_sign * 0.14;
			if (v_max_ > 1.6)
				v_max_ = 1.6;
			if (v_max_ < 0.6)
				v_max_ = 0.6;*/
			break;

		default:
			//do nothing
			break;
	}
}



