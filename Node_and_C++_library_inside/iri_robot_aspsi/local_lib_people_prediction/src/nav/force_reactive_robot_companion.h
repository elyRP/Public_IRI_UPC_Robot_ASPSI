/*
 * force_reactive_robot_companion.h
 *
 *  Created on: Nov 9, 2013
 *      Author: gonzalo. Deprecated, do mot used. The new companion by Ely Repiso is anticipative not reactive.
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


#ifndef FORCE_REACTIVE_ROBOT_COMPANION_H_
#define FORCE_REACTIVE_ROBOT_COMPANION_H_

#include "prediction_bhmip.h"
#include "prediction_behavior.h"
#include "scene_elements/person_abstract.h"
#include "scene_abstract.h"
//#include "nav/plan_local_nav.h"
/**
 *
 * \brief Force reactive Robot companion Class
 *
 * The force_reactive_robot_companion class provides an algorithm, making use of
 * intentionality and motion human prediction in order to control
 * the most human-like approach of the robot towards a moving person.
 * The approach algorithm is based on forces calculation and
 * an input feedback is expected in order to weight the importance
 * of the interacting forces.
 *
 * This work was published in "xxx IROS'2013"
 *
 *
*/

class Cforce_reactive_robot_companion : public Cprediction_bhmip
{
	public:
		Cforce_reactive_robot_companion ();
		~Cforce_reactive_robot_companion();
		enum approach_state { UNKNOWN_ZONE=0, INNER_ZONE , MID_ZONE, OUT_ZONE };
		SpointV_cov robot_approach_pose(bool stop_if_smn_in_inner=false);
		SpointV_cov robot_approach_pose( Sforce& f_goal, Sforce& f_persongoal,
				Sforce& f_int_pers, Sforce& f_int_laser, Sforce& f_int_map, Sforce& f, bool stop_if_smn_in_inner=false);
		//temporal function while no global localization is set (yet...)
		void set_target_person(int id) { target_person_id_ = id;}
		bool set_nearest_target_person();
		int get_target_person() { return target_person_id_; }
		Cperson_abstract* nearest_person( Cperson_abstract* person);
		double distance_to_nearest_person(Cperson_abstract* person);
		approach_state get_robot_state() { return robot_state_; }
		void set_v_max(double v_max){v_max_ = v_max;}
		double get_v_max(){return v_max_;}
		void set_v_cruise(double v){v_cruise_ = v;}
		void set_time_horizon(double t){time_horizon_ = t;}
		std::vector<double> get_force_params() const { return param_force_approach_; }
		void set_force_params( double force_goal, double force_toperson);
		void set_force_params( double force_goal, double force_toperson, double force_interaction);
		void set_force_params( double force_goal, double force_toperson,
					double force_interaction, double force_laser, double force_map);
		void robot_parameters_feedback_adjustment( double feedback_sign );
		void set_companion_position(double r, double ro){r_=r; ro_=ro;}
		bool get_is_target_person_visible () { return is_target_person_visible_;}

		void set_target_person_id(int target_person){target_person_id_=target_person;} // made by (ely) para seleccionar un id de una persona concreta a seguir!
		void set_robot(Crobot* internal_robot){internal_robot_=internal_robot;}
		void internal_person_obj(Cperson_abstract* internal_person_obj){internal_person_obj_=internal_person_obj;}
		void set_get_force_map_robot_position(Sforce act_force_map_robot_position){act_force_map_robot_position_=act_force_map_robot_position;}

	protected:
		Sforce robot_force_persongoal(SpointV_cov robot, SpointV_cov person, std::vector<double> social_forces_param);
		int target_person_id_;
		std::vector<double> param_force_approach_;
		approach_state robot_state_;//state with respect to its approaching target
		void set_robot_state(SpointV_cov person, SpointV_cov robot, double lambda);
		SpointV_cov robot_tangential_propagation( Sforce f );

		double v_max_;//[m/s]
		double v_cruise_;//[m/s]
		double time_horizon_;//[s]
		double v_robot_desired_;//[m/s]
		double r_,ro_;//relative robot companion position in polar coordinates
		Sdestination person_destination_;
		bool is_target_person_visible_;
		Crobot* internal_robot_;  // for robot companion.
		Cperson_abstract* internal_person_obj_;
		Sforce act_force_map_robot_position_;

};


#endif /* FORCE_REACTIVE_ROBOT_COMPANION_H_ */
