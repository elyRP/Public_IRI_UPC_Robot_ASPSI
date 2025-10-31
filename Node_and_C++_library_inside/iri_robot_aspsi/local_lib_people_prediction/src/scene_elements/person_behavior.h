/*
 * person_behavior.h
 *
 *  Created on: Dec 17, 2013. Last Modified on 2025 by Ely
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

#ifndef PERSON_BEHAVIOR_H_
#define PERSON_BEHAVIOR_H_

#include "scene_elements/person_abstract.h"
#include "scene_elements/person_bhmip.h"
#include <list>


class Cperson_behavior : public Cperson_bhmip
{
  public:
	Cperson_behavior(unsigned int id=0,
			Cperson_abstract::target_type person_target_type=Cperson_abstract::Person,
			Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
			double _time_window = 1.0);
	virtual ~Cperson_behavior();
	virtual void reset();
	virtual void add_pointV( SpointV_cov point,
			Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering,  bool robot_or_person=false);
	virtual void Cperson_bhmip_add_pointV( SpointV_cov point ,Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering, bool person_or_robot=false);
	virtual void prediction( double min_v_to_predict );
	/**
	 * this virtual method calculates interaction forces depending on the
	 * expected behavior, that is, changing the force parameters for each
	 * different person
	 */
	virtual Sbehavior* find_behavior_estimation( unsigned int id );
	virtual Cperson_abstract::behavior_type	get_best_behavior_to_person( unsigned int interacting_person ) const;

	//methods for trajectory prediction
	virtual void prediction_propagation( double dt , Sforce force = Sforce(), unsigned int index = 0 );
	virtual void planning_propagation( double dt , Sforce force = Sforce(), unsigned int index = 0 );
	virtual void planning_propagation_copy( unsigned int prediction_index );
	virtual bool is_needed_to_propagate_person_for_planning( unsigned int parent_index, Spoint robot, unsigned int& new_index_to_be_copied );
	virtual void reset_propagation_flag() { has_copied_propagation_ = false; }
	virtual const std::vector<SpointV_cov>* get_planning_trajectory() const { return &planning_trajectory_;}//raw planning trajectory
	virtual const std::vector<SpointV_cov>* get_prediction_trajectory() const {  return &prediction_trajectory_;}//predicted trajectory, according to best plan
	virtual void clear_prediction_trajectory();
	virtual void clear_planning_trajectory();
	virtual void reserve_prediction_trajectory( unsigned int n );

  private:
	std::vector<SpointV_cov> planning_trajectory_, prediction_trajectory_;//container for predictions and planning
	bool has_copied_propagation_;
	std::list<Sbehavior> expected_behavior_list_;

	bool debug_prediction_propagation_; // debug for simulation person companion.
	bool debug_add_pointV_; // debug for simulation person companion.
	bool debug_Cperson_bhmip_add_pointV_; // debug for simulation person companion.



};


#endif /* PERSON_BEHAVIOR_H_ */
