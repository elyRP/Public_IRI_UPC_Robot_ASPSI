/*
 * plan_local_nav_person_companion.h
 *
 *  Created on: Jan 5, 2019  by Ely Repiso to separate the elements of the Scene_sim for the person companion, from the planner elements of the robot. Entirelly created by Ely to have an intelligent person in simulation.
 *
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

#ifndef PLAN_LOCAL_NAV_PERSON_COMPANION_H_
#define PLAN_LOCAL_NAV_PERSON_COMPANION_H_

#include "prediction_behavior.h"
#include "scene_elements/person_abstract.h"
#include "prediction_bhmip.h"
#include "nav/force_reactive_robot_companion.h"

//#include "companion_zanlungo/CompanionFrancescoModel.h" // library of Francesco's companion model

#include <fstream>      // std::ofstream
#include <random>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>

#include<bits/stdc++.h>
//using namespace std;

//#include "matlab.hpp"

using namespace std;
/**
 *	\brief Sedge_tree_pcomp Struct
 *
 *	data struct describing a tree edge containing the index to the parent vertex
 *	and the command inputs (v.w) or forces required to get from that vertex state
 */


class Sedge_tree_pcomp
{
  public:
	Sedge_tree_pcomp(	unsigned int parent, Sforce f_=Sforce(), Sforce f_goal_=Sforce(),
			Sforce f_people_=Sforce(),Sforce f_obs_=Sforce(), Sforce f_persongoal_=Sforce());
	unsigned int parent;
	Sforce f;
	Sforce f_goal;
	Sforce f_people;
	Sforce f_obs;
	Sforce f_persongoal;
	double cost(Sedge_tree_pcomp parent)const;
	void print();
};

/**
 *	\brief Smulticost_pcom Struct
 *
 *	data struct for containing multiciple costs and compare them in a fast and clean way
 *	specially to determine dominance relations between solutions.
 */
class Smulticost_pcom
{
  public:
	Smulticost_pcom( unsigned int id_ = 0, unsigned int n = 1 );
	std::vector<double> cost;
	unsigned int id;
	bool operator< ( const Smulticost_pcom& m2) const;
	bool operator==( const Smulticost_pcom& m2) const;
	bool operator!=( const Smulticost_pcom& m2) const;
	void print() const;
	void print_ml() const;
};

/**
 *	\brief Cplan_local_nal
 *
 * Basic local planner using social forces
 *
 */
class Cplan_local_nav_person_companion : public Cprediction_behavior
{
  public:
    enum plan_mode{  F_RRT_Uniform=0,  F_RRT_Gauss_Circle, F_RRT_GC_alpha};
    enum distance_mode{  Euclidean=0,  Cost2go_erf, Cost2go_norm , Cost2go_raw};
    enum global_mode{ Scalarization=0, Weighted_sum_erf, Weighted_sum_norm, MO_erf, MO_norm};
    enum action_mode{START=0,ITER,FACEPERSON,STOP};
    enum simulation_case{case0=0,case1,case2} ;
    enum calculate_complete_group_path_{akp_planner=0,person_prediction,prediction_total_time_with_collisions};



    // funciones a eliminar de person goal/target

    void change_set_id_person_goal(){  // compartida con simul_people (no la puedo eliminar)
        	id_person_goal_=change_id_person_goal_;
        	//this->Cprediction_behavior::set_Cprediction_behavior_id_target_person(id_person_goal_);
        	 Cperson_abstract* person_obj1;
        	bool finded_person=find_person(id_person_goal_ , &person_obj1);
        	if(finded_person){
        		person_obj1->set_person_type(Cperson_abstract::Person_companion);
        	}

        	//std::cout << " id_person_goal_="<<id_person_goal_<< std::endl;

        }
    void set_id_person_goal(unsigned int in_id_person_goal){
        	//id_person_goal_=in_id_person_goal;
        	change_id_person_goal_=in_id_person_goal;
         }
    void set_plan_local_nav_group_go_to_interact_with_other_person(bool in_group_go_to_interact_with_other_person){ // compartida con simul_people (no la puedo eliminar)
        	group_go_to_interact_with_other_person_=in_group_go_to_interact_with_other_person;
        }

    Spoint get_SIM_initial_person_goal_pose_actual(){  // compartida con simul_people (no la puedo eliminar)
            	//std::cout << " SIM_initial_person_goal_pose_actual_:"<< std::endl;
         	//SIM_initial_person_goal_pose_actual_.print();
         	return SIM_initial_person_goal_pose_actual_;
         }
   // enum vel_per_ok{ Vel_per=0, Near_goal, Far_goal};
    /**
     * \brief Initial algorithm parameters in the constructor of the
     * algorithm
     */
    Cplan_local_nav_person_companion(double horizon_time = 3.0, unsigned int max_iter = 1000,
    		plan_mode mode= Cplan_local_nav_person_companion::F_RRT_Uniform, bool robot_or_person=false);
    virtual ~Cplan_local_nav_person_companion(); // false == robot= id_0; true == person_companion =id_1


    /**
     * \brief this is the public interface to calculate the robot==person_companion plan
     * according to the observed scene and the algorithm parameters.
     */

    bool person_companion_plan_companion(Spose& pose_command, Cperson_abstract::companion_reactive& reactive,double dt=0.0,std::vector<Sdestination>* person_best_dest=NULL);

    double get_distance_to_goal() { return robot_->get_current_pointV().distance( goal_ );}
    /**
     * Configuration methods
     */
    void set_planning_mode( plan_mode mode ) {plan_mode_ = mode;}
    plan_mode get_planning_mode(){ return plan_mode_; }
    void set_distance_mode( distance_mode mode )
    	{distance_mode_ = mode; nondominated_plan_vertex_index_.clear(); nondominated_end_of_plan_vertex_index_.clear();}
    distance_mode get_distance_mode(){ return distance_mode_;}
    void set_global_mode( global_mode mode )
    	{global_mode_ = mode; nondominated_plan_vertex_index_.clear();nondominated_end_of_plan_vertex_index_.clear();}
    global_mode get_global_mode(){ return global_mode_;}
    void set_ppl_collision_mode( int ppl_col ) {ppl_collision_mode_ =  ppl_col; }
    void set_pr_force_mode( int pr_force_mode ) { pr_force_mode_ = pr_force_mode;}
    /**
     * \brief set_robot_goal() function sets the value of the goal position to
     * which the planning aims to. It is assumed that a global planner, above this
     * implementation, provides these goals
     */
    void set_robot_goal(const  Spoint& goal );
    void set_robot_goal_person_companion_akp( const Spoint& goal );
    void set_robot_goal_person_goal_global_plan( const Spoint& goal );
    void set_robot_goal_person_goal_global_plan_IN_robot_VERSION( const Spoint& goal );
    void set_robot_external_goal( const Spoint& goal ); // comapnion => obtain a external goal, not infered from the person comapnion
    void set_robot_external_goal_fix( const Spoint& goal );

    Spoint get_robot_goal() { return goal_;}
    Spoint get_external_robot_goal(){ return Spoint(extern_robot_goal_.x,extern_robot_goal_.y,extern_robot_goal_.time_stamp);}
    Spoint get_robot_local_goal() { return local_goal_;}
    void set_plan_cost_parameters( double c_dist, double c_orientation, double c_w_robot,
    		double c_w_people, double c_time, double c_w_obstacles, double c_old_path, double c_l_minima);
    const std::vector<double>* get_plan_cost_parameters() const { return &cost_parameters_;}
    void set_number_of_vertex( unsigned int n );
    unsigned int get_number_of_vertex(  ) { return max_iter_;}
    void set_robot_params( double v, double w, double av, double av_break,double aw, double platform_radii);
    void set_xy_2_goal_tolerance( double tol) { xy_2_goal_tolerance_ = tol; }
    void set_v_goal_tolerance( double tol) { v_goal_tolerance_ = tol; }
    void set_distance_to_stop( double d ){ distance_to_stop_ = d; }
    double get_distance_to_stop( ) { return distance_to_stop_;}

    /**
     * Return info methods
     */
    const std::vector<unsigned int>* get_robot_plan_index() const {
    	//std::cout <<" best_plan_vertex_index_.size()="<<best_plan_vertex_index_.size()<< std::endl;
    	return &best_plan_vertex_index_;}
    const std::vector<unsigned int>* get_robot_nondominated_plan_index() const {
    	//std::cout <<" nondominated_plan_vertex_index_.size()="<<nondominated_plan_vertex_index_.size()<< std::endl;
    	return &nondominated_plan_vertex_index_;
    }
    const std::vector<unsigned int>* get_robot_nondominated_end_of_plan_index() const {
    	//std::cout <<" nondominated_end_of_plan_vertex_index_.size()="<<nondominated_end_of_plan_vertex_index_.size()<< std::endl;
    	return &nondominated_end_of_plan_vertex_index_;
    }
    const std::vector<Sedge_tree_pcomp>* get_plan_edges() const { return &edge_;}
    const std::vector<Spoint>* get_random_goals() const { return &random_goals_;}
    double get_workspace_radii() { return workspace_radii_;}

    Spose get_best_planned_pose_person_companion_akp(double dt=0.0);
    void get_navigation_instant_work( double& work_robot, double& work_persons );
    void get_navigation_cost_values(  std::vector<double>& costs )//returns the vector of bests costs, calculated in the global_cost routine
    	{ costs = best_costs_;}
    void get_navigation_mean_cost_values(  std::vector<double>& costs ) { costs = mean_costs_;}
    void get_navigation_std_cost_values(  std::vector<double>& costs ) { costs = std_costs_;}
	/**
	 * \brief calculate scene cost
	 *
	 * This function calculates the costs due to robot navigation and
	 * nearby moving people. It is used only as an observation of the scene,
	 * no calculations are done, is just for evaluating purposes.
	 */
    void calculate_navigation_cost_values( std::vector<double>& costs );
    // people companion ely functions
    double get_actual_companion_angle() { return min_next_companion_angle_; }
    double get_actual_companion_cost() { return min_next_companion_cost_; }
    SpointV_cov get_companion_person_position() {return companion_person_position_;}
    Sdestination get_person_destination() {return person_companion_goal_;}

    //*** companion dynamic reconfigure (ini) ***//
    unsigned int get_id_person_companion(){
       	//return id person companion
       	return id_person_companion_;
    }

    void set_id_person_companion(unsigned int in_id_person_companion){
    	//id_person_companion_=in_id_person_companion;
    	change_id_person_companion_=in_id_person_companion;
    }

    void set_id_person_companion_people_simulator(unsigned int in_id_person_companion){
    	change_id_person_companion_=in_id_person_companion;
    	change_set_id_person_companion();
    }

    void change_set_id_person_companion(){
    	id_person_companion_=change_id_person_companion_;
    	this->Cprediction_behavior::set_Cprediction_behavior_id_person_companion(change_id_person_companion_);
        this->Cprediction_bhmip::set_id_person_companion_Cprediction_bhmip(change_id_person_companion_);
        Cperson_abstract* person_obj1;
        bool finded_person=find_person(id_person_companion_ , &person_obj1);
        if(finded_person){
        	person_obj1->set_person_type(Cperson_abstract::Person_companion);
        }
       // std::cout << " (SET SET SET SET SET) => id_person_companion_="<<id_person_companion_<< std::endl;
    }


    /*void set_id_SECOND_person_companion_people_simulator(unsigned int in_id_second_person_companion){

    	id_SECOND_person_companion_=in_id_second_person_companion;
    	this->Cprediction_behavior::set_Cprediction_behavior_id_second_person_companion(in_id_second_person_companion);
    	this->Cprediction_bhmip::set_id_second_person_companion_Cprediction_bhmip(in_id_second_person_companion);

    	 Cperson_abstract* person_obj1;
    	bool finded_person=find_person(id_SECOND_person_companion_ , &person_obj1);
    	        if(finded_person){
    	        	person_obj1->set_person_type(Cperson_abstract::Person_companion);
    	        }
    }*/


    ////////////////
    unsigned int my_id_person_companion_simulation_; // id, actual person companion. the id of it selft (todo: ver si me hace falta verdaderamente)
    void set_my_id_person_companion_people_simulator(unsigned int in_my_id_person_companion){

        	my_id_person_companion_simulation_=in_my_id_person_companion;
        	this->Cprediction_behavior::set_Cprediction_behavior_my_id_second_person_companion_sim(in_my_id_person_companion);
        	this->Cprediction_bhmip::set_my_id_second_person_companion_sim_Cprediction_bhmip(in_my_id_person_companion);

        	// creo que no hace falta, ya que los ponemos en los otros id's.
        	// Cperson_abstract* person_obj1;
        	//bool finded_person=find_person(id_SECOND_person_companion_ , &person_obj1);
        	       // if(finded_person){
        	       // 	person_obj1->set_person_type(Cperson_abstract::Person_companion);
        	       // }
        }


    /////////



    void set_robot_person_proximity_distance(double in_proximity_distance_between_robot_and_person){
    	robot_person_proximity_distance_=in_proximity_distance_between_robot_and_person;
    }
    void set_proximity_distance_tolerance(double in_proximity_distance_tolerance){
    	robot_person_proximity_tolerance_=in_proximity_distance_tolerance;
    }
    void set_additional_distance_companion_sphere(double in_add_dist_companion_sphere){
    	robot_adition_complete_esphere_companion_distance_=in_add_dist_companion_sphere;
    	std::cout << "robot_adition_complete_esphere_companion_distance_="<<robot_adition_complete_esphere_companion_distance_<< std::endl;
    }
    void set_proximity_goals_robot_and_person(double in_proximity_goals_robot_and_person_x,double in_proximity_goals_robot_and_person_y){
    	robot_person_proximity_goals_x_=in_proximity_goals_robot_and_person_x;
    	robot_person_proximity_goals_y_=in_proximity_goals_robot_and_person_y;
    }
    void set_offset_attractive(double in_offset_attractive_state){
    	offset_atractive_=in_offset_attractive_state;
    }
    void set_force_obs_max(double in_force_obs_max_x,double in_force_obs_max_y){
    	f_obst_max_x_=in_force_obs_max_x;
    	f_obst_max_y_=in_force_obs_max_y;
    }
    void set_real_companion_angle(double in_real_companion_angle){
    	angle_companion_=in_real_companion_angle;
    	before_initial_angle_=in_real_companion_angle;
    	min_next_companion_angle_=in_real_companion_angle;
    }

    void set_person_goal_percentage(double in_person_goal_percentage){
    	person_goal_percent_=in_person_goal_percentage;
    }

    double get_robot_person_proximity_distance(){
    	return robot_person_proximity_distance_;
     }

    Sdestination get_person_companion_goal_out(){
        return person_companion_goal_out_;
    }

    void set_companion_same_person_goal(bool act_companion_same_person_goal){

    	companion_same_person_goal_=act_companion_same_person_goal;
    	std::cout << "companion_same_person_goal_="<<companion_same_person_goal_<< std::endl;
    }

    void set_overpas_obstacles_behind_person(bool in_overpas_obstacles_behind_person){
    	overpas_obstacles_behind_person_=in_overpas_obstacles_behind_person;
    }

    void set_anisotropy_threshold(double in_anisotropy_threshold){
    	anisotropy_threshold_=in_anisotropy_threshold;
    }

    void set_max_d_to_detect_laser_obs(double in_max_d_to_detect_laser_obs){
    	Cprediction_behaviour_set_max_d_to_detect_laser_obs(in_max_d_to_detect_laser_obs);
    	max_distance_to_obstacles_detected_=in_max_d_to_detect_laser_obs;
    	std::cout << "set_max_d_to_detect_laser_obs-> in_max_d_to_detect_laser_obs="<<in_max_d_to_detect_laser_obs<< std::endl;
    }

    void set_save_results_in_file(bool in_save_results_in_file){
    	save_results_in_file_=in_save_results_in_file;
       }

    void set_meters_to_goal_to_save_results_in_file(double in_meters_goal_to_save_results_in_file){
    	meters_goal_to_save_results_in_file_=in_meters_goal_to_save_results_in_file;
    }

    void set_mode_velocity(bool in_mode_velocity){
    	mode_velocity_=in_mode_velocity;
    }

    void set_results_filename(std::string in_results_file){
    	results_file_=in_results_file;
    	std::cout << " (changed) results_file_="<<results_file_<< std::endl;
    }

    void set_reduce_max_vel_dist(double in_reduce_max_vel_dist){
    	reduce_max_vel_dist_=in_reduce_max_vel_dist;
    }

    void set_marge_in_distance(double in_marge_in_distance){
    	marge_in_distance_=in_marge_in_distance;
    }

    void set_marge_angle_companion(double in_marge_angle_companion){
    	marge_angle_companion_=in_marge_angle_companion;
    }

    void set_debug_filename(std::string in_debug_file){
    	debug_file_=in_debug_file;
    }

    void set_mode_debug_cout(bool in_debug_robot_cout_robot){
    	debug_cout_robot_=in_debug_robot_cout_robot;
    }

    void set_mode_debug_file(bool in_debug_file_robot){
    	debug_file_robot_=in_debug_file_robot;
    }



    void set_externa_force_k_near_goal_akp(double in_k_near){
    	externa_force_k_near_goal_akp_=in_k_near;
    	std::cout << " (changed) externa_force_k_near_goal_akp_="<<externa_force_k_near_goal_akp_<< std::endl;
    }

    void set_ex_max_dist_to_near_goal_force_akp(double in_max_dist_to_near_goal_force){
    	max_dist_to_near_goal_force_akp_=in_max_dist_to_near_goal_force;
    	std::cout << " (changed) max_dist_to_near_goal_force_akp_="<<max_dist_to_near_goal_force_akp_<< std::endl;
    }

    void set_externa_force_k_far_goal_akp(double in_externa_force_k_far_goal_akp_){
    	externa_force_k_far_goal_akp_=in_externa_force_k_far_goal_akp_;
    	std::cout << " (changed) externa_force_k_far_goal_akp_="<<externa_force_k_far_goal_akp_<< std::endl;
    }


    void set_mode_step_near(bool in_mode_step_near){
    	mode_step_near_=in_mode_step_near;
    	std::cout << " (changed) mode_step_near_="<<mode_step_near_<< std::endl;
   }

    /*void set_out_index_step(int in_out_index_step){
    	out_index_step_=in_out_index_step;
    	std::cout << " (changed) out_index_step_="<<out_index_step_<< std::endl;
   }*/

    void set_out_index_step_companion_goal(unsigned int in_out_index_step_companion_goal){
        	out_index_step_companion_goal_=in_out_index_step_companion_goal;
        	std::cout << " (changed) out_index_step_companion_goal_="<<out_index_step_companion_goal_<< std::endl;
       }

    void set_out_index_step_final_dest_goal(unsigned int in_out_index_step_final_dest_goal){
        	out_index_step_final_dest_goal_=in_out_index_step_final_dest_goal;
        	std::cout << " (changed) out_index_step_final_dest_goal_="<<out_index_step_final_dest_goal_<< std::endl;
       }

    void only_comp_people_vel_and_robot_poses(int in_only_comp_people_vel_and_robot_poses){
    	only_comp_people_vel_and_robot_poses_=in_only_comp_people_vel_and_robot_poses;
    }
    //this->planner_.set_ex_max_dist_to_near_goal_force_akp(config.ex_max_dist_to_near_goal_force);
    //this->planner_.set_max_d_to_detect_laser_obs(config.detection_laser_obstacle_distances);
    //*** companion dynamic reconfigure (fin) ***//

    void set_person_radi_amp(double in_person_radi_amp){
    	person_radi_amp_=in_person_radi_amp;
    	std::cout << " IMPORTANT!!!!!!!!!!! (changed) person_radi_amp_="<<person_radi_amp_<< std::endl;
    	person_radi_=in_person_radi_amp +0.2;
    	person_radi2_=in_person_radi_amp-0.1;
    	person_radi_per_comp_=in_person_radi_amp;
       }
    void set_obstacle_radi_amp(double in_obstacle_radi_amp){
    	obstacle_radi_amp_=in_obstacle_radi_amp;
    	obstacle_radi_=in_obstacle_radi_amp;
    	obstacle_radi2_=in_obstacle_radi_amp;
    	std::cout << " IMPORTANT!!!!!!!!!!! (changed) obstacle_radi_amp_="<<obstacle_radi_amp_<< std::endl;
    }
    void set_ini_vel_to_increment_angle(double in_ini_vel_to_increment_angle){
    	ini_vel_to_increment_angle_=in_ini_vel_to_increment_angle;
    	std::cout << " (changed) ini_vel_to_increment_angle_="<<ini_vel_to_increment_angle_<< std::endl;
    }

    double get_horizon_time(){return horizon_time_;}

    void set_person_list(std::list<Cperson_abstract *> in_person_list_){
    	person_list_=in_person_list_;
    }

    std::list<Cperson_abstract *> get_person_list(){
     	return person_list_;
     }
    void set_person_companion_desired_velocity_sim(double in_person_companion_desired_velocity_sim){
    	person_companion_desired_velocity_sim_=in_person_companion_desired_velocity_sim;
    }

    void set_alpha_companion(double in_alpha_companion){
    	alpha_companion_=in_alpha_companion;
    }

    void set_beta_companion(double in_beta_companion){
    	beta_companion_=in_beta_companion;
    }

    double get_alpha_companion(){
    	return alpha_companion_;
    }

    double get_beta_companion(){
    	return beta_companion_;
    }

    void set_evaluate_costs_filename(std::string in_evaluate_costs_file){
    	evaluate_costs_file_=in_evaluate_costs_file;
        std::cout << " (changed) evaluate_costs_file_="<<evaluate_costs_file_<< std::endl;
    }


	void set_evaluate_change_distance_and_angle_companion_filename(std::string in_evaluate_change_distance_and_angle_companion_file){
    	evaluate_change_distance_and_angle_companion_file_=in_evaluate_change_distance_and_angle_companion_file;
	    std::cout << " (changed) evaluate_change_distance_and_angle_companion_file__file_="<<evaluate_change_distance_and_angle_companion_file_<< std::endl;
	}

    Sdestination get_person_companion_goal(){
        //std::cout << " person_companion_goal_.x="<<person_companion_goal_.x<<"; person_companion_goal_.y="<<person_companion_goal_.y<< std::endl;
        return robot_goal_to_person_companion_;
     }
    Sdestination get_robot_path_goal(){
        //std::cout << " robot_path_goal_.x="<<robot_path_goal_.x<<"; robot_path_goal_.y="<<robot_path_goal_.y<< std::endl;
        return robot_path_goal_;
     }
    Spose get_final_combined_goal(){
        //std::cout << " final_combined_goal_.x="<<final_combined_goal_.x<<"; final_combined_goal_.y="<<final_combined_goal_.y<< std::endl;
        return final_combined_goal_;
     }



    action_mode get_state_Action(){
    	//std::cout << " final_combined_goal_.x="<<final_combined_goal_.x<<"; final_combined_goal_.y="<<final_combined_goal_.y<< std::endl;
    	return Action_;
    }

    void set_state_Action(action_mode Action_in){
    	//std::cout << " final_combined_goal_.x="<<final_combined_goal_.x<<"; final_combined_goal_.y="<<final_combined_goal_.y<< std::endl;
    	Action_=Action_in;
    }

    Spoint get_SIM_initial_person_companion_pose1(){
       	//std::cout << " SIM_initial_person_companion_pose1_:"<< std::endl;
    	//SIM_initial_person_companion_pose1_.print();
    	return SIM_initial_person_companion_pose1_;
     }
    Spoint get_SIM_initial_person_companion_pose2(){
       	//std::cout << " SIM_initial_person_companion_pose1_:"<< std::endl;
    	//SIM_initial_person_companion_pose1_.print();
    	return SIM_initial_person_companion_pose2_;
     }

    simulation_case get_actual_case(){
    	//std::cout << " SIM_initial_person_companion_pose1_:"<< std::endl;
    	//SIM_initial_person_companion_pose1_.print();
    	return actual_case_;
    }

    void set_actual_case(simulation_case actual_case_in){
    	//std::cout << " SIM_initial_person_companion_pose1_:"<< std::endl;
    	//SIM_initial_person_companion_pose1_.print();
    	actual_case_=actual_case_in;
    }

    void set_ros_time_to_sec(double ros_time_to_sec_in){
    	ros_time_to_sec_=ros_time_to_sec_in;
    }

       void set_is_act_person_companion_bool(bool is_act_person_companion_in){
    	   is_act_person_companion_=is_act_person_companion_in;
       }



       void set_restart_real(bool in_restart_real){
    	   restart_real_=in_restart_real;
       }


       void set_max_dist_to_go_behind(double in_max_dist_to_go_behind){
    	   max_dist_to_go_behind_=in_max_dist_to_go_behind;
       }



       void set_restart_real_data_txt(){
    	   new_matlab_file();
    	  // new_Zanlungo_file();
    	   iteration_=1;
    	   experiment_=1;
       }

       void set_change_sim(bool change_sim){
    	   sim=change_sim;
    	   this->Cprediction_behavior::set_sim_prediction_behaviour(sim);
    	   this->Cscene_abstract::set_sim(sim);
       }

       void set_change_sim_target_per(bool change_sim_target_per){
    	   sim_target_per=change_sim_target_per;
         }

       void set_change_external_to_stop_case(bool in_change_external_to_stop_case){
    	   if(in_change_external_to_stop_case){
    		   Action_=Cplan_local_nav_person_companion::STOP;
    	   }
       }

       void set_case_stop_giro(bool in_case_stop_giro){
    	   case_stop_giro_=in_case_stop_giro;
         }

       void set_incremento_giro_positivo(double in_incremento_giro_positivo){
    	   incremento_giro_positivo_=in_incremento_giro_positivo;
         }
       void set_incremento_giro_negativo(double in_incremento_giro_negativo){
    	   incremento_giro_negativo_=in_incremento_giro_negativo;
         }


       Sforce get_force_int_between_person_comp_and_robot(){
       	return force_int_between_person_comp_and_robot_;
       }

       // set values of performance approaching

       void set_value_distance_global_path(double in_value_distance_global_path){
    	   value_distance_global_path_=in_value_distance_global_path;
    	  // std::cout << " (planner in) value_distance_global_path_="<<value_distance_global_path_<< std::endl;
       }

       void set_global_path_ini_orientation(double in_global_path_ini_orientation){
    	   global_path_ini_orientation_=in_global_path_ini_orientation;
           //std::cout << " (planner in) global_path_ini_orientation_="<<global_path_ini_orientation_<< std::endl;
        }
       void set_global_path_final_orientation(double in_global_path_final_orientation){
    	   global_path_final_orientation_=in_global_path_final_orientation;
           //std::cout << " (planner in) global_path_final_orientation_="<<global_path_final_orientation_<< std::endl;
        }


       SpointV_cov get_robot_pose(){
    	   return robot_->get_current_pointV();
       }

       void set_robot_pose_for_person_companion_simulation(Spose in_robot_pose){
    	   robot_->set_current_pose(in_robot_pose);
    	   //std::cout << " (planner in, set_robot_pose_for_person_companion_simulation) in_robot_pose.x="<<in_robot_pose.x<<"; in_robot_pose.y="<<in_robot_pose.y<< std::endl;
    	   //std::cout << " (planner in, set_robot_pose_for_person_companion_simulation) robot_.get.x="<<robot_->get_current_pose().x<<"; robot_.get.y="<<robot_->get_current_pose().y<< std::endl;
       }


/// FUNCIONES debidas a optimizacion del codigo solo validas para person_companion (simulacion)

      /* void set_actual_person_Companion_SpointV(SpointV_cov in_actual_person_Companion_SpointV){
    	   actual_person_Companion_SpointV_=in_actual_person_Companion_SpointV;
       }*/

       /*void set_actual_person_Companion_destination(Sdestination in_actual_person_Companion_destination){
    	   actual_person_Companion_destination_=in_actual_person_Companion_destination;
       }*/

      /* void set_max_desired_person_Companion_velocity(double in_max_desired_person_Companion_velocity){
    	   max_desired_person_Companion_velocity_=in_max_desired_person_Companion_velocity;
       }*/

      /* void set_actual_person_Companion_pointer(Cperson_abstract *in_actual_person_Companion_pointer){
    	   actual_person_Companion_pointer_=in_actual_person_Companion_pointer;
       }*/

   	void set_final_goal_reached_in_node(bool in_final_goal_reached_in_node){
   		final_goal_reached_in_node_=in_final_goal_reached_in_node;
   	}


   	Spoint get_medium_point(){
   		return medium_point_;
   	}

   	void set_change_goal_of_the_error(unsigned int in_change_goal_of_the_error){
   		change_goal_of_the_error_=in_change_goal_of_the_error;
   	 std::cout <<"set IN change_goal_of_the_error_="<<change_goal_of_the_error_<< std::endl;
   	}

    void new_matlab_file();

    void do_scene_prediction_from_outside(std::vector<Sdestination>* person_best_dest){
    	bool person_or_robot=true; //case, person_companion
    	//std::cout << " (5) !!! person_companion_plan_companion" << std::endl;
    	person_companion_->get_best_dest().print();
    	//std::cout << " (5.1) !!! person_or_robot="<<person_or_robot<<"; person_list_.size()="<< person_list_.size() << std::endl;

    	this->Cprediction_behavior::scene_prediction(person_or_robot,person_best_dest); // scene prediction, needed, pero sin la person cmpanion y hará falta el robot en colisiones.
    	//std::cout << " (5.2) !!! person_companion_plan_companion" << std::endl;

    }


    /*void set_second_group_companion_personSpoint(SpointV_cov in_second_group_companion_personSpoint){
    	second_group_companion_personSpoint_=in_second_group_companion_personSpoint;
    }*/

    /*void second_group_companion_personSpose(Spose in_second_group_companion_personSpose){
    	second_group_companion_personSpose_=in_second_group_companion_personSpose;
    }*/

    /*void set_second_group_companion_personDest(std::vector<Sdestination> Dest_other_Tracks){

    	for(unsigned int p=0;p<Dest_other_Tracks.size();p++){
    		if(Dest_other_Tracks[p].id==id_SECOND_person_companion_){
    			second_group_companion_personDest_=Sdestination(0,Dest_other_Tracks[p].x,Dest_other_Tracks[p].y);
    			//std::cout << " print DEST person, id="<<Dest_other_Tracks[p].id << std::endl;
    			//Dest_other_Tracks[p].print();
    		}
    	}

    }*/

    /*void set_we_have_second_person_companion(bool in_we_have_second_person_companion){
    	we_have_second_person_companion_=in_we_have_second_person_companion;
    }*/


    Sdestination akp_get_goal_path(){ // for visualization del simulador personas.
    	return akp_out_path_goal_;
    }

    Sdestination akp_get_goal_companion(){// for visualization del simulador personas.
    	return akp_out_companion_goal_;
    }

    void set_companion_angle_peopl_in_group(double in_companion_angle_peopl_in_group){ // angle in degrees.
    	companion_angle_peopl_in_group_=in_companion_angle_peopl_in_group;
    }

    void set_real_distance_between_people_of_group_(double in_real_distance_between_people_of_group){ // angle in degrees.
    	real_distance_between_people_of_group_=in_real_distance_between_people_of_group;
    }

    /*void set_second_group_companion_object_person(Cperson_abstract* in_second_group_companion_person_obj){
    	second_group_companion_person_obj_=in_second_group_companion_person_obj;
    }*/

    Spoint get_before_next_goal_of_robot(){
    	return before_next_goal_of_robot_;
    }

    // set functions of Francesco companion model
    //void set_genome_params(Cgenome in_genome){
    	//genome_params_=in_genome;
    	//Companion_Zanlungo_Model_.set_genome_params_in_CompanionFrancescoModel(in_genome);
   // }

    /*void set_ZanlungoCompModel_bool(bool in_bool_Zanlungo_CompMod){
    	Zanlungo_model_=in_bool_Zanlungo_CompMod;
    }*/

	void set_high_vel_dampening_parameter_planner(double in_dampening_parameter){
		robot_->set_high_vel_dampening_parameter(in_dampening_parameter);
	}

	void set_normal_vel_dampening_parameter_planner(double in_dampening_parameter){
		robot_->set_normal_vel_dampening_parameter(in_dampening_parameter);
	}
	void set_slow_vel_dampening_parameter_planner(double in_dampening_parameter){
		robot_->set_slow_vel_dampening_parameter(in_dampening_parameter);
	}

	void set_limit_linear_vel_for_dampening_parameter_planner(double in_limit_linear_vel){
		robot_->set_limit_linear_vel_for_dampening_parameter(in_limit_linear_vel);
	}
	void set_limit_angular_vel_for_dampening_parameter_planner(double in_limit_angular_vel){
		robot_->set_limit_angular_vel_for_dampening_parameter(in_limit_angular_vel);
	}

	void set_number_of_group_people(unsigned int in_number_of_group_people){
		number_of_group_people_=in_number_of_group_people;
	}

	/*void set_constante_multiplicar_fuerza_base_Zanlungo(double in_constante_multiplicar_fuerza_base_Zanlungo){
		constante_multiplicar_fuerza_base_Zanlungo_=in_constante_multiplicar_fuerza_base_Zanlungo;
	}*/

	void set_conf_constante_multiplicar_fuerza_goal(double in_constante_multiplicar_fuerza_goal){
		constante_multiplicar_fuerza_goal_=in_constante_multiplicar_fuerza_goal;
	}

	/*void set_up_distance_margin_Zamlungo(double in_up_distance_margin_Zamlungo){
		up_distance_margin_Zamlungo_=in_up_distance_margin_Zamlungo;
	}*/

	/*void set_down_distance_margin_Zamlungo(double in_down_distance_margin_Zamlungo){
		down_distance_margin_Zamlungo_=in_down_distance_margin_Zamlungo;
	}*/

	/*void set_bool_robot_propagation_Zanlungo(bool in_bool_robot_propagation_Zanlungo){
		bool_robot_propagation_Zanlungo_=in_bool_robot_propagation_Zanlungo;
	}*/

	void set_bool_distance_margin(bool in_bool_distance_margin){
		bool_distance_margin_=in_bool_distance_margin;
	}

	/*void set_dis_tol_Francesco_force(double in_dis_tol){
		dis_tol_=in_dis_tol;
	}*/

	bool get_change_ids_group_people_in_recongigure(){
		return change_ids_group_people_in_recongigure_;
	}

	void set_change_ids_group_people_in_recongigure(bool in_change_ids_group_people_in_recongigure){
		change_ids_group_people_in_recongigure_=in_change_ids_group_people_in_recongigure;
	}

	/*void set_debug_output_screen_mesages_zanlungo(bool in_debug_output_screen_mesages_zanlungo){
		debug_output_screen_mesages_=in_debug_output_screen_mesages_zanlungo;
	}*/
	/*void set_print_to_zanlungo_file(bool in_print_to_zanlungo_file){
		print_to_zanlungo_file_=in_print_to_zanlungo_file;
	}*/

	void set_new_time_window_for_filter_velocity(double in_new_time_window){
		this->Cprediction_behavior::change_time_window_for_filter_velocity(in_new_time_window);
	}

	void set_threshold_vel_pers_to_stop(double in_threshold_vel_pers_to_stop){
		threshold_vel_pers_to_stop_=in_threshold_vel_pers_to_stop;
	}
	void set_dist_bet_rob_ext_goal_to_stop(double in_dist_bet_rob_ext_goal_to_stop){
		dist_bet_rob_ext_goal_to_stop_=in_dist_bet_rob_ext_goal_to_stop;
	}

	void set_dist_bet_pers1Com_ext_goal_to_stop(double in_dist_bet_pers1Com_ext_goal_to_stop){
		dist_bet_pers1Com_ext_goal_to_stop_=in_dist_bet_pers1Com_ext_goal_to_stop;
	}
	void set_dist_bet_pers2Com_ext_goal_to_stop(double in_dist_bet_pers2Com_ext_goal_to_stop){
		dist_bet_pers2Com_ext_goal_to_stop_=in_dist_bet_pers2Com_ext_goal_to_stop;
	}

	void set_change_first(bool in_first){
		first_=in_first;
	}

	void set_treshold_distance_between_steps(double in_treshold_distance_between_steps){
		treshold_distance_between_steps_=in_treshold_distance_between_steps;
	}

	void set_threshold_max_total_force(double in_threshold_max_total_force){
		threshold_max_total_force_=in_threshold_max_total_force;
	}


	void set_dis_tol(double in_dis_tol){
		dis_tol_=in_dis_tol;
	}

	void set_dis_tol_side(double in_dis_tol_side){
			dis_tol_side_=in_dis_tol_side;
	}

	/*void set_data_file_Zanlungo(std::string in_data_file_Zanlungo){
		data_file_Zanlungo_=in_data_file_Zanlungo;
	}*/

	/*void set_AKP_max_desired_ve_robot_calc_with_Zanlungo(double in_max_vel){
		Companion_Zanlungo_Model_.set_max_desired_ve_robot(in_max_vel);
	}*/

	/*void set_calculated_desired_vel(bool use_zanlungo_desired_vel){
		calculated_desired_vel_=use_zanlungo_desired_vel;
	}*/

	void set_desired_vel_seteada_desde_fuera(double in_desired_vel_seteada_desde_fuera){
		desired_vel_seteada_desde_fuera_=in_desired_vel_seteada_desde_fuera;
	}

	void set_gamma_companion(double in_gamma_companion){
		gamma_companion_=in_gamma_companion;
	}

	void set_delta_companion(double in_delta_companion){
		delta_companion_=in_delta_companion;
	}

	void set_k_final_goal(double in_set_k_final_goal){

		 std::vector<double> params_robot=new_get_sfm_params(robot_);
		 params_robot.at(0)=in_set_k_final_goal;
		 if(change_k_robot_){
			 set_sfm_to_robot( params_robot );
		 }
		 if(change_k_person_){
			 set_sfm_to_person(params_robot);
		 }
		 if(change_k_person_comp_){
			 set_sfm_to_person_companion( params_robot );
		 }

		 if(change_k_obst_){
			 set_sfm_to_obstacle( params_robot );
		 }

		 //Companion_Zanlungo_Model_.set_internal_k_value(in_set_k_final_goal);
	}


	void set_change_k_robot(bool in_change_k_robot){
		change_k_robot_=in_change_k_robot;
	}

	void set_change_k_person(bool in_change_k_person){
		change_k_person_=in_change_k_person;
	}

	void set_change_k_person_comp(bool in_change_k_person_comp){
		change_k_person_comp_=in_change_k_person_comp;
	}

	void set_change_k_obst(bool in_change_k_obst){
		change_k_obst_=in_change_k_obst;
	}

	/*void set_AKP_C_vp_desired_Zanlungo(double in_AKP_C_vp_desired_Zanlungo)
	{
		Companion_Zanlungo_Model_.set_C_vp_desired_Zanlungo(in_AKP_C_vp_desired_Zanlungo);
	}
	void set_AKP_C1(double in_AKP_C1){
		Companion_Zanlungo_Model_.set_C1(in_AKP_C1);
	}
	void set_AKP_dist_r0_Zanlungo(double in_AKP_dist_r0_Zanlungo){
		Companion_Zanlungo_Model_.set_dist_r0_Zanlungo(in_AKP_dist_r0_Zanlungo);
	}
	void set_C_acc1(double in_AKP_C_acc1){
		Companion_Zanlungo_Model_.set_C_acc1(in_AKP_C_acc1);
	}*/
	/*void set_C_acc2(double in_AKP_C_acc2){
		Companion_Zanlungo_Model_.set_C_acc2(in_AKP_C_acc2);
	}*/

	/*void set_AKP_bool_show_constants_desired_vel(bool in_AKP_bool_show_constants_desired_vel){
		Companion_Zanlungo_Model_.set_bool_show_constants_desired_vel(in_AKP_bool_show_constants_desired_vel);
	}*/

	void set_minimun_velocity_case_stop_initial(double in_minimun_velocity_case_stop_initial){
		minimun_velocity_case_stop_initial_=in_minimun_velocity_case_stop_initial;
	}

	void set_final_max_v(double in_final_max_v){
		final_max_v_=in_final_max_v;
	}


	void set_initial_limit_distance_goal_to_person_robot_stop(double in_initial_limit_distance_goal_to_person_robot_stop){
		initial_limit_distance_goal_to_person_robot_stop_=in_initial_limit_distance_goal_to_person_robot_stop;
	}

	void set_ideal_max_robot_velocity(double in_ideal_max_robot_velocity){
		ideal_max_robot_velocity_=in_ideal_max_robot_velocity;
	}

	double calc_person_companion_orientation_from_outside(SpointV actual_p1_point, Sdestination person_dest, Spose in_robot);

	void set_bool_change_ct_and_cr_with_vel(bool in_bool_change_ct_and_cr_with_vel){
		bool_change_ct_and_cr_with_vel_=in_bool_change_ct_and_cr_with_vel;
	}
	bool bool_change_ct_and_cr_with_vel_;

	void set_bool_new_velocity_system( bool in_bool_new_velocity_system){
		bool_new_velocity_system_=in_bool_new_velocity_system;
	}

	bool bool_new_velocity_system_;

	// INI NEW FACKE CHANGE AND DESAPEAR ID:
	// MEjor poner con booleano... ya esta con booleano.
	void set_No_change_ids(bool in_No_change_ids){
		No_change_ids_=in_No_change_ids;
	}
	bool No_change_ids_;

	/*void erase_p1_change_history_p1_to_p2_in_person_list(unsigned int person1_id,unsigned int person2_id){
		// frist interchange history of people. Test first without change history
		// remove_people
		this->Cprediction_bhmip::remove_person_from_outside(person1_id);
	}*/
	// FIN NEW FACKE CHANGE AND DESAPEAR ID: (OJO! además quitar el intercambio de Id's de aquí dentro! molesta más que ayuda! Creo...

	void set_current_pose_robot_akp(Spose in_current_pose_robot){
		//std::cout<<" 1 IIIIIINNNNN set_current_pose_robot_akp(  ) "<< std::endl;
		  robot_->set_current_pose(in_current_pose_robot);
		  robot_->set_current_pointV(SpointV(in_current_pose_robot.x,in_current_pose_robot.y,in_current_pose_robot.time_stamp,in_current_pose_robot.v*cos(in_current_pose_robot.theta),in_current_pose_robot.v*sin(in_current_pose_robot.theta)));
		  //std::cout<<" 2 IIIIIINNNNN set_current_pose_robot_akp(  ) "<< std::endl;
		  robot_->get_current_pose().print();
		 // std::cout<<" 1 OUUUUUTtt set_current_pose_robot_akp(  ) "<< std::endl;
	}

	/*void plan_set_augment_initial_v(bool in_augment_initial_v){
		robot_->set_augment_initial_v(in_augment_initial_v);

	}*/

	void plan_set_initial_v_robot_needed(double in_initial_v_robot_needed){
		robot_->set_initial_v_robot_needed(in_initial_v_robot_needed);
	}

	/*void set_Zanlungo_model2(bool in_Zanlungo_model2){
		Zanlungo_model2_=in_Zanlungo_model2;
	}*/

	Crobot* get_return_person_companion(){ //retorna el person_companion_ para el simulador, para hacer el restart directamente.
		return person_companion_;
	}

	/*void set_plan_create_fake_fixed_second_person_companion(bool in_plan_create_fake_fixed_second_person_companion){
		plan_create_fake_fixed_second_person_companion_=in_plan_create_fake_fixed_second_person_companion;
	}*/

	void set_actual_case_sim_goal(bool in_actual_case_sim_goal){ // goal movil, dependiendo orientación grupo, tangente a perpendicular al goal, p
		//ara orientar el goal final en la direccion del robot y que no se cruce delante de las personas al estar muy cerca del goal final.
		actual_case_sim_goal_=in_actual_case_sim_goal;
	}

	Spoint get_actual_goal_to_return_initial_position(){
		return actual_goal_to_return_initial_position_;

	}

	void set_metros_al_dynamic_goal_Vform(double in_metros){
		 metros_al_dynamic_goal_Vform_=in_metros;
	}

	void set_distance_to_stop_from_first_person_comp(double in_distance){
		distance_to_stop_from_first_person_comp_=in_distance;
	}

	double distance_to_stop_from_first_person_comp_;

  protected:

    /**
     * \brief this function calculates the planner path in the companion case for the companion person
     * of the Cscene_sim, if no solution is found, then a false result is returned
     */
    bool robot_plan_anticipative_krrt_companion_person_companion( Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning);

    /**
     * \brief this function initializes the planner of the companion person akp planner, selecting the persons to be considered
     * and other things. If setting is not possible, i.e. goal is within current goal, etc..
     * a false result is returned (COMPANION CASE)
     */
    bool init_robot_plan2_person_companion(double robot_person_distance=1.5);

    /**
     * \brief this function samples the workspace for the person_companion (akp planning)
     */
    Spoint sample_workspace_for_person_companion_akp();
    void reset_scene_persons_propagation_flag();

    unsigned int find_nearest_vertex_person_companion_akp( const Spoint& random_goal ); //for the akp of the person companion.

    /**
     * \brief this function calculates the input required
     * for the propagation to get the sampled position
     */
     Sedge_tree_pcomp calculate_edge_person_companion_akp( unsigned int parent_vertex, const Sdestination& random_goal, Cperson_abstract::companion_reactive reactive =Cperson_abstract::Akp_planning);
    //Sedge_tree_pcomp calculate_edge_companion_person_akp( unsigned int parent_vertex, const Sdestination& random_goal, Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning );
    // for person companion, planning akp
    /**
     * \brief this function propagates the selected node,
     * if a collision is detected returns TRUE and if propagation
     * was not possible, false
     */
    bool propagate_vertex_person_companion_akp( unsigned int parent_index , const Sedge_tree_pcomp& u); // para la companion person usar el akp planner
    /**
     * \brief this function checks for collision for the propagated robot
     * state either with static obstacles as well as persons (their prediction)
     */
    //bool check_collision( const SpointV_cov& p2,  int t = -1 );
    double cost_to_go_person_companion_akp( const Spoint& random_goal, unsigned int i , Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning);

    Sforce force_persons_int_planning_virtual(Cperson_abstract* center , unsigned int t = 0, double min_dist2= 64.0, Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning); // to do the group propagation
    Sforce force_persons_int_planning_virtual_robot_prediction(Cperson_abstract* center , unsigned int t, double min_dist2=64.0, Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning, unsigned int id_pers_comp_rob=0);

    Sforce force_persons_int_planning_virtual_robot_companion_propagation(Cperson_abstract* center , unsigned int t= 0, double min_dist2= 64.0, Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning); // the function is for akp planer companion. To do the robot propagation
    // for person_companion that use the akp planning.
    Sforce force_persons_int_planning_virtual_companion_person_akp(Cperson_abstract* center , unsigned int t=0, double min_dist2= 64.0, Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning);
    Sforce force_persons_int_planning_virtual_companion_person_akp_person_prediction(Cperson_abstract* center , unsigned int t=0, double min_dist2= 64.0, Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning);

    Sforce force_persons_int_robot_prediction_virtual(const SpointV& center , unsigned int t = 0, double min_dist2 = 16.0);
    Sforce force_objects_laser_int_planning_virtual( Cperson_abstract* person, unsigned int planning_index = 0, double min_dist2=25.0, bool robot_collision_check_flag=false);
    Sforce force_objects_laser_int_planning_virtual_robot_propagation( Cperson_abstract* person, unsigned int planning_index=0, double min_dist2=25.0, bool robot_collision_check_flag=false);

    void calculate_navigation_instant_work(  );
    void calculate_cost_person_companion_akp( unsigned int parent_index , Sedge_tree_pcomp u , Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning);

    //preprocess different solutions to calculate the best or bests solutions
    unsigned int global_min_cost_index(   Crobot* robot_act, Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning );
    void preprocess_global_parameters(Crobot* robot_act);
    void calculate_normalization_cost_functions_parameters_erf();
    void calculate_normalization_cost_functions_parameters_norm(Crobot* robot_act);
    void calculate_non_dominated_solutions();


    //planning paramters
    plan_mode plan_mode_;
    distance_mode distance_mode_;
    global_mode global_mode_;
    unsigned int max_iter_;  // numero de nodos por path. de normal hay 40 nodos por path.
    std::vector<double> cost_parameters_;
    Spoint goal_,local_goal_, goal2_;
    bool reaching_goal_;
	//list of person to be considered in by the planner
	std::list<Cperson_abstract*> nearby_person_list_;
	std::vector<Spoint> nearby_obstacle_list_; //lista de obstaculos estaticos, solo para robot.
	std::vector<Spoint> nearby_obstacle_list2_; // lista de obstaculos estaticos para companion (ely)
	double workspace_radii_, std_goal_workspace_;
	double max_v_by_system_, cost_angular_;
	double xy_2_goal_tolerance_, v_goal_tolerance_, local_v_goal_tolerance_, distance_to_stop_;

    // alpha and sum_ratio paramter to scape local minima
    double alpha_,gamma_,delta_;
    std::vector<double> v_alpha_,v_gamma_,v_delta_;

    //random generator
	std::default_random_engine generator_;


    //tree structure data structures
    std::vector<Sedge_tree_pcomp> edge_;
    std::vector<double> cost_int_forces_;
    std::vector<double> cost_robot_;
    std::vector<double> cost_obstacles_;
    std::vector<double> cost_past_traj_;
    //std::vector<double> cost_local_minima_;
    std::vector<double> nodes_in_branch_;
    std::vector<double> cost_distance_;
    std::vector<double> cost_orientation_;
    std::vector<Spoint> random_goals_;
    std::vector<bool> collision_detected_;
    double work_persons_, work_robot_;


    //results data structures
    std::vector<unsigned int> best_plan_vertex_index_, end_of_branches_index_, nondominated_plan_vertex_index_, nondominated_end_of_plan_vertex_index_;
    std::vector<Spose> best_planning_trajectory_;
    std::vector<double> best_costs_, mean_costs_, std_costs_;

    //cost-to-go function values to measure real performance for testing
    std::vector<double> cost_values_;

    Spose last_pose_command_;//comand of the last velocity command send to the robot platform


    //Multi-Objective optimization: normalization terms
    double mean_cost_int_forces_,
		mean_cost_robot_,
		mean_cost_obstacles_,
		mean_cost_past_traj_,
		mean_cost_distance_,
		mean_cost_orientation_,
		mean_cost_companion_; // mean companion cost, add by ely.

    double std_cost_int_forces_,
		std_cost_robot_,
		std_cost_obstacles_,
		std_cost_past_traj_,
		std_cost_distance_,
		std_cost_orientation_,
		std_cost_companion_; // std companion cost, add by ely.

    //Multi-objective vector structure to find non-dominated sets
    std::vector<Smulticost_pcom> multicosts_;
    std::list<Smulticost_pcom> nondominated_multicosts_;

    //to calculate probability of collision
    double gaussian_constant_;
    int ppl_collision_mode_,pr_force_mode_;


    // ely functions
    /**
     * \brief this function calculates the companion cost for each node of the path.
     */
    void calculate_companion_cost_node(unsigned int parent_index, Crobot* robot_act); // function to calculate one node companion cost
    /**
     * \brief this function checks for collision for the propagated robot+person=couple
     * state either with static obstacles as well as persons (their prediction).
     */
    bool check_collision_person_companion_akp( const SpointV_cov& p2 ,  int t = -1);
    bool check_collision_final( const Spoint& p ,  int t); // caso robot, moverse dentro de la burbuja!

    /**
     * \brief this function calculates the additional companion cost when two adjacent angles in the best path
     * for the position of the robot have a difference greater than the initia_increment_angle and
     * the intermediates cost were not taken into account in the calculation o the initial path
     */
    double calculate_companion_addition_cost_node_of_big_angle_difference( unsigned int parent_index, double actual_angle);

    /**
     * \brief this function calculates the initial_increment_angle to move the robot around the person
     * when they have to avoid obstacles in a joint way.
     */
    void ini_increment_angle();
    void ini_increment_angle_person_companion_akp();
    void calculate_companion_path_angle_and_cost(unsigned int parent_index, double actual_angle);
    void return_next_robot_position_companion_cost_and_angle(unsigned int parent_index);
    void only_cost_calculate_companion_path_angle_and_cost(unsigned int parent_index, double actual_angle);
    /**
     * \brief this function calculates the needed angle between person-and-robot
     * for all the nodes of the best path, to obtain a progressive angle
     *  to positioned the robot around the person. (case, go robot in front of the person)
     */
    void only_angle_in_final_tree_calculate_companion_path_angle_and_cost(unsigned int i);
    void only_angle_in_final_tree_calculate_companion_path_angle_and_cost2(unsigned int i);
    /**
     * \brief this function calculates the needed angle between person-and-robot
     * for all the nodes of the best path, to obtain a progressive angle
     *  to positioned the robot around the person. (case, go robot at the rear of the person)
     */
    void go_behind_robot_only_angle_in_final_tree_calculate_companion_path_angle_and_cost2(unsigned int i);
    void go_behind_robot_only_angle_in_final_tree_calculate_companion_path_angle_and_cost3(unsigned int i);

    void go_in_front_robot_only_angle_in_final_tree_calculate_companion_path_angle_and_cost3(unsigned int i);

    /**
     * \brief this function is for see the best path with the companion angles and the companion cost.
     */
    void see_companion_path_angle_and_cost_of_min_cost_paths(double parent_index);
    /**
     * \brief this function modifies the distance between the person and robot couple to smooth the turns
     * to put the robot in front or rear the person.
     */
    double calc_robot_person_companion_distance();
    /**
     * \brief this function replan the propagated position of the robot for the next step
     * of the robot in the best path, taking into account in the propagation the two goals for the robot:
     * follow the path and goes to a concrete position respect to the person.
     */
    Spose replan_last_step_person_companion(unsigned int index, Spose robot_pose_ini, SpointV robot_Spoint_ini, Sdestination robot_goal, double dt, Sdestination robot_final_goal);
    /**
     * \brief these function perform a different collision functions for the goal in the atractive or repulsive cases.
     */
    double check_collision_companion_goal(const Spoint& p ,  int t);
    /**
     * \brief this function set the actual max_velocity_ for the robot.
     * this is the initial function only taking into account the planning for the robot.
     */
    void vel_robot_normal(double d_min);
    /**
     * \brief this function set the actual max_velocity_ for the robot.
     * this function takes into account the planning for the robot-and-person=couple.
     */
    void vel_robot_companion(double d_min , double robot_person_distance, Spoint before_next_goal_of_robot=Sdestination(), Cperson_abstract::companion_reactive reactive=Cperson_abstract::Reactiva_repulsive);

    void calculate_actual_angle_person_robot(unsigned int index);
    double calculate_actual_angle_person_robot(Spoint person, Spoint robot);
    /**
     * \brief this function returns the actual max_velocity_ to the max velocity of the robot.
     *  this is to calculate the akp path with the max velocity. Then, in the recalculation of
     *  the position for the best path, the velocity of the robot is modified taking into account
     *  also the goal of remain in a exact position respect to the companion person.
     */
    void return_max_velocity_systemRobot_to_max_value();

    /**
     * \brief this function calculate the anisotropy for the group to do not take into account
     * the collisions with obstacles at the rear of the group, for the robot positioning around the person.
     * (now is set, anisotropy_limit=0, any anisotropy is take into account, we take into account rear collisions)
     */
    double calculate_anisotropy(const SpointV_cov& center_person, const Spoint& interacting_person , double lambda=0.2);
    /**
     * \brief this function calculates the distance between the person-and-robot in the moments where the robot
     * moves around the person to avoid obstacles (put in front or behind). This function is to smooth the
     * turns of the robot, facilitate the navigation and remove the misalignments with the person in orientation
     * that provoke "s" movements for the robot.
     */
    double calculate_modif_person_robot_distance( double ite );
    double calc_robot_person_companion_distance_companion_person_akp(); // case person companion planning akp
    /**
     * \brief this function print the results in a matlab file.
     */
    void printToMatlab();
    void evaluate_costs_printToMatlab(Crobot* actual_robot);
    void calc_area_workspace_arround_person_companion(); // not used, bad idea. Only sample arrount the person prediction. not works and is not proper.
    void new_matlab_file_To_evaluate_costs();
    void new_debug_file();
    void new_matlab_file_To_evaluate_change_distance_and_angle();
    void evaluate_change_distance_and_angle_companion_with_beta_change();

    void calculate_intersecction_line_circle(double Xc, double Yc, double radius, double X_ini, double Y_ini, double X_final, double Y_final, double &intersection1, double &intersection2 );

	void robot_group_trajectory_prediction( double new_horizon_time, Sdestination goal_to_predict_pose);

	double calc_person_companion_orientation(); // return theta person companion in radians!
	Sdestination calculate_companion_goal_stable_if_person_stop(double in_act_min_companion_angle);
	Sdestination calc_companion_goal_position(); // claculation of companion goal with the actual companion angle (from collisions etc) and the position of the robot and the person.

	Spoint SIM_initial_person_goal_pose_actual_;
	unsigned int id_person_goal_; // (compartidas con sim person goal, no eliminar) id of the person that the group goes to talk, is a dynamic goal for each iteration.
	unsigned int change_id_person_goal_;
	bool group_go_to_interact_with_other_person_;

	Sdestination before_person_intersection_destination_;
    // variables companion (ely)
    bool debug_companion_;
    double robot_person_companion_distance_;
    double min_distance_collision_; // variables to calculate the companion cost and angle! => min_distance_collision_ and bool_collision_companion_
    bool bool_collision_companion_; // variables to calculate the companion cost and angle!
    double next_robot_pose_min_distance_collision_;
    std::vector<double> orientation_person_robot_angles_; // angles to calculate the companion goal using the computed path of the group

    std::vector<double> orientation_person_robot_angles_with_prediction_of_person_companion_; // angles to calculate the companion goal using the prediction of the companion person.
    bool calc_goal_companion_with_group_path_; // if true == calc the angle of the goal companion with the group path, else calc the angle of the goal companion with the prediction of the person.
	void fix_angles_to_use_person_prediction_for_the_companion_goal(); //function to fix the companion angles obtained with the person prediction.


    // new costs (companion-ely)
    std::vector<double> cost_companion_;
    std::vector<unsigned int> parent_index_vector_;
    std::vector<double> min_distance_collision_vector_;
    std::vector<double> min_distance_collision_vector_from_pred_;
    // NOTA: ojo!!! no hemos definido como calculo el coste del path entero, solo el coste en los nodos. he de mirar como lo hace Gonzalo para los otros y ver si puedo usar una de esas formas. Si hago una media del coste este durante el path o algo asi...
    double min_next_companion_angle_; // esta en grados
    double min_next_companion_angle_for_final_dest_;
    double min_next_companion_cost_;
    double angle_increment_of_increment_distance_;
    double angle_increment_of_increment_distance_vuelta_al_reves_;

    double before_initial_angle_;
    double before_initial_cost_;
    std::vector<unsigned int> BEST_path_parent_index_vector_;

    bool debug_comanion_good_;
    unsigned int id_person_companion_;
    unsigned int change_id_person_companion_;
    Sdestination person_destination_;
    //Spose
    double person_colision_radi_;
    SpointV_cov companion_person_position_; // posicion inicial de la persona a la que acompañamos.
    std::list<Cperson_abstract*> person_companion_nearby_person_list_;  // personas cercanas a la persona que acompanya el robot.
    double person_center_min_distance_collision_;
    //const std::vector<SpointV_cov>* companion_person_planning_trajectory_;//std::vector<SpointV_cov> companion_person_planning_trajectory_; //virtual const std::vector<SpointV_cov>* companion_person_planning_trajectory_
    Sdestination person_companion_goal_;
    double robot_person_distance_;
    //Cforce_reactive_robot_companion reactive_class_;
    double robot_person_proximity_distance_;  // distancia a la que esta robot y persona para que se olvide del goal el robot y se acerque a la persona.
    double robot_person_proximity_tolerance_; // increment of distance around the proximity_distance to generate a distance margin. This is for do not always omit the planning and aproximate to the person.
    double robot_person_proximity_goals_x_; // distancia entre goal de la persona que acompaña y goal al que se dirige el robot.
    double robot_person_proximity_goals_y_;
    double initial_angle_;
    Spose robot_initial_pose_;
    SpointV_cov initial_robot_spoint_;
    SpointV_cov initial_person_companion_point_;


    Cperson_abstract* pointer_to_person_companion_;
    bool we_have_pointer_to_first_person_; // if true, we can use the pointer!!!

    double robot_person_distance3_;

    bool case_robot_companion_; // bool para cambiar velocidad en person companion. hacer que vaya algo más despacio, para que reaccione mejor.
    std::vector<Spoint> random_goals2_;
    unsigned int min_branch_index_save_;

    bool debug_antes_subgoals_entre_AKP_goals_;

    bool final_collision_check_;

    bool actual_debug_;
    bool actual_debug2_;
    double offset_atractive_;

    double time_act_;
    //double time_ant_;
    double inct_;

    double robot_adition_complete_esphere_companion_distance_;

    double final_v_robot_max_;
    double final_v_real_robot_max_;

    double f_obst_max_x_; // =0.1
    double f_obst_max_y_; //=0.1

    Sdestination robot_destination_;

    bool companion_same_person_goal_;
    Sdestination extern_robot_goal_;
    Sdestination extern_robot_goal_fix_;

    double angle_companion_;
    double person_goal_percent_;
    double marge_angle_companion_;
    Sdestination person_companion_goal_out_;

    double dmin_global_;
    double angle_companion_temp_movil_;
    double angle_companion_temp_movil_complementarion_detras_;
    std::vector<double> min_angles_colisions_; // al calcular el min_colision. a que angulos estan esas colisiones
    std::vector<double> angles_colisions_; // para cada estep, el angulo a donde esta la minima colision
    std::vector<double> min_step_collision_distance_;
    double min_angle_collision_;
   bool debug_nadal_;
   bool debug_nadal2_;

   bool robot_orient_;
   double person_orient_;

   double vel_per_max_caso_robot_a_0_o_180_grados_;

   bool debug_real_test_companion_;
   bool debug_real_test_companion2_;
   bool debug_real_test_companion3_;
   bool debug_real_test_companion4_;

   double marge_in_distance_; // margen en distancia para seguir a una persona.
   bool overpas_obstacles_behind_person_;
   bool we_have_cost_companion_;

   double anisotropy_threshold_;

   Cperson_abstract::companion_reactive robot_companion_case_;
   bool check_execution_times_;

   double person_robot_actual_real_distance_;
   double little_augmented_collision_margin_;
   double person_radi_;
   double person_radi2_;
   double person_radi_amp_;
   double person_radi_per_comp_;
   double max_distance_to_obstacles_detected_;
   bool see_forces_;
  double obstacle_radi_;
  double obstacle_radi2_;
  double obstacle_radi_amp_;

  Sedge_tree_pcomp u_forces_robot_actual_; // solo para visualizacion en Rviz
  Cperson_abstract::vel_per_ok vel_per_ok_; //boolean, true= robot navigate with the person velocity, else = robot navigate with the maximun velocity.


  // Variables to save in matlab file!
  double real_distance_person_robot_;
  double ideal_distance_person_robot_;
  double real_angle_person_robot_;
  double ideal_angle_person_robot_;
  double before_ideal_angle_person_robot_;
  bool first_ideal_angle_;
  double actual_time_;
  double robot_distance_cost_;
  double robot_orientation_cost_;
  double robot_control_cost_;
  double robot_other_person_cost_;
  double robot_obstacles_cost_;
  double robot_companion_cost_;
  double robot_total_cost_;
  double robot_ant_traj_cost_;
  double person_distance_to_goal_;
  double robot_distance_to_goal_;

    double last_step_robot_other_person_cost_; // coste respecto a fuerzas de otras personas que se ejercen en el robot.
    double last_step_robot_obstacles_cost_;  // coste respecto a fuerzas de obstaculos que se ejercen en el robot.
    double last_step_robot_companion_cost_; // coste respecto a fuerzas de la companion person que se ejercen en el robot.
    double last_step_robot_control_cost_mix_;
    double last_step_robot_control_cost_goal_traj_;
    double last_step_robot_control_cost_goal_person_;
    double last_step_robot_orientation_cost_local_;
    double last_step_robot_orientation_cost_global_;
    double last_step_robot_distance_cost_;

    double other_people_due_to_robot_cost_;
    double companion_person_due_to_robot_cost_;

    double workspace_arround_companion_person_min_x_;
    double workspace_arround_companion_person_max_x_;
    double workspace_arround_companion_person_min_y_;
    double workspace_arround_companion_person_max_y_;

    bool sim;
    bool sim_target_per;
    bool first_time_;
    unsigned int iteration_;
    unsigned int experiment_;

    SpointV_cov before_initial_robot_spoint_;
    double inc_distance_ant_;
    double acum_time_;
    double time_ant_;

    bool save_results_in_file_;
    double meters_goal_to_save_results_in_file_;
    bool mode_velocity_;
    std::string results_file_;
    double reduce_max_vel_dist_;

    bool go_with_vel_per_;

    bool debug_file_robot_;
	bool debug_cout_robot_;


    std::string debug_file_;

    bool case_dynamic_;

    double external_inc_vel_for_inc_angle_;

    double externa_force_k_near_goal_akp_;
    double externa_force_k_far_goal_akp_;

    double max_dist_to_near_goal_force_akp_;

    bool mode_step_near_;
    //int out_index_step_;
    unsigned int out_index_step_final_dest_goal_;
	unsigned int out_index_step_companion_goal_;

    bool only_comp_people_vel_and_robot_poses_;

    std::vector<bool> vector_of_companion_collisions_;

    bool debug_angles_;

    double ini_vel_to_increment_angle_;
    //std::vector<bool> vector_of_companion_collisions_;

    /*
     * std::ofstream fileMatlab2;
	fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
	fileMatlab2 << "% ***** Starts Autogenerated DEBUG File - results AKP dynamic companion ELY ***** \n\n";
	fileMatlab2.close();
     */
    bool debug_person_companion_general_;
    bool debug_person_companion_increment_angle_;
    bool debug_init_robot_plan2_person_companion_;
    bool debug_robot_plan_anticipative_krrt_companion_person_companion_;
    bool debug_calculate_edge_person_companion_akp_;
    bool debug_get_best_planned_pose_person_companion_akp_;
    bool debug_correct_angle_person_init_robot2_;
    bool debug_correct_angle_person_replan_min_cost_branch_;
    bool debug_correct_angle_person_replan_last_step_;
    bool debug_correct_angle_person_calculate_edge_;
    bool debug_correct_angle_person_get_best_planned_pose_;
    bool debug_correct_angle_person_vel_robot_companion_;
    bool debug_correct_angle_person_calculate_actual_angle_person_robot_;
    bool debug_correct_angle_person_print_to_matlab_;

    bool bool_case_person_companion_;
    double person_companion_desired_velocity_sim_;
    SpointV_cov ini_point_person_companion_akp_;
    Spose ini_spose_person_companion_akp_;

    int number_of_obstacles_;
    int number_obstacles_big_force_;
	unsigned int num_steps_orientation_;


	double alpha_companion_; // (weight FINAL_GOAL) alpha to replan the last step for the companion task ( 0.4 ) anterior 0.1.
	double beta_companion_;  // (weight COMPANION_GOAL) beta to replan the last step for the companion task ( 0.6 )
	double gamma_companion_; //  (weight repulsive force other people) gamma to replan the last step for the companion task (0.5) anterior 5.0
	double delta_companion_; //  (weight repulsive force static obstacles) delta to replan the last step for the companion task (0.5)

	double threshold_dintace_select_person_side_to_go_; // al principio esta distancia minimo ha de ser 0.5 metros al lado de la persona.
	bool debug_select_person_side_to_go_with_more_free_space_;

	bool chose_better_side_to_acompani_person_;

	SsavePath_multiple_paths_and_best_path iter_act_multiple_paths_and_best_path_;
	int planner_iterations_;

    std::string evaluate_costs_file_;
    std::string evaluate_change_distance_and_angle_companion_file_;

    double orientacion_persona_actual_;
	double person_x_ant_;
	double person_y_ant_;

	bool debug_file_evaluate_costs_;

	// to visualice goals:
	Sdestination robot_goal_to_person_companion_; // guardar estos en el archivo por cada iteracion.
	Sdestination robot_path_goal_;
	Spose final_combined_goal_;

	double save_distance_between_person_and_robot_;
	double save_angle_between_person_and_robot_;
	unsigned int iter_d_;

	Spoint person_position_t_4_;
	double next_companion_angle_save_;
	double before_act_companion_angle_;

	bool use_person_prediction_until_final_goal_; // no se usa, parece aunque solo esta en companion, de momento la dejo.
	calculate_complete_group_path_ calculate_complete_group_path_var_;

	Spoint before_group_position_;
	bool first_;

	action_mode Action_;

	simulation_case actual_case_;

	Spoint actual_goal_to_return_initial_position_;

	// To have several experiments to simulate.

	Spose SIM_initial_robot_pose1_;
	Spose SIM_initial_robot_pose2_;
	Spose SIM_initial_robot_pose3_;
	Spose SIM_initial_robot_pose4_;

	Spose SIM_initial_person_companion_pose1_;
	Spose SIM_initial_person_companion_pose2_;


	unsigned int global_iter_to_change_simulation_case_;

	bool first_in_itter_;
	double ros_time_to_sec_;
	bool first_time_case1_;
	bool first_time_case2_;

	unsigned int goal1_;// goal number to do.

	Spose center_of_the_group_;


	double max_prop_time_;
	double max_prop_person_;
	unsigned int max_n_prop_person_;

	bool is_act_person_companion_;


	double save_computational_time_value_;
	double clocks_start, clocks_end;
	clock_t robot_plan_companion2_start_ITER, robot_plan_companion2_end_ITER,robot_plan_companion3_start_ITER, robot_plan_companion3_end_ITER;


	Spoint final_goal_only_vel_;
	Spoint final_person_dest_;

	bool go_behid_comp_person_;

    double last_true_person_companion_orientation_;


    bool restart_real_;
    double max_dist_to_go_behind_;

    bool case_stop_giro_;
    double incremento_giro_positivo_;
	   double incremento_giro_negativo_;

	   Cperson_abstract *actual_person_Companion_pointer_;
	   Crobot *actual_person_Companion_pointer_robot_;
	   SpointV_cov actual_person_Companion_SpointV_;
	   Sdestination actual_person_Companion_destination_;
	   double max_desired_person_Companion_velocity_; // todo: add initial bucle: max_desired_person_Companion_velocity_=iit->get_desired_velocity();


	   double d_min_global_;

	   double radii_person_list_include_person_companion_;
	   bool complete_traj_prediction_to_goal_;
	   double set_distance_person_robot_;

	   // variable save results on files or not! OJO!!! cuando quiera los datos hay que activarla!!!
	   bool save_results_on_files_for_person_companion_;
	   bool save_results_on_files_for_robot_; // only do not save the results on files for the robot
	   bool save_in_file_; // disable also the iterations!!! not only save the results on files. (only for robot)

	   // variables change forces between person companion and robot.
	   Sforce force_int_between_person_comp_and_robot_;

	   // for performance:
	   double value_distance_global_path_;
	   double global_path_ini_orientation_;
	   double global_path_final_orientation_;

	   double multiply_person_companion_force_to_persons_;
	   bool final_debug_journal_;
	  double threshold_min_vel_person_to_obtain_destination_;  // threshold_min_vel_person_to_obtain_destination_=0.2


	  double last_good_theta_person_;
	  bool firts_iter_obtain_angle_person_companion_;

	  double saved_actual_theta_person_;

	  bool debug_gazebo_journal_;
	  bool debug_gazebo_journal2_;

	  Sdestination ideal_companion_goal_in_the_next_instant_of_time_; // serve to stop robot when person stop and the robot is in the best companion point for this stop person. (para esta persona parada)
	  Sdestination before_companion_person_destination_;

	  double clocks_per_sec_my_var_; // clocks_per_sec_my_var_=1000000

	  double test_distance_to_goal_;

	  bool final_goal_reached_in_node_;
	  bool change_final_robot_orientation_; //


	  std::vector<std::vector<double> > save_best_cost_branch_;
	  bool is_case_akp_true_;
	  bool enter_on_cero_;
	  bool enter_on_cero2_;
	  bool result_;

	  double dist_to_goal_test_;
	  double v_to_goal_test_;

	  bool use_persoon_companion_to_calculate_plan_;

	  Spoint medium_point_;

	  unsigned int change_goal_of_the_error_;


	  double initial_increment_of_time_from_file_; // variable of the time obtained from the frequency of the file from the simulator of companion, to obtain the parameters of the forces.

	  Sforce f_overpas_obs_forward_;
	  Spose robot_propagation_overpas_obs_forward_;
	  double act_min_companion_angle_overpas_obs_forward;
	  Sdestination person_companion_goal_overpas_obs_forward;
	  Sforce saved_force_companion_go_rear_;
	  Sforce saved_force_companion_go_front_;

	  // Parameters for second person companion.
	 // unsigned int id_SECOND_person_companion_;
	 // SpointV_cov second_group_companion_personSpoint_;
	  //pointer_to_SECOND_person_companion_
	  //Spose second_group_companion_personSpose_;
	  //Sdestination second_group_companion_personDest_;
	  //bool we_have_second_person_companion_;

	    Sdestination akp_out_path_goal_;
	    Sdestination akp_out_companion_goal_;
	    double companion_angle_peopl_in_group_; // angle in degrees.
	    double real_distance_between_people_of_group_;
	   // Cperson_abstract* second_group_companion_person_obj_;
	    //bool we_have_pointer_to_second_person_; // if true, we can use the pointer!!!

	    // TODO:insertar prediccion de la person companion de la person_list_ del scen_sim.

	    Spoint before_next_goal_of_robot_; // (is the next immediate final goal to accompany the person) new variable to adjust well the robot velocity to accompany the person. (pure companion, do not takes into account the final goal for the velocity)
	    // sacar marker de esta variable al nodo de ros, para ver si hace lo que quiero que haga


	    // *** variables for use Francesco's companion model
	   // bool Zanlungo_model_; // Francesco_model_=true, acting francesco model if false acting my model
	   // bool Zanlungo_model3_;
	    //Cgenome genome_params_;
	    //CompanionFrancescoModel Companion_Zanlungo_Model_;

	    unsigned int number_of_group_people_;

	    //Sdestination before_person_comp_goal_Zalungo_formation_;
	    std::vector<Sedge_tree_pcomp> saved_tree_forces_to_see_companion_Force_Zanlungo_;
	   // std::vector<Sedge_tree_pcomp> saved_tree_forces_to_see_companion_Force_Zanlungo_best_path_;


	    std::vector<double> person1_repulsive_force_fx_;
	    std::vector<double> person1_repulsive_force_fy_;

	    std::vector<double> person1_companion_force_fx_;
	    std::vector<double> person1_companion_force_fy_;

	    double person1_repulsive_force_fx_act_;
	    double person1_repulsive_force_fy_act_;

	    double person1_companion_force_fx_act_;
	    double person1_companion_force_fy_act_;

	    //std::vector<double> person2_repulsive_force_fx_;
	    //std::vector<double> person2_repulsive_force_fy_;

	   // std::vector<double> person2_companion_force_fx_;
	  //  std::vector<double> person2_companion_force_fy_;

	    //double person2_repulsive_force_fx_act_;
	   // double person2_repulsive_force_fy_act_;

	    //double person2_companion_force_fx_act_;
	    //double person2_companion_force_fy_act_;

	    //bool debug_zanlungo_;

	   // double constante_multiplicar_fuerza_base_Zanlungo_;
	    double constante_multiplicar_fuerza_goal_;

	    double ori_pers_x_;
	    double ori_pers_y_;

	    std::vector<double> distance_betw_per_robo_plann_;
	    std::vector<bool> in_margin_true_;

	   // double up_distance_margin_Zamlungo_;
	   // double down_distance_margin_Zamlungo_;
	    //bool bool_robot_propagation_Zanlungo_;

	    bool bool_distance_margin_;


	   // std::string data_file_Zanlungo_;
	    //void new_Zanlungo_file();
	    double final_compt_time_;
	    Spose final_pose_robot_;

	   // std::vector<Vector2D> preferred_paths_Zanlungo_;
	    std::vector<double> dist_per_rob_path_;
	    unsigned int actual_best_path_index_;
	    std::vector<double> companion_person_theta_path_; // es la orientacion de la persona haca el goal
	    // faltan setear apartir de aqui
	    /*std::vector<double> v_desired_robot_x_; // estan dentro de person abstract es mas complicado.
	    std::vector<double> v_desired_robot_y_;
	    std::vector<double> v_current_robot_x_;
	    std::vector<double> v_current_robot_y_;
	    std::vector<double> actual_random_goal_x_;
	   	std::vector<double> actual_random_goal_y_;*/
	    clock_t group_force_start, group_force_end, rrt_start_ITER,rrt_end_ITER,initial_part_start,initial_part_end,last_part_start,last_part_end;

	   // bool  Zanlungo_model2_;

		double dis_tol_, dis_tol_side_;

		void robot_trajectory_prediction_for_group_Zanlungo( double new_horizon_time, Sdestination goal_to_predict_pose, unsigned int id_person_comp);
		// no se ahora mismo si sale fuera esta de encima.
		void find_entity_order_in_the_group();
		void find_entity_order_in_the_group_for_person_companion();
		Sforce ant_final_force_total_;

	    //bool debug_output_screen_mesages_;
	   // bool print_to_zanlungo_file_;

	    int *order_people_in_group_global_variable_;

	    bool change_ids_group_people_in_recongigure_;

	    SpointV center_of_the_group_Zanlungo_; // estas NO se borran, las usa tambien 1 person companion
	    SpointV mean_people_Zalungo_; // estas NO se borran, las usa tambien 1 person companion
	    double theta_of_the_group_Zanlungo_; // estas NO se borran, las usa tambien 1 person companion

	    double threshold_vel_pers_to_stop_;
	    double dist_bet_rob_ext_goal_to_stop_;
	    double dist_bet_pers1Com_ext_goal_to_stop_;
	    double dist_bet_pers2Com_ext_goal_to_stop_;

	    double mean(std::vector<double> arr, int n);   // Function to find mean.
	    double variance(std::vector<double> a, int n); // Function to find variance.
	    double covariance(std::vector<double> arr1, std::vector<double> arr2, int n); // Function to find covariance.

	    double a_,b_; // parameters y=a+bx of the linear regression to find the preferred velocity of the robot from the actual person companion velocity. For the group is the mean of the velocity of the two people.

	    std::vector<double> y_for_linear_regression_,x_for_linear_regression_;

	    bool debug_stop_behaviour_;

	   // std::vector<double> robot_desired_vel_calc_Zanlungo_; // the desired velocity for the robot, calculated from the persons companion velocity for the Zanlungo model.


	    double treshold_distance_between_steps_;
	    double threshold_max_total_force_;
	    double real_vel_per;
	    //Sforce companion_force_my_model_to_compare_with_zanlungo_;
	   // Sforce goal_force_my_model_to_compare_with_zanlungo_;

	    //Spose before_pose_robot_for_limit_w_zanlungo_;

	   // std::vector<double> des_vel_zanlungo_;
	    std::vector<double> des_vel_v_people_;
	    //bool calculated_desired_vel_;
	    double desired_vel_seteada_desde_fuera_;
	    bool change_k_robot_,change_k_person_,change_k_person_comp_,change_k_obst_;
	    bool initial_goal_case_bool_;

	    Sdestination last_people_dest_;
	    Spoint las_people_dest_spoint_;
	    bool initial_not_set_last_people_dest_;

	    double minimun_velocity_case_stop_initial_;

	    double final_max_v_;

	    double initial_limit_distance_goal_to_person_robot_stop_; // by deafault= 0.1 m/s

	    bool bool_out_init_pose_robot_;

	    double ideal_max_robot_velocity_; // 0.9 in muy system

	    //bool debug_cost_francesco_;

	    bool bool_tes_prop1;
	    bool bool_tes_prop2;

	    std::vector<unsigned int> order_people_in_group_;

	    Sdestination before_person_comp_goal_;

	   // bool plan_create_fake_fixed_second_person_companion_;
	    bool actual_case_sim_goal_;

	    Spoint SIM_initial_person_goal_pose1_,SIM_initial_person_goal_pose2_,SIM_initial_person_goal_pose3_;
	    double actual_robot_performance_;

	    double metros_al_dynamic_goal_Vform_;
};


#endif /* PLANN_LOCAL_NAV_PERSON_COMPANION_H_ */

