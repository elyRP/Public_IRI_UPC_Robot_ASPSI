
/*
 * generate_performance_from_data_example.cpp
 *
 *  Created on: 7-Sep-2018
 *      Author: Ely Repiso  (This performance extraction was originally created by Ely Repiso (anything is from Gonzalo).)
 */
 /*
 * 
 *
 *  Code Created on: Dec 22, 2013 by Gonzalo Ferrer. 
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

//#include "nav/plan_local_nav.h"
//#include "nav/plan_local_nav_person_companion.h"
//#include "nav/force_reactive_robot_companion.h"
//#include "scene_sim.h"
#include <iostream>
#include "companion_zanlungo/CompanionFrancescoModel.h"

int main(int argc,char *argv[])
{
	  CompanionFrancescoModel Companion_Zanlungo_Model_;


	    double in_Cr, in_r0, in_Ct, in_eta, in_rmax, in_S0;
	    in_Cr=0.62;
	    in_Ct=0.08;
	    in_r0=1.5;
	    in_eta=-0.43;
	    in_rmax=3;
	    in_S0=0.5;
	    Cgenome genome;

	    genome.Init(in_Cr, in_r0, in_Ct, in_eta, in_rmax, in_S0);

	    Companion_Zanlungo_Model_.set_genome_params_in_CompanionFrancescoModel(genome);
	    Companion_Zanlungo_Model_.set_internal_k_value(1.5);

	// TODO: falta ver como cargar los datos del fichero txt ahora tal como esta.

	std::string file_name_s="/home/ely7787/catkin_ws/src/AKP_WS/navigation/iri_robot_aspsi/txt_data/";
	const char * file_name=file_name_s.c_str();

	std::vector<double> actual_robot_performance_;
	std::vector<double> all_means_of_performances_ex_;
	std::vector<double> all_std_of_performances_ex_;

	std::cout << " OOOOOOOOOOOO77777777777777OOOOOOOOOOOOOOOO file_name="<<file_name<<std::endl;

	//ifstream reader;//(file_name);
	fstream reader;
	//reader.open(file_name_s);
	reader.open("results_person_companion_side_by_side_real.txt");
	cout <<"open_file="<<open_file<<endl;
	unsigned int experiment_,iteration_;
	experiment_=1;
	iteration_=2;
	unsigned int max_exp=8;//275-129; // until max_exp;
	unsigned int firs_experiment=1; //



	double real_computation_time_dt;

	double companion_person_x;
	double companion_person_y;
	double companion_person_vx;
	double companion_person_vy;

	double companion_person_dest_x;
	double companion_person_dest_y;

	double companion2_person_x;
	double companion2_person_y;
	double companion2_person_vx;
	double companion2_person_vy;

	double companion_person2_dest_x;
	double companion_person2_dest_y;

	double robot_x;
	double robot_y;
	double robot_vx;
	double robot_vy;
	double robot_theta;
	double robot_v;
	double robot_w;

	double robot_dest_x;
	double robot_dest_y;
	// double robot_performance;


cout <<"antes bucle experiments ="<<endl;
	//while( experiment_< max_exp){
	cout <<"iteration_"<<iteration_<<"max_exp"<<max_exp<<endl;
while( iteration_< max_exp){
		 cout <<"antes bucle iterations"<<endl;
		// while(experiment_==firs_experiment){

		 cout <<"reader.good()="<<reader.good()<<endl;
	// TODO: coger valores de arriba del fichero. Iterativamente.
	if (reader.good()) {

		/// INICIO BUCLE, tendria que ser desde aquí
		//###############################
		//# experiment_ and  iteration_ #
		//###############################
		 string line;
		 getline(reader,line);
		 getline(reader,line);
		 cout <<"(1)"<<endl;
		 //experiment_=-1;
		 cout <<"(2)"<<endl;
		 //find first comma, read #rows
			 string::size_type i = line.find("(");
			 cout <<"(3)"<<endl;
			 if (i!=reader.eof()) {
				 cout <<"(4); i="<<i<<endl;
			 	string cstr = line.substr(i+1,i+2);
			 	 cout <<"(5)"<<endl;
			 	experiment_ = atoi(cstr.c_str());
			 	 cout <<"(6)"<<endl;
			 } else {
			 	cout <<"ERROR @ GMap::read num_people_mov"<<endl;
			 	exit(EXIT_FAILURE);
			 }
			 /*if(experiment_==0){
				// getline(reader,line);
				 cout <<"(1)"<<endl;
						 experiment_=-1;
						 cout <<"(2)"<<endl;
						 //find first comma, read #rows
							 string::size_type i = line.find("(");
							 cout <<"(3)"<<endl;
							 if (i!=reader.eof()) {
								 cout <<"(4); i="<<i<<endl;
							 	string cstr = line.substr(i+1,i+2);
							 	 cout <<"(5)"<<endl;
							 	experiment_ = atoi(cstr.c_str());
							 	 cout <<"(6)"<<endl;
							 } else {
							 	cout <<"ERROR @ GMap::read num_people_mov"<<endl;
							 	exit(EXIT_FAILURE);
							 }
			 }*/
			 cout <<"experiment_="<<experiment_<<endl;
			// p=experiment_;

			 string::size_type j = line.find(")");
			 /*if(j>1000){
				 j=7;
			 }*/

			 cout <<"j="<<j<<endl;
			 if (j!=reader.eof()) {
			 	//string cstr = line.substr(j-1,j);
			 	///iteration_ = atoi(cstr.c_str());
			 } else {
			 	cout <<"ERROR @ GMap::read num_people_mov"<<endl;
			 	exit(EXIT_FAILURE);
			 }
			 cout <<"iteration_="<<iteration_<<endl;

			/* if(iteration_==2){
				 cout <<"iteration_="<<iteration_<<"; firs_experiment="<<experiment_<<endl;
				 firs_experiment=experiment_;
			 }*/

			 getline(reader,line);
			 i = line.find(" ");
			 unsigned int k=line.size();
			 cout <<"; i="<<i<<"; K="<<k<<endl;
			 if (i!=reader.eof()) {
				//string cstr = line.substr(i,k);
			 	//real_computation_time_dt = std::stod(cstr.c_str());

			 } else {
			 	cout <<"ERROR @ GMap::read num_people_mov"<<endl;
			 	exit(EXIT_FAILURE);
			 }
				cout <<"real_computation_time_dt="<<real_computation_time_dt<<endl;
		//###########################
		//# Read companion person1  #
		//###########################

				//# Read companion person1 position #
			getline(reader,line);
			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			 if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				companion_person_x = std::stod(cstr.c_str());

			 } else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"companion_person_x="<<companion_person_x<<endl;

			getline(reader,line);
			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			 if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				companion_person_y = std::stod(cstr.c_str());

			 } else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"companion_person_y="<<companion_person_y<<endl;

			//# Read companion person1 velocities #
			getline(reader,line);
			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			 if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				companion_person_vx = std::stod(cstr.c_str());

			 } else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"companion_person_vx="<<companion_person_vx<<endl;

			getline(reader,line);
			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			 if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				companion_person_vy = std::stod(cstr.c_str());

			 } else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"companion_person_vy="<<companion_person_vy<<endl;

			getline(reader,line); // companion_person_theta(1,1)= -nan

			//# Read companion person1 destinations #
			getline(reader,line); // companion_person_dest_x(1,1)= 0.220653

			i = line.find(" ");
			k=line.size();
			if(k>1000){
				k=38;
			}
			cout <<"; i="<<i<<"; K="<<k<<endl;
			 if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				companion_person_dest_x = std::stod(cstr.c_str());

			 } else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"companion_person_dest_x="<<companion_person_dest_x<<endl;

			getline(reader,line); // companion_person_dest_x(1,1)= 0.220653

			i = line.find(" ");
			k=line.size();
			if(k>1000){
				k=38;
			}
			cout <<"; i="<<i<<"; K="<<k<<endl;
			 if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				companion_person_dest_y = std::stod(cstr.c_str());

			 } else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"companion_person_dest_y="<<companion_person_dest_y<<endl;

			//###########################
			//# Read companion person2  #
			//###########################
					//# Read companion person2 position #
				getline(reader,line);
				i = line.find(" ");
				k=line.size();
				cout <<"; i="<<i<<"; K="<<k<<endl;
				 if (i!=reader.eof()) {
					string cstr = line.substr(i,k);
					companion2_person_x = std::stod(cstr.c_str());

				 } else {
					cout <<"ERROR @ GMap::read num_people_mov"<<endl;
					exit(EXIT_FAILURE);
				}
				cout <<"companion2_person_x="<<companion2_person_x<<endl;

				getline(reader,line);
				i = line.find(" ");
				k=line.size();
				cout <<"; i="<<i<<"; K="<<k<<endl;
				 if (i!=reader.eof()) {
					string cstr = line.substr(i,k);
					companion2_person_y = std::stod(cstr.c_str());

				 } else {
					cout <<"ERROR @ GMap::read num_people_mov"<<endl;
					exit(EXIT_FAILURE);
				}
				cout <<"companion2_person_y="<<companion2_person_y<<endl;

				//# Read companion person2 velocities #
				getline(reader,line);
				i = line.find(" ");
				k=line.size();
				cout <<"; i="<<i<<"; K="<<k<<endl;
				 if (i!=reader.eof()) {
					string cstr = line.substr(i,k);
					companion2_person_vx = std::stod(cstr.c_str());

				 } else {
					cout <<"ERROR @ GMap::read num_people_mov"<<endl;
					exit(EXIT_FAILURE);
				}
				cout <<"companion2_person_vx="<<companion2_person_vx<<endl;

				getline(reader,line);
				i = line.find(" ");
				k=line.size();
				cout <<"; i="<<i<<"; K="<<k<<endl;
				 if (i!=reader.eof()) {
					string cstr = line.substr(i,k);
					companion2_person_vy = std::stod(cstr.c_str());

				 } else {
					cout <<"ERROR @ GMap::read num_people_mov"<<endl;
					exit(EXIT_FAILURE);
				}
				cout <<"companion2_person_vy="<<companion2_person_vy<<endl;

				getline(reader,line); // companion_person_theta(1,1)= -nan

				//# Read companion person2 destinations #
				getline(reader,line); // companion_person_dest_x(1,1)= 0.220653

				i = line.find(" ");
				k=line.size();
				if(k>1000){
					k=39;
				}
				cout <<"; i="<<i<<"; K="<<k<<endl;
				 if (i!=reader.eof()) {
					string cstr = line.substr(i,k);
					companion_person2_dest_x = std::stod(cstr.c_str());

				 } else {
					cout <<"ERROR @ GMap::read num_people_mov"<<endl;
					exit(EXIT_FAILURE);
				}
				cout <<"companion_person2_dest_x="<<companion_person2_dest_x<<endl;

				getline(reader,line); // companion_person_dest_x(1,1)= 0.220653

				i = line.find(" ");
				k=line.size();
				cout <<"; i="<<i<<"; K="<<k<<endl;
				if(k>1000){
					k=39;
				}
				 if (i!=reader.eof()) {
					string cstr = line.substr(i,k);
					companion_person2_dest_y = std::stod(cstr.c_str());

				 } else {
					cout <<"ERROR @ GMap::read num_people_mov"<<endl;
					exit(EXIT_FAILURE);
				}
				cout <<"companion_person2_dest_y="<<companion_person2_dest_y<<endl;


			getline(reader,line); // d_r_1p_x(1,1)= 0.242083
			getline(reader,line); // d_r_1p_y(1,1)= 1.33599
			getline(reader,line); // d_r_1p_m(1,1)= 1.35775
			getline(reader,line); // d_r_1p_x(1,1)= 0.177969
			getline(reader,line); // d_r_1p_y(1,1)= 2.56626
			getline(reader,line); // d_r_2p_m(1,1)= 2.57242
			getline(reader,line); // d_2p_1p_x(1,1)= 0.420052
			getline(reader,line); // d_2p_1p_y(1,1)= 1.23027
			getline(reader,line); // d_2p_1p_m(1,1)= 1.3
			getline(reader,line); // theta_r_p1(1,1)= -79.7698
			getline(reader,line); // theta_r_p2(1,1)= -94.0148
			getline(reader,line); // theta_p1_p2(1,1)= -108.907


			getline(reader,line); // robot_x(1,1)= 0.462736

			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				robot_x = std::stod(cstr.c_str());
			} else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_x="<<robot_x<<endl;

			getline(reader,line); // robot_y(1,1)= -1.33152

			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				robot_y = std::stod(cstr.c_str());
			} else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_y="<<robot_y<<endl;

			getline(reader,line); //robot_vx(1,1)= 0

			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				robot_vx = std::stod(cstr.c_str());
			} else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_vx="<<robot_vx<<endl;

			getline(reader,line); //robot_vy(1,1)= 0

			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				robot_vy = std::stod(cstr.c_str());
			} else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_vy="<<robot_vy<<endl;

			getline(reader,line); //robot_theta(1,1)= 0.0397331

			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				robot_theta = std::stod(cstr.c_str());
			} else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_theta="<<robot_theta<<endl;

			getline(reader,line); //robot_v(1,1)= 0

			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				robot_v = std::stod(cstr.c_str());
			} else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_v="<<robot_v<<endl;

			getline(reader,line); //robot_w(1,1)= 0

			i = line.find(" ");
			k=line.size();
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (i!=reader.eof()) { // NO la uso y da problemas
				string cstr = line.substr(i,k);
				robot_w = std::stod(cstr.c_str());
			} else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_w="<<robot_w<<endl;

			// Get robot destination
			getline(reader,line); //robot_final_goal_x(1,1)= 0.462736

			i = line.find(" ");
			if(i>1000){
				i=27;
			}
			k=line.size();
			if((k>1000)||(k==0)){
				k=33;
			}
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (i!=reader.eof()) {
				string cstr = line.substr(i,k);
				robot_dest_x = std::stod(cstr.c_str());
			} else {
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_dest_x="<<robot_dest_x<<endl;

			getline(reader,line); //robot_final_goal_y(1,1)= -1.33152

			i = line.find(" ");
			if(i>1000){
				//i=27;
				getline(reader,line); //robot_final_goal_y(1,1)= -1.33152
			}
			k=line.size();
			if((k>1000)||(k==0)){
				//k=33;
				//getline(reader,line); //robot_final_goal_y(1,1)= -1.33152
			}
			cout <<"; i="<<i<<"; K="<<k<<endl;
			if (k!=reader.eof()) {
				string cstr = line.substr(i,k);
				robot_dest_y = std::stod(cstr.c_str());
			} else {
				string cstr = line.substr(i,k-1);  //NO LAS USO y dan problemas al coger los datos
				robot_dest_y = std::stod(cstr.c_str());
				cout <<"ERROR @ GMap::read num_people_mov"<<endl;
				exit(EXIT_FAILURE);
			}
			cout <<"robot_dest_y="<<robot_dest_y<<endl;

			getline(reader,line); // f_companion_x(1,1)= 0
			getline(reader,line); // f_companion_y(1,1)= 0
			getline(reader,line); // f_goal_x(1,1)= 0
			getline(reader,line); // f_goal_y(1,1)= 0
			getline(reader,line); // f_companion_x_mult(1,1)= 0
			getline(reader,line); // f_companion_y_mult(1,1)= 0
			getline(reader,line); // f_goal_x_mult(1,1)= 0
			getline(reader,line); // f_goal_y_mult(1,1)= 0
			getline(reader,line); // f_total_x(1,1)= 0.0243421
			getline(reader,line); // f_total_y(1,1)= -0.140267
			getline(reader,line); // f_pers_x(1,1)= 0.0243421
			getline(reader,line); // f_pers_y(1,1)= -0.140267
			getline(reader,line); // f_obs_x(1,1)= 0
			getline(reader,line); // f_obs_y(1,1)= 0
			getline(reader,line); // final_pose_robot_x(1,1)= 0
			getline(reader,line);  // final_pose_robot_y(1,1)= 0
			getline(reader,line); // final_pose_robot_v(1,1)= 0
			getline(reader,line); // final_pose_robot_w(1,1)= 0
			getline(reader,line); // final_pose_robot_theta(1,1)= 0
			getline(reader,line); //robot_performance(1,1)= nan

	}
	//reader.close();

	State2D Self;
// TODO: guardar id, personas en los proximos archivos de datos!!! IMPORTANTE!!!!

	Spose pose_of_the_robot=Spose( robot_x , robot_y , real_computation_time_dt , robot_theta , robot_v, robot_w);//robot_->get_current_pose();
	SpointV point_of_the_robot=SpointV( robot_x , robot_y, real_computation_time_dt, robot_vx, robot_vy ); //robot_->get_current_pointV();
	Self.Init(pose_of_the_robot.x,pose_of_the_robot.y,point_of_the_robot.vx,point_of_the_robot.vy,pose_of_the_robot.theta);

	std::cout <<"(Performance) Self.r.m="<<Self.r.m<< "; Self.r.x="<<Self.r.x<<"; Self.r.y="<<Self.r.y<<"; Self.r.th"<<Self.r.th*180/3.14<<"; pose_of_the_robot.theta*180/3.14="<<pose_of_the_robot.theta*180/3.14<<std::endl;
	std::cout <<"(Performance) Self.v.m="<<Self.v.m<< "; Self.v.x="<<Self.v.x<<"; Self.v.y="<<Self.v.y<<"; Self.v.th"<<Self.v.th*180/3.14<<std::endl;
	//Self.r.th=pose_of_the_robot.theta;

	unsigned int number_of_group_people_=2;
	State2D *Others=new State2D[number_of_group_people_];
	//std::cout << " *Others=new State2D[number_of_group_people_]"<<std::endl;
	SpointV_cov actual_first_person_companion_point=SpointV(companion_person_x,companion_person_y,real_computation_time_dt,companion_person_vx,companion_person_vy);//pointer_to_person_companion_->get_current_pointV();
	State2D first_companion_person(actual_first_person_companion_point.x,actual_first_person_companion_point.y,actual_first_person_companion_point.vx,actual_first_person_companion_point.vy);
	//first_companion_person.r.th=pose_of_the_robot.theta;
	Others[0]=first_companion_person;
	std::cout <<"(FIRST) id_person="<< " first_companion_person.r.x="<<first_companion_person.r.x<<"; first_companion_person.r.y="<<first_companion_person.r.y<<"; first_companion_person.v.x="<<first_companion_person.v.x<<"; first_companion_person.v.y="<<first_companion_person.v.y<<"; th="<<first_companion_person.r.th*180/3.14<<std::endl;


	SpointV_cov actual_second_person_companion_point=SpointV_cov(companion2_person_x,companion2_person_y,real_computation_time_dt,companion2_person_vx,companion2_person_vy);//=second_group_companion_person_obj_->get_prediction_trajectory()->at(parent_vertex);
	//if((we_have_pointer_to_second_person_)&&(number_of_group_people_>1)){
		//actual_second_person_companion_point=//second_group_companion_person_obj_->get_current_pointV();
		State2D second_companion_person(actual_second_person_companion_point.x,actual_second_person_companion_point.y,actual_second_person_companion_point.vx,actual_second_person_companion_point.vy);
		//second_companion_person.r.th=pose_of_the_robot.theta;

		Others[1]=second_companion_person;
		std::cout <<"(SECOND) id_person="<< " second_companion_person.r.x="<<second_companion_person.r.x<<"; second_companion_person.r.y="<<second_companion_person.r.y<<"; second_companion_person.v.x="<<second_companion_person.v.x<<"; second_companion_person.v.y="<<second_companion_person.v.y<<"; th="<<second_companion_person.r.th*180/3.14<<std::endl;
	//}
	Vector2D preferred;
	double x_orient_goal,y_orient_goal,orient_goal;
			//if((number_of_group_people_>1)&&(we_have_pointer_to_second_person_)){
				x_orient_goal=companion_person_dest_x - companion_person_x;
				y_orient_goal=companion_person_dest_y - companion_person_y;
				orient_goal=atan(y_orient_goal/x_orient_goal);
				if(isnan((float)orient_goal)){ // si es nan el calculo, quetate con la orientacion actual del robot. (caso inicio generalmente)
					orient_goal=robot_theta;
				}
				//preferred=Vector2D(x_orient_goal,y_orient_goal,orient_goal); // is the orientation until the goal of the group.
				if((companion_person_vx!=0)&&(companion2_person_vx!=0)&&(companion_person_vy!=0)&&(companion2_person_vy!=0)){
					preferred=Vector2D((companion_person_vx+companion2_person_vx)/2,(companion_person_vy+companion2_person_vy)/2);
				}else{
					x_orient_goal=companion_person_dest_x - companion_person_x;
					y_orient_goal=companion_person_dest_y - companion_person_y;
					orient_goal=atan(y_orient_goal/x_orient_goal);
					if(isnan((float)orient_goal)){ // si es nan el calculo, quetate con la orientacion actual del robot. (caso inicio generalmente)
						orient_goal=robot_theta;
					}
					preferred=Vector2D(x_orient_goal,y_orient_goal,orient_goal); // is the orientation until the goal of the group.
				}

				std::cout << "(caso1 save in file) preferred.x= "<<preferred.x<<"; preferred.y="<<preferred.y<<"; preferred.m="<<preferred.m<<"; preferred.th="<<preferred.th<< std::endl;
							//}
				// TODO: mirar que esten seteadas todas las constantes por defecto.
	 double vp=Companion_Zanlungo_Model_.f2_vp2v(preferred.m);//f2_vp2v(0.4);

	 preferred.Scale(vp/preferred.m);
	// vp=robot_->get_current_pose().v;
	 //preferred=Vector2D(0,vp);//la velocidad preferida
	 std::cout << "(caso2 save in file. ahora uso caso 1) preferred.x= "<<preferred.x<<"; preferred.y="<<preferred.y<<"; preferred.m="<<preferred.m<<"; preferred.th="<<preferred.th*180/3.14<<"; vp="<<vp<< std::endl;

	 double actual_performance=Companion_Zanlungo_Model_.Exmet3(Self,Others,preferred);

	 std::cout << "actual_performance="<<actual_performance<< std::endl;

	 	 if(actual_performance!=0){ //saltatela. Pasan por como guarde los datos, no porque de verdad sea 0, es cuando se paraba la simulacion.
	 		actual_robot_performance_.push_back(actual_performance); // performace por experimento; iteration.
	 	 }



	// }
	firs_experiment=experiment_;

	// Calcular media:
	double sum=0.0;
	for(unsigned int t=0; t<actual_robot_performance_.size();t++){

		if(isnan((float)actual_robot_performance_.at(t))){
		}else{
			sum=sum+actual_robot_performance_.at(t);
			std::cout << "sum="<<sum<<"; actual_robot_performance_="<<actual_robot_performance_.at(t)<< std::endl;
		}

	}
	double media_one_experiment=sum;// /actual_robot_performance_.size();
	std::cout << "media_one_experiment="<<media_one_experiment<<"; sum/actual_robot_performance_.size()="<<sum/actual_robot_performance_.size()<< std::endl;

	if(isnan((float)media_one_experiment)){

	}else{

	}

	// calcular std: (TODO: calcularla!!!)
	double ste=0.0;
	for(unsigned int t=0; t<actual_robot_performance_.size();t++){
		if(isnan((float)actual_robot_performance_.at(t))){
		}else{
			ste=ste+(actual_robot_performance_.at(t)-media_one_experiment)*(actual_robot_performance_.at(t)-media_one_experiment);
		}

	}
	double final_std=ste; // /(actual_robot_performance_.size());
	std::cout << "final_std="<<final_std<< std::endl;

	if((isnan((float)media_one_experiment))||(isnan((float)final_std))){ // evitar casos raros que no quite de los datos, donde se paraba la simulacion.

	}else{
		if((media_one_experiment!=0)){
			all_means_of_performances_ex_.push_back(media_one_experiment);
			all_std_of_performances_ex_.push_back(final_std);
		}

	}


	 actual_robot_performance_.clear();
	 iteration_++;
	}

	double sum2=0.0;
	for(unsigned int t=0; t<all_means_of_performances_ex_.size();t++){
		sum2=sum2+all_means_of_performances_ex_.at(t);
		std::cout << "sum2="<<sum2<<"; all_means_of_performances_ex_="<<all_means_of_performances_ex_.at(t)<< std::endl;
	}
	double media_all_experiment=sum2/all_means_of_performances_ex_.size();

	// calcular std: (TODO: calcularla!!!)
	double sum3=0.0;
	for(unsigned int t=0; t<all_std_of_performances_ex_.size();t++){
		if((all_std_of_performances_ex_.at(t)<1)&&(all_std_of_performances_ex_.at(t)>0)){
			sum3=sum3+all_std_of_performances_ex_.at(t);
		}

		std::cout << "sum3="<<sum3<<"; all_std_of_performances_ex_="<<all_std_of_performances_ex_.at(t)<< std::endl;
	}
	double media_all_std=sum3/all_std_of_performances_ex_.size();

	/// INI TO calculate real exp std:
	double ste2=0;
	for(unsigned int t=0; t<all_means_of_performances_ex_.size();t++){
			if(isnan((float)all_means_of_performances_ex_.at(t))){
			}else{
				ste2=ste2+(all_means_of_performances_ex_.at(t)-media_all_experiment)*(all_means_of_performances_ex_.at(t)-media_all_experiment);
				 std::cout << "ste2="<<ste2<< std::endl;
			}

		}
	media_all_std=ste2/all_means_of_performances_ex_.size();
	/// FIN TO calculate real exp std:

	 std::cout << "media_all_experiment="<<media_all_experiment<<"; media_all_std="<<media_all_std<< std::endl;





/*
	std::string  data_file_Zanlungo_="/home/erepiso/iri-lab/iri_ws/src/iri_navigation/iri_atr_akp_local_planner_companion/txt_data/data_companion.txt";





	std::ofstream fileMatlab;
	fileMatlab.open (data_file_Zanlungo_.c_str(), std::ofstream::out | std::ofstream::app);
	// const char *direc_file_name_;
	//direc_file_name_= results_file_.c_str();
	//home/ely7787/iri-lab/labrobotica/restricted/algorithms/people_prediction/branches/ely_people_prediction_companion_robot/1_data_results
	//fileMatlab.open (results_file_.c_str(), std::ofstream::out | std::ofstream::app);
	//fileMatlab.open (data_file_Zanlungo_.c_str(), std::ofstream::out | std::ofstream::trunc);
	fileMatlab << "\n";

	fileMatlab << "robot_performance("<<experiment_<<","<<iteration_<<")= "<<actual_robot_performance_<<"\n";
*/
	return 0;
}
