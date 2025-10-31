/*
 * akp_local_planner_alg_node.h
 *
 *  Created on: Dec 22, 2013 by Gonzalo Ferrer. 
 *  Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration)
 *      Author: Initial code of the Robot alone AKP navigation is from Gonzalo Ferrer (2013).
 *      Author: Additions and new codes of ASPSI for people accompaniment code of Ely Repiso (from 2015 and currently).
 *
 *      License (for other authors that will not be the original ones): CC BY-NC-ND 4.0 
 *              (Attribution-NonCommercial-NoDerivatives 4.0 International)
 *               https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
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

#ifndef _akp_local_planner_alg_node_h_
#define _akp_local_planner_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "akp_local_planner_alg.h"

#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/server.h>
#include <iri_robot_aspsi/AkpLocalPlannerConfig.h>
 
// [publisher subscriber headers]
#include <iri_nav_msgs/companionStateResults.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <iri_perception_msgs/detectionArray.h>
#include <visualization_msgs/MarkerArray.h>


#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
//#include <tf/transform_broadcaster.h> // robot companion (ely) trasladar modelo tibi robot companion al marcker donde deberia estar el robot.

#include <iri_nav_msgs/restartSim.h>

#include <random>
#include <iostream>
#include <fstream>      // std::ofstream
#include <string>
#include <stdio.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
 #include <tf2/impl/utils.h>

using namespace std;

// #include <sensor_msgs/Joy.h> // [Wii and PS3 comandaments]
// [Wii and PS3 comandaments]
// #include <iri_common_drivers_msgs/ps3state.h>
// #define BUTTON_CANCEL_GOAL         iri_common_drivers_msgs::ps3state::PS3_BUTTON_SELECT
// #define BUTTON_DEAD_MAN              iri_common_drivers_msgs::ps3state::PS3_BUTTON_REAR_RIGHT_1     //R1
// #define BUTTON_TRANS_SPEED_UP        iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_TRIANGLE
// #define BUTTON_TRANS_SPEED_DOWN      iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_CROSS
// #define BUTTON_ROT_SPEED_UP          iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_CIRCLE
// #define BUTTON_ROT_SPEED_DOWN        iri_common_drivers_msgs::ps3state::PS3_BUTTON_ACTION_SQUARE
// #define AXIS_TRANS_FORWARD           iri_common_drivers_msgs::ps3state::PS3_AXIS_STICK_LEFT_UPWARDS    //L3
// #define AXIS_ROT_LEFTWARD            iri_common_drivers_msgs::ps3state::PS3_AXIS_STICK_RIGHT_LEFTWARDS //R3
// #define NEAR_PERS_COMP               iri_common_drivers_msgs::ps3state::PS3_BUTTON_CROSS_LEFT
// #define LAUNCH_GOAL     	     iri_common_drivers_msgs::ps3state::PS3_BUTTON_CROSS_UP
// #define RESET_DATA_FILE     	     iri_common_drivers_msgs::ps3state::PS3_BUTTON_START
// #define NEAR_PERS_COMP_ONEP  	     iri_common_drivers_msgs::ps3state::PS3_BUTTON_CROSS_RIGHT
// #define NEAR_PERS_COMP_ONEP2  	     iri_common_drivers_msgs::ps3state::PS3_BUTTON_CROSS_RIGHT
// #define STOP_ROBOT_MANUAL 	     iri_common_drivers_msgs::ps3state::PS3_BUTTON_REAR_LEFT_1
//#define LAUNCH_GOAL2     	     iri_common_drivers_msgs::ps3state::PS3_BUTTON_CROSS_DOWN
// [publisher subscriber headers]

////////////////


// [service client headers]
#include <iri_nav_msgs/InitialiceSim.h>
#include <move_base/move_base.h>

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <tf/transform_broadcaster.h>

// [service client headers]


// [libraries]
#include "nav/plan_local_nav.h"

#include <geometry_msgs/Twist.h>

typedef enum {
  HSQ_INIT,
  HSQ_IT,
  HSQ_STOP,
  HSQ_RECEIVE_OBS,
  HSQ_SEND_GOAL
} states;



/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class AkpLocalPlanner : public nav_core::BaseLocalPlanner
{
  enum Goal_providing_mode { Crop_local_window=0, Slicing_global};
  private:


   std::string frame_map_;
   std::string frame_robot_footprint_;

    // [publisher attributes]
    ros::Publisher cost_params_publisher_;
    std_msgs::Float64MultiArray Float64_msg_;   
    ros::Publisher goal_publisher_;
    ros::Publisher cmd_vel_publisher_;
    geometry_msgs::Twist cmd_vel_msg_;

    ros::Publisher tibi_pose_for_sim_; // publisher to restart robot pose in simulation
    geometry_msgs::PoseWithCovarianceStamped tibi_restart_pose_;

    ros::Publisher companionState_publisher_; //publisher of planner state, for the companion state to do that tibi talks.
    iri_nav_msgs::companionStateResults companionState_companionStateResults_msg_; // was to obtain som states from severan situations and do a authomatic robot's speech for human-robot interaction. Do not needed for only accompaniment.

    // [publisher attributes]
    ros::Publisher g_plan_pub_;
    ros::Publisher l_plan_pub_;

    pthread_mutex_t planner_mutex_;
    void planner_mutex_enter(void);
    void planner_mutex_exit(void);


    // markers for planner
    ros::Publisher markers_publisher_;
    ros::Publisher markers_publisher_m2_;
    ros::Publisher markers_publisher_comp_markers_;
    ros::Publisher markers_publisher_people_prediction_time_;


    visualization_msgs::MarkerArray MarkerArray_msg_;
    visualization_msgs::MarkerArray MarkerArray_msg_comp_markers_;
    visualization_msgs::MarkerArray MarkerArray_msg_m2_;
    visualization_msgs::MarkerArray MarkerArray_msg_people_prediction_time_;

    visualization_msgs::Marker pred_traj_marker_, 
      pred_traj2d_marker_,
      cylinder_marker_,
      planning_marker_,
      robot_goal_marker_,
      robot_subgoal_marker_,
      robot_marker_,
      workspace_marker_,
      planning_goals_marker_,
      best_path_marker_,
      best_path2d_marker_,
      laser_obstacle_marker_,
      force_marker_,
      force_goal_marker_,
      force_int_person_marker_,
      force_obstacle_marker_,
      force_int_robot_marker_,
      text_marker_,
      nd_path_marker_,
      nd_path2d_marker_,
      person_companion_marker_,
      robot_companion_marker_,
      center_companion_marker_,
      robot_see_goal_marker_,
      person_90_degre_marker_,
      force_companion_marker_,
      force_companion_goal_marker_,
      best_next_pose_robot_companion_markers_,
      robot_goal_marker2_,
      robot_goal_markerASPSI_,
      robot_goal_marker3_,
	  next_goal_marker_,
      robot_goal_to_position_respect_to_the_person,
      robot_goal_to_follow_the_path,
      final_robot_goal_act_iteration,
      pred_traj_point_marker_,
      medium_point_face_person_,
      see_before_next_goal_of_robot_,
      rand_goal_comp_p1_marker_,
      rand_goal_comp_p2_marker_,
      centre_group_for_goals_marker_;

    nav_msgs::OccupancyGrid map_; //map received from gmapping
    int num_map_cells_=0; //number of map cells (height*width)
    bool map_received=false;
    bool out_of_map_;
    int max_value_map_obstacle_for_goal_;
    bool isValidPoint(const geometry_msgs::Point & point);
    bool isValidPoint_lessExigent(const geometry_msgs::Point & point);

    int point2cell(const geometry_msgs::Point & point);
    geometry_msgs::Point cell2point(const int & cell);
    void getStraightPoints(int point, int points[]);
    void getAdjacentPoints(int point, int points[]);
    int left(int point);
    int upleft(int point);
    int up(int point);
    int upright(int point);
    int right(int point);
    int downright(int point);
    int down(int point);
    int downleft(int point);
    int floor0(float value);


    std::string in_robot_name_;

    // [subscriber attributes]
    ros::Subscriber costmap_subscriber_;
    void costmap_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg); //subscriber to the gmapping
    ros::Subscriber fscan_subscriber_;
    void fscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t fscan_mutex_;
    void fscan_mutex_enter(void);
    void fscan_mutex_exit(void);
    sensor_msgs::LaserScan fscan;
    bool fscan_received;



    ros::Subscriber rscan_subscriber_;
    void rscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    pthread_mutex_t rscan_mutex_;
    void rscan_mutex_enter(void);
    void rscan_mutex_exit(void);
    sensor_msgs::LaserScan rscan;
    bool rscan_received;

    ros::Subscriber odom_subscriber_;
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    pthread_mutex_t odom_mutex_;
    void odom_mutex_enter(void);
    void odom_mutex_exit(void);

    // todo: new callback for odom_person
   // ros::Subscriber odom_person_subscriber_;
   // void odom_person_callback(const human_msgs::TrackedHumanArray::ConstPtr& msg);
   // pthread_mutex_t odom_person_mutex_;
//void odom_person_mutex_enter(void);
 //void odom_person_mutex_exit(void);

    ros::Subscriber tracks_subscriber_;
    void tracks_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg);
    pthread_mutex_t tracks_mutex_;
    void tracks_mutex_enter(void);
    void tracks_mutex_exit(void);

    ros::Subscriber params_values_subscriber_;
    void params_values_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    pthread_mutex_t params_values_mutex_;
    void params_values_mutex_enter(void);
    void params_values_mutex_exit(void);


    // [service attributes]

    // [client attributes]
    ros::ServiceClient init_simulations_client_;
    ros::ServiceClient get_plan_client_;

    // Publish the status of the companion. To tibi talk.
    ros::Publisher tibi_akp_status_publisher_;
    std_msgs::UInt64 tibi_akp_status_UInt64_msg_;

    iri_nav_msgs::InitialiceSim init_simulations_srv_;

    nav_msgs::GetPlan get_plan_srv_;

    // [action server attributes]
    // [action client attributes]

    // [library attributes]
    Cplan_local_nav planner_;
    Cplan_local_nav::plan_mode plan_mode_;
    Spose  robot_pose_;
    Spose  robot_pose_from_odom_;
    SdetectionObservation companion_pose_; //Spose  companion_pose_;
 
    double we_do_not_want_stop_slowly_; //to disable thhe stop slowly of the robot. Just in case if it interfers.
    boost::array<double, 36UL> cov_companion;
    Spoint robot_goal_;
    Spose best_next_pose_companion_markers_;
    std::vector<Spoint> laser_points;
    std::vector<SdetectionObservation> obs;
    std::vector<SdetectionObservation> obs_temp; 
    std::vector<SdetectionObservation> obs2;
    std::string force_map_path_, destination_map_path_;
    std::string robot_;
    std::string fixed_frame;
    std::string robot_frame;
    std::string companion_frame;
    costmap_2d::Costmap2DROS* costmap_ros_;
 
    // tf::TransformListener* tf_;
    tf2_ros::Buffer* tf_;
    tf2_ros::Buffer tf2_buffer; // for version in melodic
    // tf::TransformListener tf_listener_;
    tf2_ros::TransformListener tf2_listener; // for version in melodic

    bool initialized_;
    bool initialized2_;
    bool setup_;
    void init();
    std::vector<geometry_msgs::PoseStamped> global_plan_,sliced_global_plan_;//global coordinates
    std::vector<geometry_msgs::PoseStamped> local_plan_;//local coordinates
    double xy_goal_tolerance, v_goal_tolerance;

     bool transformGlobalPlan(const tf2_ros::TransformListener& tf, 
                              const std::vector<geometry_msgs::PoseStamped>& global_plan, 
                              const costmap_2d::Costmap2DROS& costmap, 
                              const std::string& global_frame, 
                              std::vector<geometry_msgs::PoseStamped>& transformed_plan);

     bool transformPose(const tf2_ros::TransformListener& tf, 
                                      const geometry_msgs::PoseStamped& plan_pose, 
                                      const costmap_2d::Costmap2DROS& costmap, 
                                      const std::string& target_frame, 
                                      geometry_msgs::PoseStamped& transformed_pose);

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

    std::vector<Spoint> scan2points(const sensor_msgs::LaserScan scan);

    dynamic_reconfigure::Server<iri_robot_aspsi::AkpLocalPlannerConfig> *dsrv_;
    iri_robot_aspsi::AkpLocalPlannerConfig default_config_;
   
 

    void init_force_planner_and_markers();
    void fill_scene_markers();
    void fill_best_path_2d();
    void fill_forces_markers();
    void fill_laser_obstacles();
    void fill_planning_markers_2d();
    void fill_planning_markers_3d();
    void fill_planning_markers_3d_companion();
    void fill_people_prediction_markers_2d();
    void fill_people_prediction_markers_2d_companion();
    void fill_people_prediction_markers_3d();
    void fill_people_prediction_markers_3d_companion();
    void fill_my_covariance_marker( visualization_msgs::Marker& marker, const SpointV_cov& point ,    unsigned int track_id);
    void fill_robot_companion_markers();
    void fill_test_markers();
    void fill_people_prediction_markers_path_points();
    void fill_my_prediction_points( visualization_msgs::Marker& marker, const SpointV_cov& point );

    void fill_people_companion_see_normal_and_chancges();

    void fill_markers_side_by_side_2people(); // markers to see if the side-by-side with two people is correct.

    unsigned int vis_mode_;
    bool frozen_mode_, move_base;
    unsigned int text_markers_old_size_;
    
    Goal_providing_mode goal_providing_mode_;
    void slice_plan();
    
    double slicing_path_diff_orientation_;
    
    std::deque<double> velocities_;

    // companion variables (ely)
    geometry_msgs::PointStamped header_point_;
    std::string target_frame_id;
    std::string source_frame_id;
 
    bool check_pers_comp_;
    bool check_isValidGoal_;
    bool change_goal_to_valid_goal_;
    int increment_check_arround_goals_;

    unsigned int id_person_companion_;
    unsigned int id_SECOND_person_companion_;
  
		double before_odom_time_;
    double START_time_secs_fill_markers;
 		bool simulation_; // true if we have on simulation mode. False if we have in real robot mode.

    //send a goal variables (ely) // [action client attributes]
    move_base_msgs::MoveBaseGoal move_base_goal_;
    move_base_msgs::MoveBaseGoal before_good_move_base_goal_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

    Spoint actual_goal;
    bool isMoveBaseActive;
    // change initial position, variables.
    double robot_ini_pose_x_;
    double robot_ini_pose_y_;
    double robot_ini_pose_theta_;
    bool test_with_2people_but_tibi_really_only_with_me_;

  	Cperson_abstract::companion_reactive reactive_;

    
    bool debug_antes_subgoals_entre_AKP_goals_;
    //tf::TransformListener* tf_listener2_;
    tf2_ros::TransformListener* tf_listener2_;
    // ely functions to send a goal
    
		bool get_scout_results_doGoal_;
    bool doGoal(); 
    bool doGoal2(); 
    Spoint checkIfGoalIsValidPoint(Spoint in_goal_to_check_if_valid);
    bool move_baseMakeActionRequest();
    void move_baseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void move_baseActive();
    void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
    

    bool external_goal_; // boolean to do a external goal from rviz or not.
    bool debug_real_test_companion_; 
    bool debug_real_test_companion_robot_; 
    bool we_have_companion_person_;
    int iter;

 
    bool check_execution_times_;
    int iterator_facke_;

		bool flag_play_change_id_;

		bool we_have_p2_in_obs_;

		bool now_we_do_not_have_p2_;

		unsigned int num_people_node_;
	
		unsigned int before_id_SECOND_person_companion_;
		double before_people_time_,actual_people_time_;	

		double x_velodine_, y_velodine_;

		bool change_ids_group_people_in_node_;
	
		double speed_k_;

		/* obtain near persons with ps3 button */
		bool human_is_alive_;

    double vt_max; // estas en teoria se eliminaran o las cambiare por las que a mi me hacen falta, pq son variables para teleop.
    double vr_max;
    double trans_speed_scale;
    double rot_speed_scale;
    bool cancel_goal;

    bool use_default_wii_button_;
    bool use_default_PS3_button_;
   	// for wii and PS3 comandaments (to get people id's, launch goal. Etc from the PS3 commandement. Do not need it for only accompaniment.)
    // [subscriber attributes]
    //ros::Subscriber joy_subscriber_;
    //void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
   //pthread_mutex_t joy_mutex_;
    //void joy_mutex_enter(void);
   // void joy_mutex_exit(void);
    
    //void useWii(std::vector<int> current_buttons, std::vector<int> last_buttons);
    //void useWiiButton(const unsigned int & index);
   // void usePs3(std::vector<int> current_buttons, std::vector<int> last_buttons, std::vector<float> current_axes);
   // void usePs3Button(const unsigned int & index);
   // void function_joy_teleop_mainNodeThread(void);
	//	bool change_ps3_config_;
 		double ros_time_to_sec_general_before_iter_;
		double real_companion_angle_SideBySide_; // companion_angle_zanlungo_,
		bool stop_robot_manually_;
		bool to_stop_we_have_p1_,to_stop_we_have_p2_;
		double step_decreace_velocity_stop_robot_,limit_velocity_stop_robot_;
		unsigned int planner_companion_status_; // status=1 (No P1 ni P2) ; status=2 (No P2)
	 	// the planner_companion_status_ is to communicate with the teleop, to do a Tibi speech  in these case.

		bool no_status_;
		double meters_to_return_;  // de momento 2.0 metros, pero se puede cambiar desde el reconfigure.
		bool first_retun_;
		bool debug_sideBySide2019_;
		bool in_see_cout_for_testdemo4_abril_ros_;
		bool see_std_out_mesages_ros_,see_std_out_velocities_ros_;//,fake_person_lateral_;
		std::vector<double> params_side_by_side_central_;
		std::vector<double> params_side_by_side_lateral_or_one_person_;
		bool first_iter_for_update_scene_;

		/**
    * \brief Resets joy_watchdog time
    */
   // void reset_joy_watchdog(void);
    /**
    * \brief Returns true if joy_watchdog timeouts
    */
  //  bool joy_watchdog_active(void);
    /**
    * \brief Updates joy_watchdog time
    */
   /// void update_joy_watchdog(void);
    /**
    * \brief Watchdog timeout duration
    */
   // ros::Duration joy_watchdog_duration;
    /**
    * \brief Watchdog access mutex
    */
   // CMutex joy_watchdog_access;



   /**
    * \brief config variable
    *
    * This variable has all the parameters defined in the cfg config file.
    * Is updated everytime function node_config_update() is called.
    */
    iri_robot_aspsi::AkpLocalPlannerConfig config_;
    
    // variables to stop the node to evaluate the costs:
    states current_state; // variable indicating the current state of the node.
    ros::Subscriber cmd_vel_stop_subscriber_;
    pthread_mutex_t cmd_vel_stop_mutex_;
    void cmd_vel_stop_mutex_enter(void);
    void cmd_vel_stop_mutex_exit(void);
    double debug_stop_node_to_evaluate_costs_;
    bool fuera_bolitas_goals_companion_markers_;
    // variables for goal to person goal.
    // id person goal to go the group.
		unsigned int id_person_goal_;


    Cplan_local_nav::action_mode Action_ROS_;
    Cplan_local_nav::simulation_case Actual_case_ROS_;

    double in_set_planner_dt_;

		double before_tim;

		double v_max_due_to_people_companion_;  //este limite lo hago en el nodo directamente, ya que es más restrictivo que el que hago interno de la V_max seada como valor maximo.
		SpointV person_companion_position_, second_person_companion_position_;
	
		double dist_betw_rob_and_comp_people_to_slow_velocity_;

		double ros_max_real_speed_out_;
		double ros_max_real_angular_speed_out_;

		double number_of_people_in_group_;
		double  num_people_for_state_;

		bool output_screen_messages_;




		bool debug_all_IVO_;
		bool debug_few_IVO_;
	  unsigned int max_obs_to_consider_narrow_passage_;

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    AkpLocalPlanner(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~AkpLocalPlanner(void);
  /**
    * @brief  Constructs the ros wrapper
    * @param name The name to give this instance of the trajectory planner
    * @param tf A pointer to a transform listener
    * @param costmap The cost map to use for assigning costs to trajectories
    */
    AkpLocalPlanner(std::string name, tf2_ros::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros);

    /**
      * @brief  Constructs the ros wrapper
      * @param name The name to give this instance of the trajectory planner
      * @param tf A pointer to a transform listener
      * @param costmap The cost map to use for assigning costs to trajectories
      */
    void initialize(std::string name, tf2_ros::Buffer* tf,
        costmap_2d::Costmap2DROS* costmap_ros);

    /**
      * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
      * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
      * @return True if a valid trajectory was found, false otherwise
      */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
      * @brief  Set the plan that the controller is following
      * @param orig_global_plan The plan to pass to the controller
      * @return True if the plan was updated successfully, false otherwise
      */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
      * @brief  Check if the goal pose has been achieved
      * @return True if achieved, false otherwise
      */
    bool isGoalReached();

  protected:
   /**
    * \brief template algorithm class
    *
    * This template class refers to an implementation of an specific algorithm
    * interface. Will be used in the derivate class to define the common 
    * behaviour for all the different implementations from the same algorithm.
    */
    AkpLocalPlannerAlgorithm alg_;

   /**
    * \brief public node handle communication object
    *
    * This node handle is going to be used to create topics and services within
    * the node namespace. Additional node handles can be instantatied if 
    * additional namespaces are needed.
    */
    ros::NodeHandle public_node_handle_;

   /**
    * \brief private node handle object
    *
    * This private node handle will be used to define algorithm parameters into
    * the ROS parametre server. For communication pruposes please use the 
    * previously defined node_handle_ object.
    */
    ros::NodeHandle private_node_handle_;

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void reconfigureCallback(iri_robot_aspsi::AkpLocalPlannerConfig &config, uint32_t level);

    // [diagnostic functions]
    
    // [test functions]

// Help to remmember the numbers that match with each PS3 button for the speech interactions with tibi.
// #define PS3_BUTTON_SELECT            0
// #define PS3_BUTTON_STICK_LEFT        1
// #define PS3_BUTTON_STICK_RIGHT       2
// #define PS3_BUTTON_START             3
// #define PS3_BUTTON_CROSS_UP          4
// #define PS3_BUTTON_CROSS_RIGHT       5
// #define PS3_BUTTON_CROSS_DOWN        6
// #define PS3_BUTTON_CROSS_LEFT        7
// #define PS3_BUTTON_REAR_LEFT_2       8
// #define PS3_BUTTON_REAR_RIGHT_2      9
// #define PS3_BUTTON_REAR_LEFT_1       10
// #define PS3_BUTTON_REAR_RIGHT_1      11
// #define PS3_BUTTON_ACTION_TRIANGLE   12
// #define PS3_BUTTON_ACTION_CIRCLE     13
// #define PS3_BUTTON_ACTION_CROSS      14
// #define PS3_BUTTON_ACTION_SQUARE     15
// #define PS3_BUTTON_PAIRING           16
// 
// #define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
// #define PS3_AXIS_STICK_LEFT_UPWARDS      1
// #define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
// #define PS3_AXIS_STICK_RIGHT_UPWARDS     3
// #define PS3_AXIS_BUTTON_CROSS_UP         4
// #define PS3_AXIS_BUTTON_CROSS_RIGHT      5
// #define PS3_AXIS_BUTTON_CROSS_DOWN       6
// #define PS3_AXIS_BUTTON_CROSS_LEFT       7
// #define PS3_AXIS_BUTTON_REAR_LEFT_2      8
// #define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
// #define PS3_AXIS_BUTTON_REAR_LEFT_1      10
// #define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
// #define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
// #define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
// #define PS3_AXIS_BUTTON_ACTION_CROSS     14
// #define PS3_AXIS_BUTTON_ACTION_SQUARE    15
// #define PS3_AXIS_ACCELEROMETER_LEFT      16
// #define PS3_AXIS_ACCELEROMETER_FORWARD   17
// #define PS3_AXIS_ACCELEROMETER_UP        18
// #define PS3_AXIS_GYRO_YAW                19
// 
// #define BUTTON_DEAD_MAN              PS3_BUTTON_REAR_RIGHT_1
// #define BUTTON_TRANS_SPEED_UP        PS3_BUTTON_ACTION_TRIANGLE
// #define BUTTON_TRANS_SPEED_DOWN      PS3_BUTTON_ACTION_CROSS
// #define BUTTON_ROT_SPEED_UP          PS3_BUTTON_ACTION_CIRCLE
// #define BUTTON_ROT_SPEED_DOWN        PS3_BUTTON_ACTION_SQUARE
// #define AXIS_TRANS_FORWARD           PS3_AXIS_STICK_LEFT_UPWARDS
// #define AXIS_ROT_LEFTWARD            PS3_AXIS_STICK_RIGHT_LEFTWARDS

// para los botones de la wii:
// http://docs.ros.org/jade/api/wiimote/html/msg/State.html
//int8    INVALID       = -1
//float32 INVALID_FLOAT = -1.0

//int8 MSG_BTN_1     = 0
//int8 MSG_BTN_2     = 1
//int8 MSG_BTN_A     = 2
//int8 MSG_BTN_B     = 3
//int8 MSG_BTN_PLUS  = 4
//int8 MSG_BTN_MINUS = 5
//int8 MSG_BTN_LEFT  = 6
//int8 MSG_BTN_RIGHT = 7
//int8 MSG_BTN_UP    = 8
//int8 MSG_BTN_DOWN  = 9
//int8 MSG_BTN_HOME  = 10
//int8 MSG_BTN_Z     = 0
//int8 MSG_BTN_C     = 1
//int8 MSG_CLASSIC_BTN_X       = 0
//int8 MSG_CLASSIC_BTN_Y       = 1
//int8 MSG_CLASSIC_BTN_A       = 2
//int8 MSG_CLASSIC_BTN_B       = 3
//int8 MSG_CLASSIC_BTN_PLUS    = 4
//int8 MSG_CLASSIC_BTN_MINUS   = 5
//int8 MSG_CLASSIC_BTN_LEFT    = 6
//int8 MSG_CLASSIC_BTN_RIGHT   = 7
//int8 MSG_CLASSIC_BTN_UP      = 8
//int8 MSG_CLASSIC_BTN_DOWN    = 9
//int8 MSG_CLASSIC_BTN_HOME    = 10
//int8 MSG_CLASSIC_BTN_L       = 11
//int8 MSG_CLASSIC_BTN_R       = 12
//int8 MSG_CLASSIC_BTN_ZL      = 13
//int8 MSG_CLASSIC_BTN_ZR      = 14

};

#endif
