#include "tm_rrt.h"

template <typename T>
T get_ros_param(const std::shared_ptr<rclcpp::Node>& node, std::string param_name, T default_value) {
    T param_value;
    if (!node->get_parameter(param_name, param_value)) {
        RCLCPP_WARN(node->get_logger(), "WARNING: parameter %s not set, default value \"%s\" will be used", param_name.c_str(), std::to_string(default_value).c_str());
        return default_value;
    }
    return param_value;
}

int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    
    //get prameters from ROS
    std::string package_path = ament_index_cpp::get_package_share_directory("tm_rrt");
    auto tmRRT = std::make_shared<TM_RRTplanner>(package_path);

    //DOMAIN FILE IS NECESSARY
    std::string param_domain_file;
    if( !tmRRT->get_parameter<std::string>("/tm_rrt/domain_file", param_domain_file) ){
        std::cout<<ansi::red<<"ERROR: planning domain not specified"<<ansi::end<<std::endl;
        return 0;
    }

    //get debug value 
    bool param_debug = get_ros_param<bool>(tmRRT, "/tm_rrt/debug",false);
    //get planner type (simple, divided)
    std::string param_planner_type = get_ros_param<std::string>(tmRRT, "/tm_rrt/planner_type","simple");
    //get task selector (best = 0, uniform = 1, montecarlo = 2)
    int param_task_selector = get_ros_param<int>(tmRRT, "/tm_rrt/task_selector",0);
    //get number of runs for this domain
    int param_number_of_runs = get_ros_param<int>(tmRRT, "/tm_rrt/number_of_runs",30);
    //get timeout for the planner (in seconds)
    double param_planner_timeout = get_ros_param<double>(tmRRT, "/tm_rrt/planner_timeout",300);
    //get timeout for the BFS (in seconds)
    double param_bsf_timeout = get_ros_param<double>(tmRRT, "/tm_rrt/BFS_timeout",2);
    //get horizon of the rrt (in meters)
    double param_rrt_horizon = get_ros_param<double>(tmRRT, "/tm_rrt/RRT_horizon",5);
    //get TM-RRT parameters
    double param_w_b = get_ros_param<double>(tmRRT, "/tm_rrt/w_b",1.0);
    double param_w_t = get_ros_param<double>(tmRRT, "/tm_rrt/w_t",5.0);
    double param_path_len = get_ros_param<double>(tmRRT, "/tm_rrt/path_len",0.9);
    //go-to-goal probabilities
    double param_p_s = get_ros_param<double>(tmRRT, "/tm_rrt/p_s",0.3);
    double param_p_c = get_ros_param<double>(tmRRT, "/tm_rrt/p_c",0.3);

    //load symbolic doman and geometric map 
    tmRRT->update_obstacles_from_FILE();

    //ROS-based drawing and visualization of the map
    
    tmRRT->draw_map(tmRRT->S_init.pose, tmRRT->S_goal.pose);
        
    std::vector<Point3d> void_obs;
    tmRRT->draw_points(tmRRT->S_init.pose, tmRRT->compute_dynamic_obstacles(tmRRT->S_init, void_obs) );
    
    tmRRT->rviz_image_plan();

    int j = 1;

    //execute it several times
    for (auto i = 0; i < param_number_of_runs; i++) {

        tmRRT->plan.clear();
        tmRRT->draw_map();

        RCLCPP_INFO(tmRRT->get_logger(), "");
        RCLCPP_INFO(tmRRT->get_logger(), "TM_PLANNER: ROUND NUMBER %d", i + 1);
        RCLCPP_INFO(tmRRT->get_logger(), "");


        //set parameters
        tmRRT->w_b = param_w_b; //weight of the path (bottom) 
        tmRRT->w_t = param_w_t; //weight of the task (top)
        tmRRT->path_len = param_path_len; //max length of the path (default value)

        tmRRT->P_task_to_goal = param_p_s; //probability to sample the goal symbolic state
        tmRRT->P_go_to_goal = param_p_c; //probability to sample the goal configuration

        tmRRT->debug_on = param_debug;
        
        tmRRT->mode = TaskSelector(param_task_selector);

        if(param_planner_type == "tm_rrt") //TM-RRT
            tmRRT->plan_TM_RRT(tmRRT->S_init, tmRRT->S_goal, tmRRT->plan,
                               param_planner_timeout, param_rrt_horizon, tmRRT->path_len);
        else if(param_planner_type == "bfs_rrt") //BFS+RRT
            tmRRT->plan_BFS_RRT(tmRRT->S_init, tmRRT->S_goal, tmRRT->plan,
                                param_planner_timeout, param_rrt_horizon, tmRRT->path_len, param_bsf_timeout);
        else{
            // std::cout<<ansi::red<<"ERROR: planner of type "<<param_planner_type<<" does not exists"<<ansi::end<<std::endl;
            RCLCPP_ERROR(tmRRT->get_logger(), "ERROR: planner of type %s does not exists", param_planner_type.c_str());
            return 0;
        }


        cv::putText(tmRRT->image, std::to_string(j) + "." + std::to_string(i + 1),
                    cv::Point(15, 30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 143, 143), 2);
        tmRRT->draw_plan(tmRRT->S_init.pose);
        tmRRT->rviz_image_plan();
        
        if(!rclcpp::ok()) break;

        tmRRT->ros_publish_plan();
        rclcpp::spin_some(tmRRT);
    }

    tmRRT->rrt_report.push_back("");

    RCLCPP_INFO(tmRRT->get_logger(), "");
    RCLCPP_INFO(tmRRT->get_logger(), "REPORT: ");
    RCLCPP_INFO(tmRRT->get_logger(), "\tplanning_time\texplored_states\trejected_states\tplan_size\tplan_length\tcombined_cost\tnumber_of_tasks\tBFS_time");
    // std::cout << std::endl << "report: " << std::endl;
    // std::cout << "\t" << "planning_time" << "\t" << "explored_states" << "\t" << "rejected_states" << "\t" << "plan_size" << "\t" << "plan_length" << "\t" << "combined_cost" << "\t" << "number_of_tasks"<<"\t"<< "bfs_time"<<std::endl;
    for (auto i = 0; i < tmRRT->rrt_report.size(); i++)
        RCLCPP_INFO(tmRRT->get_logger(), "%d\t%s", i + 1, tmRRT->rrt_report[i].c_str());
        // std::cout << i + 1 << "\t" << tmRRT->rrt_report[i] << std::endl;
        
    return 0;
}
