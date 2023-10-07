#include <dynamic_gap/utils/gap_utils.h>

namespace dynamic_gap {
    // GapUtils::GapUtils() {}

    // GapUtils::~GapUtils() {}

    ////////////////// STATIC SCAN SEPARATION ///////////////////////

    bool compareModelBearingValues(dynamic_gap::Estimator* model_one, dynamic_gap::Estimator* model_two) {
        Eigen::Matrix<double, 4, 1> state_one = model_one->get_frozen_cartesian_state();
        Eigen::Matrix<double, 4, 1> state_two = model_two->get_frozen_cartesian_state();
        
        return atan2(state_one[1], state_one[0]) < atan2(state_two[1], state_two[0]);
    }

    bool checkModelSimilarity(dynamic_gap::Estimator * curr_model, dynamic_gap::Estimator * prev_model) {
        double eps = 0.00001;
        
        Eigen::Matrix<double, 4, 1> curr_state = curr_model->get_frozen_cartesian_state();
        Eigen::Matrix<double, 4, 1> prev_state = prev_model->get_frozen_cartesian_state();
        
        Eigen::Vector2d curr_vel(curr_state[2], curr_state[3]);
        Eigen::Vector2d prev_vel(prev_state[2], prev_state[3]);

        Eigen::Vector2d curr_vel_dir = curr_vel / (curr_vel.norm() + eps);
        Eigen::Vector2d prev_vel_dir = prev_vel / (prev_vel.norm() + eps);
        // ROS_INFO_STREAM("curr_velocity: " << curr_state[2] << ", " << curr_state[3] << ", prev_velocity: " << prev_state[2] << ", " << prev_state[3]);
        // ROS_INFO_STREAM("curr_vel_dir: " << curr_vel_dir[0] << ", " << curr_vel_dir[1]);
        // ROS_INFO_STREAM("prev_vel_dir: " << prev_vel_dir[0] << ", " << prev_vel_dir[1]);
        double dot_prod = curr_vel_dir.dot(prev_vel_dir);
        // ROS_INFO_STREAM("dot product: " << dot_prod);
        double norm_ratio = prev_vel.norm() / (curr_vel.norm() + eps);
        // ROS_INFO_STREAM("norm ratio: " << norm_ratio);

        return (dot_prod > 0.5 && curr_vel.norm() > 0.1 && std::abs(norm_ratio - 1.0) < 0.1);
    }

    void createAgentFromModels(dynamic_gap::Estimator * curr_model,    
                               dynamic_gap::Estimator * prev_model,
                               std::vector<Eigen::Matrix<double, 4, 1> > & agents,
                               bool print) {
        Eigen::Matrix<double, 4, 1> curr_state = curr_model->get_frozen_cartesian_state();
        Eigen::Matrix<double, 4, 1> prev_state = prev_model->get_frozen_cartesian_state();
        
        Eigen::Matrix<double, 4, 1> new_agent = (curr_state + prev_state) / 2;
        // if (print) ROS_INFO_STREAM("instantiating agent: " << new_agent[0] << ", " << new_agent[1] << ", " << new_agent[2] << ", " << new_agent[3]);
        agents.push_back(new_agent);                                  
    }

    void clearAgentFromStaticScan(dynamic_gap::Estimator * curr_model, 
                               dynamic_gap::Estimator * prev_model,
                               sensor_msgs::LaserScan & curr_scan) {

        int half_num_scan = curr_scan.ranges.size() / 2;

        Eigen::Vector2d curr_meas = curr_model->get_x_tilde();
        Eigen::Vector2d prev_meas = prev_model->get_x_tilde();

        int curr_idx = int(std::round(std::atan2(curr_meas[1], curr_meas[0]) * (half_num_scan / M_PI))) + half_num_scan;
        int prev_idx = int(std::round(std::atan2(prev_meas[1], prev_meas[0]) * (half_num_scan / M_PI))) + half_num_scan;
        // ROS_INFO_STREAM("prev_idx: " << prev_idx << ", curr_idx: " << curr_idx);

        int prev_free_idx, curr_free_idx;
        float prev_free_dist, curr_free_dist;
        for (int j = 1; j < 2; j++) {
            prev_free_idx = (prev_idx - j);
            if (prev_free_idx < 0) {
                prev_free_idx += 2*half_num_scan;
            }
            prev_free_dist = curr_scan.ranges.at(prev_free_idx);
            // ROS_INFO_STREAM("range at " << prev_free_idx << ": " << prev_free_dist);
        }

        for (int j = 1; j < 2; j++) {
            curr_free_idx = (curr_idx + j) % (2*half_num_scan);
            curr_free_dist = curr_scan.ranges.at(curr_free_idx);
            // ROS_INFO_STREAM("range at " << curr_free_idx << ": " << curr_free_dist);
        }

        // need distance from curr to prev
        float prev_to_curr_idx_dist = (curr_free_idx - prev_free_idx);
        if (prev_to_curr_idx_dist < 0) {
            prev_to_curr_idx_dist += 2*half_num_scan;
        }
        // ROS_INFO_STREAM("prev_to_curr_idx_dist: " << prev_to_curr_idx_dist);

        for (int counter = 1; counter < prev_to_curr_idx_dist; counter++) {
            int intermediate_idx = (prev_free_idx + counter) % (2*half_num_scan);
            float new_dist = (curr_free_dist - prev_free_dist) * (counter / prev_to_curr_idx_dist) + prev_free_dist;
            // ROS_INFO_STREAM("replacing range at " << intermediate_idx << " from " << curr_scan.ranges.at(intermediate_idx) << " to " << new_dist);
            curr_scan.ranges.at(intermediate_idx) = new_dist;
        }
    }

    sensor_msgs::LaserScan GapUtils::staticDynamicScanSeparation(std::vector<dynamic_gap::Gap> observed_gaps, 
                                                                boost::shared_ptr<sensor_msgs::LaserScan const> msg,
                                                                bool print) {

        sensor_msgs::LaserScan curr_scan = *msg.get(); 
        if (observed_gaps.size() == 0) {
            return curr_scan;
        }

        std::vector<dynamic_gap::Estimator *> obs_models;
        for (auto gap : observed_gaps) {
            obs_models.push_back(gap.left_model);
            obs_models.push_back(gap.right_model);
        }

        for (auto & model : obs_models) {
            model->freeze_robot_vel();
        }
        
        sort(obs_models.begin(), obs_models.end(), compareModelBearingValues);

        // iterate through models
        dynamic_gap::Estimator * prev_model = obs_models[0];
        dynamic_gap::Estimator * curr_model = obs_models[1];

        std::vector<Eigen::Matrix<double, 4, 1> > agents;

        // ROS_INFO_STREAM("looping through models");
        for (int i = 1; i < obs_models.size(); i++) {
            curr_model = obs_models[i];

            if (checkModelSimilarity(curr_model, prev_model)) {
                clearAgentFromStaticScan(curr_model, prev_model, curr_scan);
                createAgentFromModels(curr_model, prev_model, agents, print);
            }

            prev_model = curr_model;
        }

        // bridging models
        prev_model = obs_models[obs_models.size() - 1];
        curr_model = obs_models[0];

        if (checkModelSimilarity(curr_model, prev_model)) {
            clearAgentFromStaticScan(curr_model, prev_model, curr_scan);
            createAgentFromModels(curr_model, prev_model, agents, print);
        }

        curr_agents = agents;
        return curr_scan;

        // create a static scan (would need to interpolate to fill in spots where agents are)
        
        // create a pose for each agent, forward propagate it, somehow turn that pose into bearings for scan 

        // 
    }

    std::vector<Eigen::Matrix<double, 4, 1> > GapUtils::getCurrAgents() {
        return curr_agents;
    }


}