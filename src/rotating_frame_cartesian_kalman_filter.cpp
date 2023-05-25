#include <dynamic_gap/rotating_frame_cartesian_kalman_filter.h>

namespace dynamic_gap 
{
    rotatingFrameCartesianKalmanFilter::rotatingFrameCartesianKalmanFilter(const int & filter_index, 
                                                                            const double & initial_range, 
                                                                            const double & initial_bearing,
                                                                            const ros::Time & t_update,
                                                                            const geometry_msgs::TwistStamped & last_ego_rbt_vel,
                                                                            const geometry_msgs::TwistStamped & last_ego_rbt_acc)
    {
        this->filter_index = filter_index;
        this->t_last_update = t_update;
        this->last_ego_rbt_vel = last_ego_rbt_vel;
        this->last_ego_rbt_acc = last_ego_rbt_acc;

        eyes = Eigen::MatrixXd::Identity(4,4);

        // OBSERVATION MATRIX
        H << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0;
        H_transpose = H.transpose();    

        // NOISE MATRICES
        Q_k << 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.5, 0.0,
            0.0, 0.0, 0.0, 0.5;
        dQ = Q_k;

        R_scalar = 0.1;

        // COVARIANCE MATRIX
        P_kmin1_plus << 0.01, 0.0, 0.0, 0.0,
                        0.0, 0.01, 0.0, 0.0,
                        0.0, 0.0, 0.1, 0.0,
                        0.0, 0.0, 0.0, 0.1;
        P_k_minus = P_kmin1_plus;
        P_k_plus = P_kmin1_plus;       

        // MEASUREMENT
        x_tilde << initial_range * std::cos(initial_bearing), 
                initial_range * std::sin(initial_bearing);
        x_hat_kmin1_plus << x_tilde[0], x_tilde[1], -last_ego_rbt_vel.twist.linear.x, -last_ego_rbt_vel.twist.linear.y;
        x_hat_k_minus = x_hat_kmin1_plus; 
        x_hat_k_plus = x_hat_kmin1_plus;

        // KALMAN GAIN
        G_k << 1.0, 1.0,
            1.0, 1.0,
            1.0, 1.0,
            1.0, 1.0;

        // STATE DYNAMICS MATRICES
        A << 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0;
        STM = A;       
    }

    void rotatingFrameCartesianKalmanFilter::processEgoRobotVelsAndAccs(const ros::Time & t_update)
    {
        /*
        // Printing original dt values from intermediate odom measurements
        ROS_INFO_STREAM("   t_0 - t_last_update difference:" << (ego_rbt_vels[0].header.stamp - t_last_update).toSec() << " sec");

        for (int i = 0; i < (ego_rbt_vels.size() - 1); i++)
        {
            ROS_INFO_STREAM("   t_" << (i+1) << " - t_" << i << " difference: " << (ego_rbt_vels[i + 1].header.stamp - ego_rbt_vels[i].header.stamp).toSec() << " sec");
        }

        ROS_INFO_STREAM("   t_update" << " - t_" << (ego_rbt_vels.size() - 1) << " difference:" << (t_update - ego_rbt_vels[ego_rbt_vels.size() - 1].header.stamp).toSec() << " sec");
        */

        // Tweaking ego robot velocities/acceleration to make sure that updates:
        //      1. Are never negative (backwards in time)
        //      2. Always start from time of last update
        //      3. Always at end of time of incoming laser scan measurement

        // Erasing odometry measurements that are from *before* the last update 
        while (!ego_rbt_vels.empty() && t_last_update > ego_rbt_vels[0].header.stamp)
        {
            ego_rbt_vels.erase(ego_rbt_vels.begin());
            ego_rbt_accs.erase(ego_rbt_accs.begin());
        }

        // Inserting placeholder odometry to represent the time of the last update
        ego_rbt_vels.insert(ego_rbt_vels.begin(), last_ego_rbt_vel);
        ego_rbt_vels[0].header.stamp = t_last_update;

        ego_rbt_accs.insert(ego_rbt_accs.begin(), last_ego_rbt_acc);
        ego_rbt_accs[0].header.stamp = t_last_update;

        // Erasing odometry measurements that occur *after* the incoming laser scan was received
        while (!ego_rbt_vels.empty() && t_update < ego_rbt_vels[ego_rbt_vels.size() - 1].header.stamp)
        {
            ego_rbt_vels.erase(ego_rbt_vels.end() - 1);
            ego_rbt_accs.erase(ego_rbt_accs.end() - 1);
        }

        // Inserting placeholder odometry to represent the time that the incoming laser scan was received
        ego_rbt_vels.push_back(ego_rbt_vels[ego_rbt_vels.size() - 1]);
        ego_rbt_vels[ego_rbt_vels.size() - 1].header.stamp = t_update;

        for (int i = 0; i < (ego_rbt_vels.size() - 1); i++)
        {
            double dt = (ego_rbt_vels[i + 1].header.stamp - ego_rbt_vels[i].header.stamp).toSec();
            
            if (print)
                ROS_INFO_STREAM("   t_" << (i+1) << " - t_" << i << " difference: " << dt << " sec");
            
            if (dt < 0)
            {
                ROS_INFO_STREAM("   ERROR IN TIMESTEP CALCULATION, SHOULD NOT BE NEGATIVE");
            }
        }
    }

    void rotatingFrameCartesianKalmanFilter::frozen_state_propagate(double froz_dt) {
        Eigen::Matrix<double, 4, 1> new_frozen_x;     
        new_frozen_x << 0.0, 0.0, 0.0, 0.0;

        Eigen::Vector2d frozen_linear_acc_ego(0.0, 0.0);

        Eigen::Vector2d frozen_linear_vel_ego(0.0, 0.0); 
        double frozen_ang_vel_ego = 0.0;

        double vdot_x_body = frozen_linear_acc_ego[0];
        double vdot_y_body = frozen_linear_acc_ego[1];

        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_frozen_x[0] = frozen_x[0] + (frozen_x[2] + frozen_x[1]*frozen_ang_vel_ego)*froz_dt;
        new_frozen_x[1] = frozen_x[1] + (frozen_x[3] - frozen_x[0]*frozen_ang_vel_ego)*froz_dt;
        new_frozen_x[2] = frozen_x[2] + (frozen_x[3]*frozen_ang_vel_ego - vdot_x_body)*froz_dt;
        new_frozen_x[3] = frozen_x[3] + (-frozen_x[2]*frozen_ang_vel_ego - vdot_y_body)*froz_dt;
        frozen_x = new_frozen_x; 
    }

    void rotatingFrameCartesianKalmanFilter::rewind_propagate(double rew_dt) {
        Eigen::Matrix<double, 4, 1> new_rewind_x;     
        new_rewind_x << 0.0, 0.0, 0.0, 0.0;

        Eigen::Vector2d frozen_linear_acc_ego(0.0, 0.0);

        Eigen::Vector2d frozen_linear_vel_ego(0.0, 0.0); 
        double frozen_ang_vel_ego = 0.0;

        double vdot_x_body = frozen_linear_acc_ego[0];
        double vdot_y_body = frozen_linear_acc_ego[1];

        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_rewind_x[0] = rewind_x[0] + (rewind_x[2] + rewind_x[1]*frozen_ang_vel_ego)*rew_dt;
        new_rewind_x[1] = rewind_x[1] + (rewind_x[3] - rewind_x[0]*frozen_ang_vel_ego)*rew_dt;
        new_rewind_x[2] = rewind_x[2] + (rewind_x[3]*frozen_ang_vel_ego - vdot_x_body)*rew_dt;
        new_rewind_x[3] = rewind_x[3] + (-rewind_x[2]*frozen_ang_vel_ego - vdot_y_body)*rew_dt;
        rewind_x = new_rewind_x; 
    }

    Eigen::Vector4d rotatingFrameCartesianKalmanFilter::integrate()
    {
        Eigen::Vector4d x_intermediate = x_hat_kmin1_plus;
        Eigen::Vector4d new_x = x_hat_kmin1_plus;  

        // Performing integrations with the ego robot vel/acc at beginning of timestep, not doing a midpoint or anything.
        for (int i = 0; i < (ego_rbt_vels.size() - 1); i++)
        {
            if (print) 
                ROS_INFO_STREAM("   i: " << i ); 
            
            double dt = (ego_rbt_vels[i + 1].header.stamp - ego_rbt_vels[i].header.stamp).toSec();
            double ang_vel_ego = ego_rbt_vels[i].twist.angular.z;

            if (print)
                ROS_INFO_STREAM("   ang_vel_ego: " << ang_vel_ego);

            double p_dot_x = (x_intermediate[2] + ang_vel_ego*x_intermediate[1]);
            double p_dot_y = (x_intermediate[3] - ang_vel_ego*x_intermediate[0]);
            
            if (print) 
                ROS_INFO_STREAM("   p_dot_x: " << p_dot_x << ", p_dot_y: " << p_dot_y);

            double vdot_x_body = ego_rbt_accs[i].twist.linear.x;
            double vdot_y_body = ego_rbt_accs[i].twist.linear.y;
            
            if (print) 
                ROS_INFO_STREAM("   vdot_x_body: " << vdot_x_body << ", vdot_y_body: " << vdot_y_body);

            double v_dot_x = (x_intermediate[3]*ang_vel_ego - vdot_x_body);
            double v_dot_y = (-x_intermediate[2]*ang_vel_ego - vdot_y_body);

            if (print)
                ROS_INFO_STREAM("   v_dot_x: " << v_dot_x << ", v_dot_y: " << v_dot_y);

            new_x << x_intermediate[0] + p_dot_x*dt, // r_x
                    x_intermediate[1] + p_dot_y*dt, // r_y
                    x_intermediate[2] + v_dot_x*dt, // v_x
                    x_intermediate[3] + v_dot_y*dt; // v_y
            
            x_intermediate = new_x;
            if (print)
                ROS_INFO_STREAM("   x_intermediate: " << x_intermediate[0] << ", " << x_intermediate[1] << ", " << x_intermediate[2] << ", " << x_intermediate[3]);
        }

        return x_intermediate;
    }

    void rotatingFrameCartesianKalmanFilter::linearize(int idx) 
    {
        double dt = (ego_rbt_vels[idx + 1].header.stamp - ego_rbt_vels[idx].header.stamp).toSec();    
        double ang_vel_ego = ego_rbt_vels[idx].twist.angular.z;
        
        A << 0.0, ang_vel_ego, 1.0, 0.0,
            -ang_vel_ego, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, ang_vel_ego,
            0.0, 0.0, -ang_vel_ego, 0.0;

        STM = (A*dt).exp(); // ARE WE SURE THIS DOES WHAT WE WANT, I THINK
    }

    void rotatingFrameCartesianKalmanFilter::discretizeQ(int idx) 
    {
        double dt = (ego_rbt_vels[idx + 1].header.stamp - ego_rbt_vels[idx].header.stamp).toSec();

        Q_1 = Q_k;
        Q_2 = A * Q_1 + Q_1 * A.transpose();
        Q_3 = A * Q_2 + Q_2 * A.transpose();

        dQ = (Q_1 * dt) + (Q_2 * dt * dt / 2.0) + (Q_3 * dt * dt * dt / 6.0);
    }


    void rotatingFrameCartesianKalmanFilter::update(Eigen::Matrix<double, 2, 1> laserscan_measurement, 
                                                    const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied, 
                                                    const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied, 
                                                    const ros::Time & t_update,
                                                    bool print)
    {
        /*
        * Update from:
        * t_last_update --> t_0
        * t_0 --> .......  --> t_k
        * t_k --> t_update
        */    
        this->ego_rbt_vels = ego_rbt_vels_copied;
        this->ego_rbt_accs = ego_rbt_accs_copied;
        this->print = print;
        this->x_tilde << laserscan_measurement[0]*std::cos(laserscan_measurement[1]),
                        laserscan_measurement[0]*std::sin(laserscan_measurement[1]);
            
        ROS_INFO_STREAM("rotatingFrameCartesianKalmanFilter::update()");
        ROS_INFO_STREAM("   model index: " << filter_index);
        ROS_INFO_STREAM("   ego_rbt_vels size: " << ego_rbt_vels.size() << ", ego_rbt_accs size: " << ego_rbt_accs.size());

        // DO I NEED TO SAFEGUARD FOR IF EGO_ROBOT_VELS is LENGTH ZERO?
        if (ego_rbt_vels.size() == 0)
            return;

        processEgoRobotVelsAndAccs(t_update);

        if (print)
            ROS_INFO_STREAM("   x_hat_kmin1_plus: " << x_hat_kmin1_plus[0] << ", " << x_hat_kmin1_plus[1] << 
                                            ", " << x_hat_kmin1_plus[2] << ", " << x_hat_kmin1_plus[3]);    


        if (print) 
            ROS_INFO_STREAM("   INTEGRATING");

        x_hat_k_minus = integrate();
        
        if (print)
            ROS_INFO_STREAM("   x_hat_k_minus: " << x_hat_k_minus[0] << ", " << x_hat_k_minus[1] << 
                                            ", " << x_hat_k_minus[2] << ", " << x_hat_k_minus[3]);

        P_intermediate = P_kmin1_plus;
        P_tmp = P_kmin1_plus;    
        for (int i = 0; i < (ego_rbt_vels.size() - 1); i++)
        {
            // linearize
            linearize(i);

            // discretizeQ
            discretizeQ(i);

            // update P
            P_tmp = STM * P_intermediate * STM.transpose() + dQ;

            P_intermediate = P_tmp;        
        }
        P_k_minus = P_intermediate;

        if (print)
            ROS_INFO_STREAM("   x_tilde: " << x_tilde[0] << ", " << x_tilde[1]);

        innovation = x_tilde - H*x_hat_k_minus;
        x_hat_k_plus = x_hat_k_minus + G_k*innovation;
        residual = x_tilde - H*x_hat_k_plus;

        double sensor_noise_factor = R_scalar * laserscan_measurement[0];
        R_k << sensor_noise_factor, 0.0,
            0.0, sensor_noise_factor;

        tmp_mat = H*P_k_minus*H_transpose + R_k;

        G_k = P_k_minus * H_transpose * tmp_mat.inverse();

        P_k_plus = (eyes - G_k*H)*P_k_minus;
        // Q_tmp = (alpha_Q * Q_k) + (1.0 - alpha_Q) * (G_k * residual * residual.transpose() * G_k.transpose());
        // Q_k = Q_tmp;

        if (print) 
        {
            ROS_INFO_STREAM("   x_hat_k_plus: " << x_hat_k_plus[0] << ", " << x_hat_k_plus[1] << ", " << x_hat_k_plus[2] << ", " << x_hat_k_plus[3]);       
            ROS_INFO_STREAM("-----------");
        }

        x_hat_kmin1_plus = x_hat_k_plus;
        P_kmin1_plus = P_k_plus;
        t_last_update = t_update;
        last_ego_rbt_vel = ego_rbt_vels[ego_rbt_vels.size() - 1];
        last_ego_rbt_acc = ego_rbt_accs[ego_rbt_accs.size() - 1];

        return;
    }

    int rotatingFrameCartesianKalmanFilter::get_index()
    {
        return filter_index;
    }

    Eigen::Vector4d rotatingFrameCartesianKalmanFilter::get_model_state()
    {
        return x_hat_k_plus;
    }

    geometry_msgs::TwistStamped rotatingFrameCartesianKalmanFilter::get_v_ego()
    {
        return last_ego_rbt_vel;
    }

    void rotatingFrameCartesianKalmanFilter::freeze_robot_vel()
    {
        Eigen::Vector4d cartesian_state = get_model_state();
        
        // fixing position (otherwise can get bugs)
        cartesian_state[0] = x_tilde[0];
        cartesian_state[1] = x_tilde[1];

        // update cartesian
        cartesian_state[2] += last_ego_rbt_vel.twist.linear.x;
        cartesian_state[3] += last_ego_rbt_vel.twist.linear.y;
        frozen_x = cartesian_state;
    }    

    Eigen::Vector4d rotatingFrameCartesianKalmanFilter::get_frozen_cartesian_state() 
    {
        return frozen_x;
    }

    Eigen::Vector2d rotatingFrameCartesianKalmanFilter::get_x_tilde() 
    {
        return x_tilde;
    }

    void rotatingFrameCartesianKalmanFilter::set_side(std::string _side) 
    {
        side = _side;
    }
    
    std::string rotatingFrameCartesianKalmanFilter::get_side() 
    {
        return side;
    }

    Eigen::Vector4d rotatingFrameCartesianKalmanFilter::get_frozen_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d frozen_mp_state;
        Eigen::Vector4d frozen_cart_state = get_frozen_cartesian_state();
        frozen_mp_state << 1.0 / sqrt(pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2)),
                           std::atan2(frozen_cart_state[1], frozen_cart_state[0]),
                           (frozen_cart_state[0]*frozen_cart_state[2] + frozen_cart_state[1]*frozen_cart_state[3]) / (pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2)),
                           (frozen_cart_state[0]*frozen_cart_state[3] - frozen_cart_state[1]*frozen_cart_state[2]) / (pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2));
        return frozen_mp_state;
    }

    void rotatingFrameCartesianKalmanFilter::set_rewind_state() {
        rewind_x = frozen_x;
    }

    Eigen::Vector4d rotatingFrameCartesianKalmanFilter::get_rewind_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = rewind_x;
        return return_x;
    }

    Eigen::Vector4d rotatingFrameCartesianKalmanFilter::get_rewind_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d rewind_mp_state;
        Eigen::Vector4d rewind_cart_state = get_rewind_cartesian_state();
        rewind_mp_state << 1.0 / sqrt(pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2)),
                           std::atan2(rewind_cart_state[1], rewind_cart_state[0]),
                           (rewind_cart_state[0]*rewind_cart_state[2] + rewind_cart_state[1]*rewind_cart_state[3]) / (pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2)),
                           (rewind_cart_state[0]*rewind_cart_state[3] - rewind_cart_state[1]*rewind_cart_state[2]) / (pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2));
        return rewind_mp_state;
    }

} // namespace dynamic_gap