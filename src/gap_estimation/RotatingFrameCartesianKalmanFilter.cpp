#include <dynamic_gap/gap_estimation/RotatingFrameCartesianKalmanFilter.h>

namespace dynamic_gap 
{
    RotatingFrameCartesianKalmanFilter::RotatingFrameCartesianKalmanFilter() 
    {
        H_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
        H_transpose_ = H_.transpose();
        
        Q_scalar = 0.1;
        R_scalar = 0.5; // low value: velocities become very sensitive

        Q_temp_ << 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, Q_scalar, 0.0,
               0.0, 0.0, 0.0, Q_scalar;
        Q_k_ = Q_temp_;
        R_temp_ << R_scalar, 0.0,
                0.0, R_scalar;
        R_k_ = R_temp_;

        G_k_ << 1.0, 1.0,
                1.0, 1.0,
                1.0, 1.0,
                1.0, 1.0;

        A_ << 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0;
        STM_ = A_;

        eyes = Eigen::MatrixXf::Identity(4,4);
    }

    // For initializing a new model
    void RotatingFrameCartesianKalmanFilter::initialize(const std::string & side, const int & modelID,
                                                        const float & gapPtX, const float & gapPtY,
                                                        const ros::Time & t_update, const geometry_msgs::TwistStamped & lastRbtVel,
                                                        const geometry_msgs::TwistStamped & lastRbtAcc, const EstimationParameters & estParams) 
    {
        this->side_ = side;
        this->modelID_ = modelID;
        // ROS_INFO_STREAM("    initialize model: " << modelID_);

        this->R_scalar = estParams.R_;
        this->Q_scalar = estParams.Q_;

        Q_temp_ << 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, Q_scalar, 0.0,
        0.0, 0.0, 0.0, Q_scalar;
        Q_k_ = Q_temp_;
        R_temp_ << R_scalar, 0.0,
                0.0, R_scalar;
        R_k_ = R_temp_;

        // COVARIANCE MATRIX
        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        // larger P_0 helps with GT values that are non-zero
        // larger P_0 gives more weight to measurements (behaves like Q)
        this->P_kmin1_plus_ << 0.01, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0,
                                0.0, 0.0, 0.10, 0.0,
                                0.0, 0.0, 0.0, 0.10;
        this->P_k_minus_ = P_kmin1_plus_;
        this->P_k_plus_ = P_kmin1_plus_;

        this->lastRbtVel_ = lastRbtVel;
        this->lastRbtAcc_ = lastRbtAcc;
        this->tLastUpdate_ = t_update;

        float gapPtVxRel = -lastRbtVel_.twist.linear.x;
        float gapPtVyRel = -lastRbtVel_.twist.linear.y;

        // ROS_INFO_STREAM("        gapPtVxRel: " << gapPtVxRel << ", gapPtVyRel: " << gapPtVyRel);

        this->xTilde_ << gapPtX, gapPtY;

        this->x_hat_kmin1_plus_ << gapPtX, gapPtY, gapPtVxRel, gapPtVyRel;
        // ROS_INFO_STREAM("        x_hat_kmin1_plus_: " << x_hat_kmin1_plus_.transpose());

        this->x_hat_k_minus_ = x_hat_kmin1_plus_; 
        this->x_hat_k_plus_ = x_hat_kmin1_plus_;

        this->tStart_ = t_update;
    }

    // For transferring an existing model state to a new model
    void RotatingFrameCartesianKalmanFilter::transfer(const Estimator & model)
    {
        // ROS_INFO_STREAM("        transfering of model: " << model.modelID_);
        // From Estimator.h
        this->modelID_ = model.modelID_;
        this->side_ = model.side_;

        this->x_hat_kmin1_plus_ = model.x_hat_kmin1_plus_;
        this->x_hat_k_minus_ = model.x_hat_k_minus_;
        this->x_hat_k_plus_ = model.x_hat_k_plus_;

        // COVARIANCE MATRIX
        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        this->P_kmin1_plus_ = model.P_kmin1_plus_;
        this->P_k_minus_ = model.P_k_minus_;
        this->P_k_plus_ = model.P_k_plus_;

        this->xFrozen_ = model.xFrozen_;
        this->xRewind_ = model.xRewind_;

        this->G_k_ = model.G_k_;
        this->xTilde_ = model.xTilde_;

        this->R_k_ = model.R_k_;
        this->R_temp_ = model.R_temp_;
        this->Q_k_ = model.Q_k_;
        this->Q_temp_ = model.Q_temp_;


        this->intermediateRbtVels_ = model.intermediateRbtVels_;
        this->intermediateRbtAccs_ = model.intermediateRbtAccs_;
        this->lastRbtVel_ = model.lastRbtVel_;
        this->lastRbtAcc_ = model.lastRbtAcc_;

        this->tStart_ = model.tStart_;
        this->tLastUpdate_ = model.tLastUpdate_;

        this->manip_ = false; // will set to true if we do need to manip

        return;
    }

    // For transferring an existing model state to a new model
    void RotatingFrameCartesianKalmanFilter::transferFromPropagatedGapPoint(const Estimator & model)
    {
        // ROS_INFO_STREAM("        transfering of model: " << model.modelID_);
        // From Estimator.h
        this->modelID_ = model.modelID_;  // keeping the same for now ...
        this->side_ = model.side_;

        // We have been propagating xFrozen_ forward in time, so we will use that as our new x_hat_kmin1_plus_
        this->x_hat_kmin1_plus_ = model.xFrozen_;
        this->x_hat_k_minus_ = model.xFrozen_;
        this->x_hat_k_plus_ = model.xFrozen_;

        // COVARIANCE MATRIX
        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        this->P_kmin1_plus_ = model.P_kmin1_plus_;
        this->P_k_minus_ = model.P_k_minus_;
        this->P_k_plus_ = model.P_k_plus_;

        this->xFrozen_ = model.xFrozen_;
        this->xRewind_ = model.xRewind_;

        this->G_k_ = model.G_k_;
        this->xTilde_ = model.xFrozen_.head(2);

        this->R_k_ = model.R_k_;
        this->R_temp_ = model.R_temp_;
        this->Q_k_ = model.Q_k_;
        this->Q_temp_ = model.Q_temp_;


        this->intermediateRbtVels_ = model.intermediateRbtVels_;
        this->intermediateRbtAccs_ = model.intermediateRbtAccs_;
        this->lastRbtVel_ = model.lastRbtVel_;
        this->lastRbtAcc_ = model.lastRbtAcc_;

        this->tStart_ = model.tStart_;
        this->tLastUpdate_ = model.tLastUpdate_;

        this->manip_ = false; // will set to true if we do need to manip

        return;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::integrate() 
    {
        // ROS_INFO_STREAM("    [integrate()]");
        Eigen::Vector4f x_intermediate = x_hat_kmin1_plus_;
        Eigen::Vector4f new_x = x_hat_kmin1_plus_;

        for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++) 
        {
            // ROS_INFO_STREAM("        intermediate step " << i);
            
            float dt = (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec();

            // ROS_INFO_STREAM("        dt " << dt);

            float ang_vel_ego = intermediateRbtVels_[i].twist.angular.z;
            
            // ROS_INFO_STREAM("        ang_vel_ego: " << ang_vel_ego);

            float p_dot_x = (x_intermediate[2] + ang_vel_ego*x_intermediate[1]);
            float p_dot_y = (x_intermediate[3] - ang_vel_ego*x_intermediate[0]);
            // ROS_INFO_STREAM("        p_dot_x: " << p_dot_x << ", p_dot_y: " << p_dot_y);

            float vdot_x_body = intermediateRbtAccs_[i].twist.linear.x;
            float vdot_y_body = intermediateRbtAccs_[i].twist.linear.y;
            // ROS_INFO_STREAM("        vdot_x_body: " << vdot_x_body << ", vdot_y_body: " << vdot_y_body);

            float v_dot_x = (x_intermediate[3]*ang_vel_ego - vdot_x_body);
            float v_dot_y = (-x_intermediate[2]*ang_vel_ego - vdot_y_body);
            // ROS_INFO_STREAM("        v_dot_x: " << v_dot_x << ", v_dot_y: " << v_dot_y);

            new_x << x_intermediate[0] + p_dot_x*dt, // r_x
                     x_intermediate[1] + p_dot_y*dt, // r_y
                     x_intermediate[2] + v_dot_x*dt, // v_x
                     x_intermediate[3] + v_dot_y*dt; // v_y
            
            x_intermediate = new_x;
            // ROS_INFO_STREAM("        x_intermediate: " << x_intermediate.transpose());
        
        }

        return x_intermediate;
    }

    void RotatingFrameCartesianKalmanFilter::linearize(const int & idx) 
    {
        float dt = (intermediateRbtVels_[idx + 1].header.stamp - intermediateRbtVels_[idx].header.stamp).toSec();    
        float ang_vel_ego = intermediateRbtVels_[idx].twist.angular.z;
        
        A_ << 0.0, ang_vel_ego, 1.0, 0.0,
             -ang_vel_ego, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, ang_vel_ego,
             0.0, 0.0, -ang_vel_ego, 0.0;

        STM_ = (A_*dt).exp();
    }

    void RotatingFrameCartesianKalmanFilter::discretizeQ(const int & idx) 
    {
        float dt = (intermediateRbtVels_[idx + 1].header.stamp - intermediateRbtVels_[idx].header.stamp).toSec();

        Q_1_ = Q_k_;
        Q_2_ = A_ * Q_1_ + Q_1_ * A_.transpose();
        Q_3_ = A_ * Q_2_ + Q_2_ * A_.transpose();

        dQ_ = (Q_1_ * dt) + (Q_2_ * dt * dt / 2.0) + (Q_3_ * dt * dt * dt / 6.0);
    }

    void RotatingFrameCartesianKalmanFilter::update(const Eigen::Vector2f & measurement, 
                                                    const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                                                    const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                                                    const std::map<std::string, geometry_msgs::Pose> & agentPoses,
                                                    const std::map<std::string, geometry_msgs::Vector3Stamped> & agentVels,
                                                    const ros::Time & t_update)
    {    
        // acceleration and velocity come in wrt robot frame
        intermediateRbtVels_ = intermediateRbtVels;
        intermediateRbtAccs_ = intermediateRbtAccs;

        // ROS_INFO_STREAM("    update for model: " << getID()); // << ", life_time: " << life_time << ", dt: " << dt << ", inter_dt: " << inter_dt);
        // ROS_INFO_STREAM("    t_update: " << t_update); // << ", life_time: " << life_time << ", dt: " << dt << ", inter_dt: " << inter_dt);
        // ROS_INFO_STREAM("    tLastUpdate_: " << tLastUpdate_); // << ", life_time: " << life_time << ", dt: " << dt << ", inter_dt: " << inter_dt);

        // ROS_INFO_STREAM("    intermediateRbtVels_.size(): " << intermediateRbtVels_.size());
        // ROS_INFO_STREAM("    intermediateRbtAccs_.size(): " << intermediateRbtAccs_.size());

        if (intermediateRbtVels_.size() == 0 || intermediateRbtAccs_.size() == 0)
        {
            ROS_WARN_STREAM_COND_NAMED(intermediateRbtVels_.size() == 0, "GapEstimation", "intermediateRbtVels_ is empty, no update");
            ROS_WARN_STREAM_COND_NAMED(intermediateRbtAccs_.size() == 0, "GapEstimation", "intermediateRbtAccs_ is empty, no update");
            return;
        }

        // ROS_INFO_STREAM("    x_hat_kmin1_plus_: " << x_hat_kmin1_plus_[0] << ", " << x_hat_kmin1_plus_[1] << ", " << x_hat_kmin1_plus_[2] << ", " << x_hat_kmin1_plus_[3]);
        // ROS_INFO_STREAM("    current_rbt_vel, x_lin: " << lastRbtVel_.twist.linear.x << ", y_lin: " << lastRbtVel_.twist.linear.y << ", z_ang: " << lastRbtVel_.twist.angular.z);

        processEgoRobotVelsAndAccs(t_update);

        if (intermediateRbtVels_.size() != intermediateRbtAccs_.size())
        {
            // ROS_INFO_STREAM("    intermediateRbtVels_ is of size " << intermediateRbtVels_.size() << " while intermediateRbtAccs_ is of size " << intermediateRbtAccs_.size());
            return;
        }

        // get_intermediateRbtVels__accs();

        xTilde_ = measurement; 
                // << range_bearing_measurement[0]*std::cos(range_bearing_measurement[1]),
                //    range_bearing_measurement[0]*std::sin(range_bearing_measurement[1]);
        
        // ROS_INFO_STREAM("    linear ego vel: " << lastRbtVel_.twist.linear.x << ", " << lastRbtVel_.twist.linear.y << ", angular ego vel: " << lastRbtVel_.twist.angular.z);
        // ROS_INFO_STREAM("    linear ego acceleration: " << lastRbtAcc_.twist.linear.x << ", " << lastRbtAcc_.twist.linear.y << ", angular ego acc: " << lastRbtAcc_.twist.angular.z);        

        x_hat_k_minus_ = integrate();
        
        // ROS_INFO_STREAM("    x_hat_k_minus_: " << x_hat_k_minus_.transpose());

        P_intermediate = P_kmin1_plus_;
        new_P = P_kmin1_plus_;
        for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++) 
        {
            linearize(i);

            // ROS_INFO_STREAM("    A_: " << A_(0, 0) << ", " << A_(0, 1) << ", " << A_(0, 2) << ", " << A_(0, 3));
            // ROS_INFO_STREAM("        " << A_(1, 0) << ", " << A_(1, 1) << ", " << A_(1, 2) << ", " << A_(1, 3));
            // ROS_INFO_STREAM("        " << A_(2, 0) << ", " << A_(2, 1) << ", " << A_(2, 2) << ", " << A_(2, 3));
            // ROS_INFO_STREAM("        " << A_(3, 0) << ", " << A_(3, 1) << ", " << A_(3, 2) << ", " << A_(3, 3));     

            // ROS_INFO_STREAM("    STM_: " << STM_(0, 0) << ", " << STM_(0, 1) << ", " << STM_(0, 2) << ", " << STM_(0, 3));
            // ROS_INFO_STREAM("          " << STM_(1, 0) << ", " << STM_(1, 1) << ", " << STM_(1, 2) << ", " << STM_(1, 3));
            // ROS_INFO_STREAM("          " << STM_(2, 0) << ", " << STM_(2, 1) << ", " << STM_(2, 2) << ", " << STM_(2, 3));
            // ROS_INFO_STREAM("          " << STM_(3, 0) << ", " << STM_(3, 1) << ", " << STM_(3, 2) << ", " << STM_(3, 3));     

            discretizeQ(i);

            // ROS_INFO_STREAM("    dQ_: " << dQ_(0, 0) << ", " << dQ_(0, 1) << ", " << dQ_(0, 2) << ", " << dQ_(0, 3));
            // ROS_INFO_STREAM("         " << dQ_(1, 0) << ", " << dQ_(1, 1) << ", " << dQ_(1, 2) << ", " << dQ_(1, 3));
            // ROS_INFO_STREAM("         " << dQ_(2, 0) << ", " << dQ_(2, 1) << ", " << dQ_(2, 2) << ", " << dQ_(2, 3));
            // ROS_INFO_STREAM("         " << dQ_(3, 0) << ", " << dQ_(3, 1) << ", " << dQ_(3, 2) << ", " << dQ_(3, 3));     

            // ROS_INFO_STREAM("    P_intermediate: " << P_intermediate(0, 0) << ", " << P_intermediate(0, 1) << ", " << P_intermediate(0, 2) << ", " << P_intermediate(0, 3));
            // ROS_INFO_STREAM("                    " << P_intermediate(1, 0) << ", " << P_intermediate(1, 1) << ", " << P_intermediate(1, 2) << ", " << P_intermediate(1, 3));
            // ROS_INFO_STREAM("                    " << P_intermediate(2, 0) << ", " << P_intermediate(2, 1) << ", " << P_intermediate(2, 2) << ", " << P_intermediate(2, 3));
            // ROS_INFO_STREAM("                    " << P_intermediate(3, 0) << ", " << P_intermediate(3, 1) << ", " << P_intermediate(3, 2) << ", " << P_intermediate(3, 3));     

            new_P = STM_ * P_intermediate * STM_.transpose() + dQ_;

            P_intermediate = new_P;
        }
        P_k_minus_ = new_P;
        

        // ROS_INFO_STREAM("    P_k_minus_: " << P_k_minus_(0, 0) << ", " << P_k_minus_(0, 1) << ", " << P_k_minus_(0, 2) << ", " << P_k_minus_(0, 3));
        // ROS_INFO_STREAM("                " << P_k_minus_(1, 0) << ", " << P_k_minus_(1, 1) << ", " << P_k_minus_(1, 2) << ", " << P_k_minus_(1, 3));
        // ROS_INFO_STREAM("                " << P_k_minus_(2, 0) << ", " << P_k_minus_(2, 1) << ", " << P_k_minus_(2, 2) << ", " << P_k_minus_(2, 3));
        // ROS_INFO_STREAM("                " << P_k_minus_(3, 0) << ", " << P_k_minus_(3, 1) << ", " << P_k_minus_(3, 2) << ", " << P_k_minus_(3, 3));     

        // ROS_INFO_STREAM("    xTilde_: " << xTilde_[0] << ", " << xTilde_[1]);
        
        innovation_ = xTilde_ - H_*x_hat_k_minus_;
        x_hat_k_plus_ = x_hat_k_minus_ + G_k_*innovation_;
        residual_ = xTilde_ - H_*x_hat_k_plus_;

        tmp_mat = H_*P_k_minus_*H_transpose_ + R_k_;

        G_k_ = P_k_minus_ * H_transpose_ * tmp_mat.inverse();

        // ROS_INFO_STREAM("G_k_: " << G_k_(0, 0) << ", " << G_k_(0, 1));
        // ROS_INFO_STREAM("      " << G_k_(1, 0) << ", " << G_k_(1, 1));
        // ROS_INFO_STREAM("      " << G_k_(2, 0) << ", " << G_k_(2, 1));
        // ROS_INFO_STREAM("      " << G_k_(3, 0) << ", " << G_k_(3, 1));

        // ROS_INFO_STREAM("eyes: " << eyes(0, 0) << ", " << eyes(0, 1) << ", " << eyes(0, 2) << ", " << eyes(0, 3));
        // ROS_INFO_STREAM("      " << eyes(1, 0) << ", " << eyes(1, 1) << ", " << eyes(1, 2) << ", " << eyes(1, 3));
        // ROS_INFO_STREAM("      " << eyes(2, 0) << ", " << eyes(2, 1) << ", " << eyes(2, 2) << ", " << eyes(2, 3));
        // ROS_INFO_STREAM("      " << eyes(3, 0) << ", " << eyes(3, 1) << ", " << eyes(3, 2) << ", " << eyes(3, 3));

        // ROS_INFO_STREAM("H_: " << H_(0, 0) << ", " << H_(0, 1) << ", " << H_(0, 2) << ", " << H_(0, 3));
        // ROS_INFO_STREAM("    " << H_(1, 0) << ", " << H_(1, 1) << ", " << H_(1, 2) << ", " << H_(1, 3));
        
        P_k_plus_ = (eyes - G_k_*H_)*P_k_minus_;

        x_hat_kmin1_plus_ = x_hat_k_plus_;
        P_kmin1_plus_ = P_k_plus_;
        tLastUpdate_ = t_update;

        // ROS_INFO_STREAM("    intermediateRbtVels_ size: " << intermediateRbtVels_.size());
        // ROS_INFO_STREAM("    intermediateRbtAccs_ size: " << intermediateRbtAccs_.size());
        
        if (intermediateRbtVels_.size() > 0)
            lastRbtVel_ = intermediateRbtVels_.back();
        
        if (intermediateRbtAccs_.size() > 0)
            lastRbtAcc_ = intermediateRbtAccs_.back();

        // ROS_INFO_STREAM("4");

        // ROS_INFO_STREAM("    x_hat_k_plus_: " << x_hat_k_plus_[0] << ", " << x_hat_k_plus_[1] << ", " << x_hat_k_plus_[2] << ", " << x_hat_k_plus_[3]);       
        // ROS_INFO_STREAM("    P_k_plus_: " << P_k_plus_(0, 0) << ", " << P_k_plus_(0, 1) << ", " << P_k_plus_(0, 2) << ", " << P_k_plus_(0, 3));
        // ROS_INFO_STREAM("               " << P_k_plus_(1, 0) << ", " << P_k_plus_(1, 1) << ", " << P_k_plus_(1, 2) << ", " << P_k_plus_(1, 3));
        // ROS_INFO_STREAM("               " << P_k_plus_(2, 0) << ", " << P_k_plus_(2, 1) << ", " << P_k_plus_(2, 2) << ", " << P_k_plus_(2, 3));
        // ROS_INFO_STREAM("               " << P_k_plus_(3, 0) << ", " << P_k_plus_(3, 1) << ", " << P_k_plus_(3, 2) << ", " << P_k_plus_(3, 3));             
        // ROS_INFO_STREAM("    -----------");

        return;
    }    

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::getState()
    { 
        // // ROS_INFO_STREAM("[getState()]");
        Eigen::Vector4f state = x_hat_k_plus_;

        // if gap has been pivoted: add additional flag to model to set velocity to zero.

        // if model is still fairly fresh and not converged, just assume it is attached to a static part of the environment
        bool newModel = (tLastUpdate_ - tStart_).toSec() < lifetimeThreshold_; 
        if (newModel)
        {
            // // ROS_INFO_STREAM("        new model");
            state[2] = 0.0 - lastRbtVel_.twist.linear.x;
            state[3] = 0.0 - lastRbtVel_.twist.linear.y;   
        } 

        if (manip_)
        {
            // // ROS_INFO_STREAM("        manipulated model");
            state[0] = manipPosition[0];
            state[1] = manipPosition[1];
            state[2] = 0.0 - lastRbtVel_.twist.linear.x;
            state[3] = 0.0 - lastRbtVel_.twist.linear.y;               
        }        

        return state;  
    }    
}