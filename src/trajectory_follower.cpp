#include <ros/ros.h>
#include <dynamic_gap/trajectory_follower.h>
#include <math.h>
#include <cmath>

namespace dynamic_gap
{
    float P2EDistance(float r1, float r2, float the1, float the2)
    {
        float t1 = pow(r1, 2) + pow(r2, 2);
        float t2 = 2 * r1 * r2 * cos(the1 - the2);
        return sqrt(t1 + t2);
    }

    Eigen::Matrix2cd getComplexMatrix(double x, double y, double quat_w, double quat_z)
    {
        std::complex<double> phase(quat_w, quat_z);
        phase = phase * phase;

        Eigen::Matrix2cd g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }

    Eigen::Matrix2cd getComplexMatrix(double x, double y, double theta)
    {
        std::complex<double> phase(std::cos(theta), std::sin(theta));

        Eigen::Matrix2cd g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }

    geometry_msgs::Twist controlLaw(geometry_msgs::Pose current, nav_msgs::Odometry desired, double k_turn_, double k_drive_x_, double k_drive_y_)
    {
        geometry_msgs::Twist cmd_vel;

        geometry_msgs::Point position = current.position;
        geometry_msgs::Quaternion orientation = current.orientation;

        Eigen::Matrix2cd g_curr = getComplexMatrix(position.x, position.y, orientation.w, orientation.z);
        // ROS_DEBUG_NAMED(name_, "[%s] Current:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_curr.real()(0, 0), g_curr.imag()(0, 0), g_curr.real()(0, 1), g_curr.imag()(0, 1), g_curr.real()(1, 0), g_curr.imag()(1, 0), g_curr.real()(1, 1), g_curr.imag()(1, 1));

        position = desired.pose.pose.position;
        orientation = desired.pose.pose.orientation;

        Eigen::Matrix2cd g_des = getComplexMatrix(position.x, position.y, orientation.w, orientation.z);
        // ROS_DEBUG_NAMED(name_, "[%s] Desired:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_des.real()(0, 0), g_des.imag()(0, 0), g_des.real()(0, 1), g_des.imag()(0, 1), g_des.real()(1, 0), g_des.imag()(1, 0), g_des.real()(1, 1), g_des.imag()(1, 1));

        Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;

        // ROS_DEBUG_NAMED(name_, "[%s] Error:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_error.real()(0, 0), g_error.imag()(0, 0), g_error.real()(0, 1), g_error.imag()(0, 1), g_error.real()(1, 0), g_error.imag()(1, 0), g_error.real()(1, 1), g_error.imag()(1, 1));

        double theta_error = std::arg(g_error(0, 0));
        double x_error = g_error.real()(0, 1);
        double y_error = g_error.imag()(0, 1);

        double v_ang_fb = theta_error * k_turn_ + y_error * k_drive_y_;
        // ROS_INFO_STREAM("Theta Err: " << theta_error << ", y err:" << y_error);
        double v_lin_fb = abs(theta_error) > M_PI / 3? 0 : x_error * k_drive_x_;

        double v_ang_ff = desired.twist.twist.angular.z;
        double v_lin_ff = desired.twist.twist.linear.x;

        double v_ang = v_ang_fb + v_ang_ff;
        double v_lin = v_lin_fb + v_lin_ff;

        cmd_vel.linear.x = v_lin;
        cmd_vel.angular.z = v_ang;
        return cmd_vel;
    }

    geometry_msgs::Twist controlLaw(geometry_msgs::Pose current, nav_msgs::Odometry desired, 
                                    const sensor_msgs::LaserScan& inflated_egocircle, 
                                    bool holonomic, bool projection_operator,
                                    double k_turn_, double k_drive_x_, double k_drive_y_, double k_po_,
                                    float v_ang_const, float v_lin_x_const, float v_lin_y_const,
                                    float r_min, float r_norm, float r_norm_offset)
    {
        geometry_msgs::Twist cmd_vel;

        geometry_msgs::Point position = current.position;
        geometry_msgs::Quaternion orientation = current.orientation;

        tf::Quaternion q_c(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m_c(q_c);
        double c_roll, c_pitch, c_yaw;
        m_c.getRPY(c_roll, c_pitch, c_yaw);

        Eigen::Matrix2cd g_curr = getComplexMatrix(position.x, position.y, c_yaw);
        // ROS_DEBUG_NAMED(name_, "[%s] Current:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_curr.real()(0, 0), g_curr.imag()(0, 0), g_curr.real()(0, 1), g_curr.imag()(0, 1), g_curr.real()(1, 0), g_curr.imag()(1, 0), g_curr.real()(1, 1), g_curr.imag()(1, 1));

        position = desired.pose.pose.position;
        orientation = desired.pose.pose.orientation;

        tf::Quaternion q_d(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m_d(q_d);
        double d_roll, d_pitch, d_yaw;
        m_d.getRPY(d_roll, d_pitch, d_yaw);

        Eigen::Matrix2cd g_des = getComplexMatrix(position.x, position.y, d_yaw);
        // ROS_DEBUG_NAMED(name_, "[%s] Desired:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_des.real()(0, 0), g_des.imag()(0, 0), g_des.real()(0, 1), g_des.imag()(0, 1), g_des.real()(1, 0), g_des.imag()(1, 0), g_des.real()(1, 1), g_des.imag()(1, 1));

        Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;

        // ROS_DEBUG_NAMED(name_, "[%s] Error:\n%f + %fi, %f + %fi\n%f + %fi, %f + %fi", name_.c_str(), g_error.real()(0, 0), g_error.imag()(0, 0), g_error.real()(0, 1), g_error.imag()(0, 1), g_error.real()(1, 0), g_error.imag()(1, 0), g_error.real()(1, 1), g_error.imag()(1, 1));

        float theta_error = std::arg(g_error(0, 0));
        float x_error = g_error.real()(0, 1);
        float y_error = g_error.imag()(0, 1);

        // float v_ang_fb = 0;
        // float v_lin_x_fb = 0;
        // float v_lin_y_fb = 0;

        float u_add_x = 0;
        float u_add_y = 0;

        float v_ang_fb = theta_error * k_turn_;
        float v_lin_x_fb = x_error * k_drive_x_;
        float v_lin_y_fb = y_error * k_drive_y_;

        // bool projected = false;
        if(projection_operator)
        {
            // float r_min = 0.5;
            // float r_norm = 1;
            float r_max = r_norm + r_norm_offset;

            // find the minimum distance point on the inflated egocircle
            int min_idx = std::min_element( inflated_egocircle.ranges.begin(), inflated_egocircle.ranges.end() ) - inflated_egocircle.ranges.begin();
            float min_dist = (float) inflated_egocircle.ranges[min_idx];
            min_dist = min_dist >= r_max ? r_max : min_dist;
            min_dist = min_dist <= 0 ? 0.01 : min_dist;

            float min_dist_ang = (float)(min_idx) * inflated_egocircle.angle_increment + inflated_egocircle.angle_min;
            float min_x = min_dist * cos(min_dist_ang);
            float min_y = min_dist * sin(min_dist_ang);
            float min_diff_x = - min_x;
            float min_diff_y = - min_y;

            float si = (r_min / min_dist - r_min / r_norm) / (1. - r_min / r_norm);

            // std::cout << "Si: " << si << std::endl;

            float si_der_x = min_diff_x / sqrt(pow(min_dist, 3)) / pow(min_dist, 2) * r_min * (1. - r_min / r_norm);
            float si_der_y = min_diff_y / sqrt(pow(min_dist, 3)) / pow(min_dist, 2) * r_min * (1. - r_min / r_norm);
            // float si_der_x = min_diff_x;
            // float si_der_y = min_diff_y;

            // std::cout << "Si_der: [ " << si_der_x << " , " << si_der_y << " ]" << std::endl;

            float norm_si_der = sqrt(pow(si_der_x, 2) + pow(si_der_y, 2));
            float norm_si_der_x = si_der_x / norm_si_der;
            float norm_si_der_y = si_der_y / norm_si_der;

            // float prod_mul = x_error * norm_si_der_x + y_error * norm_si_der_y;
            float prod_mul = v_lin_x_fb * norm_si_der_x + v_lin_y_fb * norm_si_der_y;

            // std::cout << "prod_mul: " << prod_mul << std::endl;

            if(si >= 0 && prod_mul <= 0)
            {
                u_add_x = si * prod_mul * - norm_si_der_x;
                u_add_y = si * prod_mul * - norm_si_der_y;
            }

            // std::cout << "Prev vel: [ " << v_lin_x_fb << " , " << v_lin_y_fb << " , " << v_ang_fb << " ]" << std::endl;
            // std::cout << "After x_error: " << x_error << " , after y_error: " << y_error << std::endl;
        }

        if(holonomic)
        {
            v_ang_fb = v_ang_fb + v_ang_const;
            v_lin_x_fb = abs(theta_error) > M_PI / 3? 0 : v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;
            // v_lin_x_fb = v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;
            v_lin_y_fb = v_lin_y_fb + v_lin_y_const + k_po_ * u_add_y;

            if(v_lin_x_fb < 0)
                v_lin_x_fb = 0;
        }
        else
        {
            v_ang_fb = v_ang_fb + v_lin_y_fb + k_po_ * u_add_y + v_ang_const;
            // ROS_INFO_STREAM("Theta Err: " << theta_error << ", y err:" << y_error);
            v_lin_x_fb = abs(theta_error) > M_PI / 3? 0 : v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;
            v_lin_y_fb = 0;

            if(v_lin_x_fb < 0)
                v_lin_x_fb = 0;
        }

        cmd_vel.linear.x = v_lin_x_fb;
        cmd_vel.linear.y = v_lin_y_fb;
        cmd_vel.angular.z = v_ang_fb;
        return cmd_vel;
    }
} // namespace dynamic_gap
