#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Utils.h>
// #include <geometry_msgs/Point.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>
// #include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dynamic_gap/gap_estimation/RotatingFrameCartesianKalmanFilter.h>
#include <dynamic_gap/gap_estimation/StaticEstimator.h>

namespace dynamic_gap
{
    class Gap
    {
        public:
            Gap() {};

            // colon used here is an initialization list. helpful for const variables.
            Gap(std::string frame, int right_idx, float rdist, bool radial, float half_scan, float min_safe_dist) : 
                _frame(frame), _right_idx(right_idx), _rdist(rdist), _radial(radial), half_scan(half_scan), min_safe_dist(min_safe_dist)
            {
                qB << 0.0, 0.0;
                terminal_qB << 0.0, 0.0;
                right_bezer_origin << 0.0, 0.0;
                left_bezier_origin << 0.0, 0.0;
            };

            ~Gap() {};
            
            // Setters and Getters for LR Distance and Index (initial and terminal gaps)
            int LIdx() const { return _left_idx; }
            void setLIdx(int lidx) { _left_idx = lidx; }

            int RIdx() const { return _right_idx; }
            void setRIdx(int ridx) { _right_idx = ridx; }

            float LDist() const { return _ldist; }
            void setLDist(float ldist) { _ldist = ldist; }

            float RDist() const { return _rdist; }
            void setRDist(float rdist) { _rdist = rdist; }

            int term_RIdx() const { return terminal_ridx; }
            void setTermRIdx(int terminal_ridx) { terminal_ridx = terminal_ridx; }

            int term_LIdx() const { return terminal_lidx; }
            void setTermLIdx(int terminal_lidx) { terminal_lidx = terminal_lidx; }

            float term_RDist() const { return terminal_rdist; }
            void setTermRDist(float term_rdist) { terminal_rdist = term_rdist; }

            float term_LDist() const { return terminal_ldist; }
            void setTermLDist(float term_ldist) { terminal_ldist = term_ldist; }

            int cvx_RIdx() const { return convex.convex_ridx; }
            void setCvxRIdx(int cvx_ridx) { convex.convex_ridx = cvx_ridx; }

            int cvx_LIdx() const { return convex.convex_lidx; }
            void setCvxLIdx(int cvx_lidx) { convex.convex_lidx = cvx_lidx; }

            float cvx_RDist() const { return convex.convex_rdist; }
            void setCvxRDist(float cvx_rdist) { convex.convex_rdist = cvx_rdist; }

            float cvx_LDist() const { return convex.convex_ldist; }
            void setCvxLDist(float cvx_ldist) { convex.convex_ldist = cvx_ldist; }

            int cvx_term_RIdx() const { return convex.terminal_ridx; }
            void setCvxTermRIdx(int cvx_term_ridx) { convex.terminal_ridx = cvx_term_ridx; }

            int cvx_term_LIdx() const { return convex.terminal_lidx; }
            void setCvxTermLIdx(int cvx_term_lidx) { convex.terminal_lidx = cvx_term_lidx; }

            float cvx_term_RDist() const { return convex.terminal_rdist; }
            void setCvxTermRDist(float cvx_term_rdist) { convex.terminal_rdist = cvx_term_rdist; }

            float cvx_term_LDist() const { return convex.terminal_ldist; }
            void setCvxTermLDist(float cvx_term_ldist) { convex.terminal_ldist = cvx_term_ldist; }

            // Concluding the Gap after constructing with left information
            void addLeftInformation(int left_idx, float ldist) 
            {
                _left_idx = left_idx;
                _ldist = ldist;
                right_type = _rdist < _ldist;

                if (!_radial)
                {
                    _radial = isRadial();
                }

                convex.convex_ridx = _right_idx;
                convex.convex_lidx = _left_idx;
                convex.convex_rdist = _rdist;
                convex.convex_ldist = _ldist;
            }

            void addTerminalRightInformation()
            {
                terminal_right_type = terminal_rdist < terminal_ldist;

                if (!_terminal_radial)
                {
                    _terminal_radial = isRadial();
                }

                convex.terminal_ridx = terminal_ridx;
                convex.terminal_lidx = terminal_lidx;
                convex.terminal_rdist = terminal_rdist;
                convex.terminal_ldist = terminal_ldist;
            }

            // Get Left Cartesian Distance
            // edited by Max: float &x, float &y
            void getLCartesian(float &x, float &y) const
            {
                float ltheta = idx2theta(_left_idx);
                x = (_ldist) * cos(ltheta);
                y = (_ldist) * sin(ltheta);
            }

            // Get Right Cartesian Distance
            void getRCartesian(float &x, float &y) const
            {
                float rtheta = idx2theta(_right_idx);
                x = (_rdist) * cos(_right_idx);
                y = (_rdist) * sin(_right_idx);
            }

            void getSimplifiedLCartesian(float &x, float &y) const
            {
                float ltheta = idx2theta(convex.convex_lidx);
                // std::cout << "convex_rdist: " << convex_rdist << ", convex_ridx: " << convex_ridx << ", half_scan: " << half_scan << std::endl;
                x = (convex.convex_ldist) * cos(ltheta);
                y = (convex.convex_ldist) * sin(ltheta);
            }

            void getSimplifiedRCartesian(float &x, float &y) const
            {
                float rtheta = idx2theta(convex.convex_ridx);
                // std::cout << "convex_ldist: " << convex_ldist << ", convex_lidx: " << convex_lidx << ", half_scan: " << half_scan << std::endl;
                x = (convex.convex_rdist) * cos(rtheta);
                y = (convex.convex_rdist) * sin(rtheta);
            }

            void initManipIndices() 
            {
                convex.convex_ridx = _right_idx;
                convex.convex_rdist = _rdist;
                convex.convex_lidx = _left_idx;
                convex.convex_ldist = _ldist;

                convex.terminal_ridx = terminal_ridx;
                convex.terminal_rdist = terminal_rdist;
                convex.terminal_lidx = terminal_lidx;
                convex.terminal_ldist = terminal_ldist;
            }

            void setRadial(bool initial = true)
            {
                // ROS_INFO_STREAM("setRadial:");
                // does resoln here imply 360 deg FOV?
                int check_l_idx = initial ? _left_idx : terminal_lidx;
                int check_r_idx = initial ? _right_idx : terminal_ridx;

                float check_l_dist = initial ? _ldist : terminal_ldist;
                float check_r_dist = initial ? _rdist : terminal_rdist;

                // ROS_INFO_STREAM("   check_l_idx: " << check_l_idx);
                // ROS_INFO_STREAM("   check_l_dist: " << check_l_dist);
                // ROS_INFO_STREAM("   check_r_idx: " << check_r_idx);
                // ROS_INFO_STREAM("   check_r_dist: " << check_r_dist);

                float resoln = M_PI / half_scan;
                float gap_angle = (check_l_idx - check_r_idx) * resoln;
                if (gap_angle < 0)
                    gap_angle += 2*M_PI;

                // ROS_INFO_STREAM("   gap_angle: " << gap_angle);
                float short_side = right_type ? check_r_dist : check_l_dist;
                // law of cosines
                float opp_side = sqrt(pow(check_r_dist, 2) + pow(check_l_dist, 2) - 2 * check_r_dist * check_l_dist * cos(gap_angle));
                // law of sines
                float small_angle = asin((short_side / opp_side) * sin(gap_angle));
                // ROS_INFO_STREAM("short_side: " << short_side);
                // ROS_INFO_STREAM("opp_side: " << opp_side);
                // ROS_INFO_STREAM("small angle: " << small_angle);

                // ROS_INFO_STREAM("   small_angle: " << small_angle);
                // ROS_INFO_STREAM("   gap_angle: " << gap_angle);
                float alpha = (M_PI - small_angle - gap_angle);
                // ROS_INFO_STREAM("   alpha: " << alpha);

                if (initial)
                    _radial = alpha > 0.75 * M_PI;
                else
                    _terminal_radial = alpha > 0.75 * M_PI;     
            }

            bool isRadial(bool initial = true) const
            {
                return (initial ? _radial : _terminal_radial);
            }

            // void setRadial()
            // {
            //     _radial = false;
            // }

            bool isRightType(bool initial = true) const
            {
                if (initial)
                    return right_type;
                else
                    return terminal_right_type;
            }

            void resetFrame(std::string frame) 
            {
                _frame = frame;
            }

            float getMinSafeDist() 
            {
                return min_safe_dist;
            }

            void setTerminalMinSafeDist(float _dist) 
            {
                terminal_min_safe_dist = _dist;
            }

            float getTerminalMinSafeDist() 
            {
                return terminal_min_safe_dist;
            }

            std::string getFrame() 
            {
                return _frame;
            }

            void setCategory(std::string _category) {
                // ROS_INFO_STREAM("setting category to: " << _category);
                category = _category;
            }

            std::string getCategory() {
                return category;
            }

            void setCrossingPoint(float x, float y) {
                // ROS_INFO_STREAM("setting crossing point to: " << x << ", " << y);
                crossing_pt << x,y;
            }

            Eigen::Vector2f getCrossingPoint() {
                return crossing_pt;
            }

            void setClosingPoint(float x, float y) {
                // ROS_INFO_STREAM("setting closing point to: " << x << ", " << y);
                closing_pt << x,y;
            }

            Eigen::Vector2f getClosingPoint() {
                return closing_pt;
            }

            // used in calculating alpha, the angle formed between the two gap lines and the robot. (angle of the gap).
            // calculates the euclidean distance between the left and right gap points using the law of cosines
            float get_gap_euclidean_dist() const 
            {
                int idx_diff = _left_idx - _right_idx;
                if (idx_diff < 0) {
                    idx_diff += (2*half_scan);
                } 
                float gap_angle = (float(idx_diff) / float(half_scan)) * M_PI;
                return sqrt(pow(_rdist, 2) + pow(_ldist, 2) - 2 * _rdist * _ldist * cos(gap_angle));
            }

            void setTerminalPoints(float _terminal_lidx, float _terminal_ldist, float _terminal_ridx, float _terminal_rdist) {
                
                terminal_lidx = _terminal_lidx;
                terminal_ldist = _terminal_ldist;
                terminal_ridx = _terminal_ridx;
                terminal_rdist = _terminal_rdist;

                if (terminal_lidx == terminal_ridx) {
                    // ROS_INFO_STREAM("terminal indices are the same");
                    terminal_lidx = (terminal_lidx + 1) % 512;
                }
                // ROS_INFO_STREAM("setting terminal points to, left: (" << terminal_lidx << ", " << terminal_ldist << "), right: ("  << terminal_ridx << ", " << terminal_rdist << ")");
            }

            void printCartesianPoints(bool initial, bool simplified) 
            {
                float x_l, y_l, x_r, y_r;
                float ltheta, rtheta, ldist, rdist;
                if (initial) 
                {
                    if (simplified) 
                    {
                        ltheta = idx2theta(_left_idx);
                        rtheta = idx2theta(_right_idx);
                        ldist = _ldist;
                        rdist = _rdist;
                    } else 
                    {
                        ltheta = idx2theta(convex.convex_lidx);
                        rtheta = idx2theta(convex.convex_ridx);    
                        ldist = convex.convex_ldist;
                        rdist = convex.convex_rdist;                                                         
                    }
                } else 
                {
                    if (simplified) 
                    {
                        ltheta = idx2theta(terminal_lidx);
                        rtheta = idx2theta(terminal_ridx);     
                        ldist = terminal_ldist;
                        rdist = terminal_rdist;          
                    } else 
                    {
                        ltheta = idx2theta(convex.terminal_lidx);
                        rtheta = idx2theta(convex.terminal_ridx);    
                        ldist = convex.terminal_ldist;
                        rdist = convex.terminal_rdist;                       
                    }
                }

                x_l = ldist * cos(ltheta);
                y_l = ldist * sin(ltheta);
                x_r = rdist * cos(rtheta);
                y_r = rdist * sin(rtheta);

                ROS_INFO_STREAM("x_l, y_l: (" << x_l << ", " << y_l << "), x_r,y_r: (" << x_r << ", " << y_r << ")");
            }   
            
            float gap_lifespan = 5.0   ;

            int _right_idx = 0;
            int _left_idx = 511;
            float _rdist = 5;
            float _ldist = 5;
            int terminal_ridx = 0;
            int terminal_lidx = 511;
            float terminal_rdist = 5;
            float terminal_ldist = 5;

            float min_safe_dist = -1;
            float terminal_min_safe_dist = -1;
            Eigen::Vector2f qB;
            Eigen::Vector2f terminal_qB;
            Eigen::Vector2f right_bezer_origin;
            Eigen::Vector2f left_bezier_origin;
            float half_scan = 256;

            std::string _frame = "";
            bool _radial = false;
            bool _terminal_radial = false;
            bool right_type = false;
            bool terminal_right_type = false;

            float peak_velocity_x = 0.0;
            float peak_velocity_y = 0.0;

            struct converted {
                int convex_ridx = 0;
                int convex_lidx = 511;
                float convex_rdist = 5;
                float convex_ldist = 5;
                int terminal_ridx = 0;
                int terminal_lidx = 511;
                float terminal_rdist = 5;
                float terminal_ldist = 5;
            } convex;

            struct GapMode {
                bool reduced = false;
                bool convex = false;
                bool agc = false;
                bool terminal_reduced = false;
                bool terminal_convex = false;
                bool terminal_agc = false;
            } mode;

            struct Goal {
                float x, y;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } goal;

            struct TerminalGoal {
                float x, y;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } terminal_goal;

            Estimator *right_model;
            Estimator *left_model;
            int _index;
            std::string category;
            Eigen::Vector2f crossing_pt;
            Eigen::Vector2f closing_pt;

            bool gap_crossed = false;
            bool gap_closed = false;
            bool gap_crossed_behind = false;

            bool artificial = false;

            float left_weight = 0.0;
            float right_weight = 0.0;
            Eigen::MatrixXd left_right_centers, all_curve_pts;
            Eigen::Vector4f spline_x_coefs, spline_y_coefs;

            Eigen::Vector2d left_pt_0, left_pt_1, right_pt_0, right_pt_1;
            int num_left_rge_points = 0;
            int num_right_rge_points = 0;
        // private:
    };
}