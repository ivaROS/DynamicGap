#ifndef GAP_H
#define GAP_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dynamic_gap/cart_model.h>

namespace dynamic_gap
{
    class Gap
    {
        public:
            Gap() {};

            // colon used here is an initialization list. helpful for const variables.
            Gap(std::string frame, int right_idx_pov, float rdist_pov, bool axial = false, float half_scan = 256) : 
                _frame(frame), _right_idx_pov(right_idx_pov), _rdist_pov(rdist_pov), _axial(axial), half_scan(half_scan)
            {
                qB << 0.0, 0.0;
                terminal_qB << 0.0, 0.0;
                right_pov_bezier_origin << 0.0, 0.0;
                left_pov_bezier_origin << 0.0, 0.0;
            };

            ~Gap() {};
            
            // Setters and Getters for LR Distance and Index (initial and terminal gaps)
            int RIdxPOV() { return _right_idx_pov; }
            void setRIdxPOV(int ridx) { _right_idx_pov = ridx; }

            int LIdxPOV() { return _left_idx_pov; }
            void setLIdxPOV(int lidx) { _left_idx_pov = lidx; }

            float RDistPOV() { return _rdist_pov; }
            void setRDistPOV(float rdist) { _rdist_pov = rdist; }

            float LDistPOV() { return _ldist_pov; }
            void setLDistPOV(float ldist) { _ldist_pov = ldist; }

            int term_RIdxPOV() { return terminal_ridx_pov; }
            void setTermRIdxPOV(int terminal_ridx) { terminal_ridx_pov = terminal_ridx; }

            int term_LIdxPOV() { return terminal_lidx_pov; }
            void setTermLIdxPOV(int terminal_lidx) { terminal_lidx_pov = terminal_lidx; }

            float term_RDistPOV() { return terminal_rdist_pov; }
            void setTermRDistPOV(float term_rdist) { terminal_rdist_pov = term_rdist; }

            float term_LDistPOV() { return terminal_ldist_pov; }
            void setTermLDistPOV(float term_ldist) { terminal_ldist_pov = term_ldist; }

            int cvx_RIdxPOV() { return convex.convex_ridx_pov; }
            void setCvxRIdxPOV(int cvx_ridx) { convex.convex_ridx_pov = cvx_ridx; }

            int cvx_LIdxPOV() { return convex.convex_lidx_pov; }
            void setCvxLIdxPOV(int cvx_lidx) { convex.convex_lidx_pov = cvx_lidx; }

            float cvx_RDistPOV() { return convex.convex_rdist_pov; }
            void setCvxRDistPOV(float cvx_rdist) { convex.convex_rdist_pov = cvx_rdist; }

            float cvx_LDistPOV() { return convex.convex_ldist_pov; }
            void setCvxLDistPOV(float cvx_ldist) { convex.convex_ldist_pov = cvx_ldist; }

            int cvx_term_RIdxPOV() { return convex.terminal_ridx_pov; }
            void setCvxTermRIdxPOV(int cvx_term_ridx) { convex.terminal_ridx_pov = cvx_term_ridx; }

            int cvx_term_LIdxPOV() { return convex.terminal_lidx_pov; }
            void setCvxTermLIdxPOV(int cvx_term_lidx) { convex.terminal_lidx_pov = cvx_term_lidx; }

            float cvx_term_RDistPOV() { return convex.terminal_rdist_pov; }
            void setCvxTermRDistPOV(float cvx_term_rdist) { convex.terminal_rdist_pov = cvx_term_rdist; }

            float cvx_term_LDistPOV() { return convex.terminal_ldist_pov; }
            void setCvxTermLDistPOV(float cvx_term_ldist) { convex.terminal_ldist_pov = cvx_term_ldist; }

            // Concluding the Gap after constructing with left information
            void addLeftPOVInformation(int left_idx, float ldist) 
            {
                _left_idx_pov = left_idx;
                _ldist_pov = ldist;
                right_type_pov = _rdist_pov < _ldist_pov;

                if (!_axial)
                {
                    _axial = isAxial();
                }

                convex.convex_ridx_pov = _right_idx_pov;
                convex.convex_lidx_pov = _left_idx_pov;
                convex.convex_rdist_pov = _rdist_pov;
                convex.convex_ldist_pov = _ldist_pov;
            }

            void addTerminalRightInformation()
            {
                terminal_right_type_pov = terminal_rdist_pov < terminal_ldist_pov;

                if (!_terminal_axial)
                {
                    _terminal_axial = isAxial();
                }

                convex.terminal_ridx_pov = terminal_ridx_pov;
                convex.terminal_lidx_pov = terminal_lidx_pov;
                convex.terminal_rdist_pov = terminal_rdist_pov;
                convex.terminal_ldist_pov = terminal_ldist_pov;
            }

            // Get Left Cartesian Distance
            void getRCartesianPOV(float &x, float &y)
            {
                x = (_rdist_pov) * cos(-((float) half_scan - _right_idx_pov) / half_scan * M_PI);
                y = (_rdist_pov) * sin(-((float) half_scan - _right_idx_pov) / half_scan * M_PI);
            }

            // Get Right Cartesian Distance
            // edited by Max: float &x, float &y
            void getLCartesianPOV(float &x, float &y)
            {
                x = (_ldist_pov) * cos(-((float) half_scan - _left_idx_pov) / half_scan * M_PI);
                y = (_ldist_pov) * sin(-((float) half_scan - _left_idx_pov) / half_scan * M_PI);
            }

            void getSimplifiedRCartesianPOV(float &x, float &y){
                // std::cout << "convex_ldist: " << convex_ldist << ", convex_lidx: " << convex_lidx << ", half_scan: " << half_scan << std::endl;
                x = (convex.convex_rdist_pov) * cos(-((float) half_scan - convex.convex_ridx_pov) / half_scan * M_PI);
                y = (convex.convex_rdist_pov) * sin(-((float) half_scan - convex.convex_ridx_pov) / half_scan * M_PI);
            }

            void getSimplifiedLCartesianPOV(float &x, float &y){
                // std::cout << "convex_rdist: " << convex_rdist << ", convex_ridx: " << convex_ridx << ", half_scan: " << half_scan << std::endl;
                x = (convex.convex_ldist_pov) * cos(-((float) half_scan - convex.convex_lidx_pov) / half_scan * M_PI);
                y = (convex.convex_ldist_pov) * sin(-((float) half_scan - convex.convex_lidx_pov) / half_scan * M_PI);
            }

            // Decimate Gap 
            void segmentGap2Vec(std::vector<dynamic_gap::Gap>& gap, int min_resoln)
            {
                double gap_idx_size;
                if (_left_idx_pov > _right_idx_pov) {
                    gap_idx_size = (_left_idx_pov - _right_idx_pov);
                } else {
                    gap_idx_size = (_left_idx_pov - _right_idx_pov) + 2*half_scan;
                }

                int num_gaps = gap_idx_size / min_resoln + 1;
                int idx_step = gap_idx_size / num_gaps;
                float dist_step = (_ldist_pov - _rdist_pov) / num_gaps;
                int sub_gap_lidx = _right_idx_pov;
                float sub_gap_ldist = _rdist_pov;
                int sub_gap_ridx = _right_idx_pov;

                if (num_gaps < 3) {
                    gap.push_back(*this);
                    return;
                }
                
                for (int i = 0; i < num_gaps; i++) {
                    Gap detected_gap(_frame, sub_gap_lidx, sub_gap_ldist);
                    // ROS_DEBUG_STREAM("lidx: " << sub_gap_lidx << "ldist: " << sub_gap_ldist);

                    sub_gap_lidx = (sub_gap_lidx + idx_step) % 2*half_scan;
                    sub_gap_ldist += dist_step;
                    // ROS_DEBUG_STREAM("ridx: " << sub_gap_lidx << "rdist: " << sub_gap_ldist);
                    if (i == num_gaps - 1)
                    {
                        detected_gap.addLeftPOVInformation(_left_idx_pov, _ldist_pov);
                    } else {
                        detected_gap.addLeftPOVInformation(sub_gap_lidx - 1, sub_gap_ldist);
                    }
                    gap.push_back(detected_gap);
                }
            }

            void initManipIndices() {
                convex.convex_ridx_pov = _right_idx_pov;
                convex.convex_rdist_pov = _rdist_pov;
                convex.convex_lidx_pov = _left_idx_pov;
                convex.convex_ldist_pov = _ldist_pov;

                convex.terminal_ridx_pov = terminal_ridx_pov;
                convex.terminal_rdist_pov = terminal_rdist_pov;
                convex.terminal_lidx_pov = terminal_lidx_pov;
                convex.terminal_ldist_pov = terminal_ldist_pov;
            }

            bool isAxial(bool initial = true)
            {
                // does resoln here imply 360 deg FOV?
                // ROS_INFO_STREAM("running isAxial");
                int check_r_idx_pov = initial ? _right_idx_pov : terminal_ridx_pov;
                int check_l_idx_pov = initial ? _left_idx_pov : terminal_lidx_pov;
                float check_r_dist_pov = initial ? _rdist_pov : terminal_rdist_pov;
                float check_l_dist_pov = initial ? _ldist_pov : terminal_ldist_pov;

                float resoln = M_PI / half_scan;
                float gap_angle = (check_l_idx_pov - check_r_idx_pov) * resoln;
                if (gap_angle < 0) {
                    gap_angle += 2*M_PI;
                }
                // ROS_INFO_STREAM("gap_angle: " << gap_angle);
                float short_side = right_type_pov ? check_r_dist_pov : check_l_dist_pov;
                // law of cosines
                float opp_side = (float) sqrt(pow(check_r_dist_pov, 2) + pow(check_l_dist_pov, 2) - 2 * check_r_dist_pov * check_l_dist_pov * (float)cos(gap_angle));
                // law of sines
                float small_angle = (float) asin((short_side / opp_side) * (float) sin(gap_angle));
                // ROS_INFO_STREAM("short_side: " << short_side);
                // ROS_INFO_STREAM("opp_side: " << opp_side);
                // ROS_INFO_STREAM("small angle: " << small_angle);
                if (initial) {
                    _axial = (M_PI - small_angle - gap_angle) > 0.75 * M_PI;
                    // std::cout << "checking isSwept: " << _axial << std::endl;
                    return _axial;
                } else {
                    _terminal_axial = (M_PI - small_angle - gap_angle) > 0.75 * M_PI; 
                    // std::cout << "checking isSwept: " << _terminal_axial << std::endl;
                    return _terminal_axial;
                }
            }

            void setRadial()
            {
                _axial = false;
            }

            bool isRightPOVType(bool initial = true)
            {
                if (initial) { 
                    return right_type_pov;
                } else {
                    return terminal_right_type_pov;
                }
            }

            void resetFrame(std::string frame) {
                _frame = frame;
            }

            void setMinSafeDist(float _dist) {
                min_safe_dist = _dist;
            }

            float getMinSafeDist() {
                return min_safe_dist;
            }

            void setTerminalMinSafeDist(float _dist) {
                terminal_min_safe_dist = _dist;
            }

            float getTerminalMinSafeDist() {
                return terminal_min_safe_dist;
            }

            std::string getFrame() {
                return _frame;
            }

            void setCategory(std::string _category) {
                //std::cout << "setting category to: " << _category << std::endl;
                category = _category;
            }

            std::string getCategory() {
                return category;
            }

            void setCrossingPoint(float x, float y) {
                crossing_pt << x,y;
            }

            Eigen::Vector2f getCrossingPoint() {
                return crossing_pt;
            }

            void setClosingPoint(float x, float y) {
                closing_pt << x,y;
            }

            Eigen::Vector2f getClosingPoint() {
                return closing_pt;
            }

            // used in calculating alpha, the angle formed between the two gap lines and the robot. (angle of the gap).
            float get_dist_side() {
                int idx_diff = _left_idx_pov - _right_idx_pov;
                if (idx_diff < 0) {
                    idx_diff += (2*half_scan);
                } 
                return sqrt(pow(_rdist_pov, 2) + pow(_ldist_pov, 2) - 2 * _rdist_pov * _ldist_pov * (cos(float(idx_diff) / float(half_scan) * M_PI)));
            }

            void setTerminalPoints(float _terminal_ridx_pov, float _terminal_rdist_pov, float _terminal_lidx_pov, float _terminal_ldist_pov) {
                terminal_ridx_pov = _terminal_ridx_pov;
                terminal_rdist_pov = _terminal_rdist_pov;
                terminal_lidx_pov = _terminal_lidx_pov;
                terminal_ldist_pov = _terminal_ldist_pov;
                ROS_INFO_STREAM("setting terminal points to, left: (" << terminal_ridx_pov << ", " << terminal_rdist_pov << "), right: (" << terminal_lidx_pov << ", " << terminal_ldist_pov << ")");
                if (terminal_lidx_pov < terminal_ridx_pov) {
                    ROS_INFO_STREAM("potentially incorrect terminal points");
                }
            }

            void printCartesianPoints(bool initial, bool simplified) {
                float x_r_pov,y_r_pov,x_l_pov,y_l_pov;

                if (initial) {
                    if (simplified) {
                        x_r_pov = (_rdist_pov) * cos(-((float) half_scan - _right_idx_pov) / half_scan * M_PI);
                        y_r_pov = (_rdist_pov) * sin(-((float) half_scan - _right_idx_pov) / half_scan * M_PI);
                        x_l_pov = (_ldist_pov) * cos(-((float) half_scan - _left_idx_pov) / half_scan * M_PI);
                        y_l_pov = (_ldist_pov) * sin(-((float) half_scan - _left_idx_pov) / half_scan * M_PI);
                    } else {
                        x_r_pov = (convex.convex_rdist_pov) * cos(-((float) half_scan - convex.convex_ridx_pov) / half_scan * M_PI);
                        y_r_pov = (convex.convex_rdist_pov) * sin(-((float) half_scan - convex.convex_ridx_pov) / half_scan * M_PI);
                        x_l_pov = (convex.convex_ldist_pov) * cos(-((float) half_scan - convex.convex_lidx_pov) / half_scan * M_PI);
                        y_l_pov = (convex.convex_ldist_pov) * sin(-((float) half_scan - convex.convex_lidx_pov) / half_scan * M_PI);
                    }
                } else {
                    if (simplified) {
                        x_r_pov = (terminal_rdist_pov) * cos(-((float) half_scan - terminal_ridx_pov) / half_scan * M_PI);
                        y_r_pov = (terminal_rdist_pov) * sin(-((float) half_scan - terminal_ridx_pov) / half_scan * M_PI);
                        x_l_pov = (terminal_ldist_pov) * cos(-((float) half_scan - terminal_lidx_pov) / half_scan * M_PI);
                        y_l_pov = (terminal_ldist_pov) * sin(-((float) half_scan - terminal_lidx_pov) / half_scan * M_PI);
                    } else {
                        x_r_pov = (convex.terminal_rdist_pov) * cos(-((float) half_scan - convex.terminal_ridx_pov) / half_scan * M_PI);
                        y_r_pov = (convex.terminal_rdist_pov) * sin(-((float) half_scan - convex.terminal_ridx_pov) / half_scan * M_PI);
                        x_l_pov = (convex.terminal_ldist_pov) * cos(-((float) half_scan - convex.terminal_lidx_pov) / half_scan * M_PI);
                        y_l_pov = (convex.terminal_ldist_pov) * sin(-((float) half_scan - convex.terminal_lidx_pov) / half_scan * M_PI);
                    }
                }

                ROS_INFO_STREAM("x_l_pov, y_l_pov: (" << x_l_pov << ", " << y_l_pov << "), x_r_pov,y_r_pov: (" << x_r_pov << ", " << y_r_pov << ")");
            }   
            
            double gap_lifespan = 5.0   ;

            int _right_idx_pov = 0;
            int _left_idx_pov = 511;
            float _rdist_pov = 5;
            float _ldist_pov = 5;
            int terminal_ridx_pov = 0;
            int terminal_lidx_pov = 511;
            float terminal_rdist_pov = 5;
            float terminal_ldist_pov = 5;

            float min_safe_dist = -1;
            float terminal_min_safe_dist = -1;
            Eigen::Vector2f qB;
            Eigen::Vector2f terminal_qB;
            Eigen::Vector2f right_pov_bezier_origin;
            Eigen::Vector2f left_pov_bezier_origin;
            float half_scan = 256;

            std::string _frame = "";
            bool _axial = false;
            bool _terminal_axial = false;
            bool right_type_pov = false;
            bool terminal_right_type_pov = false;

            double peak_velocity_x = 0.0;
            double peak_velocity_y = 0.0;

            struct converted {
                int convex_ridx_pov = 0;
                int convex_lidx_pov = 511;
                float convex_rdist_pov = 5;
                float convex_ldist_pov = 5;
                int terminal_ridx_pov = 0;
                int terminal_lidx_pov = 511;
                float terminal_rdist_pov = 5;
                float terminal_ldist_pov = 5;
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

            cart_model *right_model_pov;
            cart_model *left_model_pov;
            int _index;
            std::string category;
            Eigen::Vector2f crossing_pt;
            Eigen::Vector2f closing_pt;

            bool gap_crossed = false;
            bool gap_closed = false;

            bool pivoted_right_pov = false;
            bool artificial = false;

            double left_weight = 0.0;
            double right_weight = 0.0;
            Eigen::MatrixXd left_right_centers;
        // private:
    };
}

#endif