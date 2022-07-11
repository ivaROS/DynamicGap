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
            Gap(std::string frame, int right_idx, float rdist, bool axial = false, float half_scan = 256) : 
                _frame(frame), _right_idx(right_idx), _rdist(rdist), _axial(axial), half_scan(half_scan)
            {
                qB << 0.0, 0.0;
                terminal_qB << 0.0, 0.0;
                right_bezer_origin << 0.0, 0.0;
                left_bezier_origin << 0.0, 0.0;
            };

            ~Gap() {};
            
            // Setters and Getters for LR Distance and Index (initial and terminal gaps)
            int RIdx() { return _right_idx; }
            void setRIdx(int ridx) { _right_idx = ridx; }

            int LIdx() { return _left_idx; }
            void setLIdx(int lidx) { _left_idx = lidx; }

            float RDist() { return _rdist; }
            void setRDist(float rdist) { _rdist = rdist; }

            float LDist() { return _ldist; }
            void setLDist(float ldist) { _ldist = ldist; }

            int term_RIdx() { return terminal_ridx; }
            void setTermRIdx(int terminal_ridx) { terminal_ridx = terminal_ridx; }

            int term_LIdx() { return terminal_lidx; }
            void setTermLIdx(int terminal_lidx) { terminal_lidx = terminal_lidx; }

            float term_RDist() { return terminal_rdist; }
            void setTermRDist(float term_rdist) { terminal_rdist = term_rdist; }

            float term_LDist() { return terminal_ldist; }
            void setTermLDist(float term_ldist) { terminal_ldist = term_ldist; }

            int cvx_RIdx() { return convex.convex_ridx; }
            void setCvxRIdx(int cvx_ridx) { convex.convex_ridx = cvx_ridx; }

            int cvx_LIdx() { return convex.convex_lidx; }
            void setCvxLIdx(int cvx_lidx) { convex.convex_lidx = cvx_lidx; }

            float cvx_RDist() { return convex.convex_rdist; }
            void setCvxRDist(float cvx_rdist) { convex.convex_rdist = cvx_rdist; }

            float cvx_LDist() { return convex.convex_ldist; }
            void setCvxLDist(float cvx_ldist) { convex.convex_ldist = cvx_ldist; }

            int cvx_term_RIdx() { return convex.terminal_ridx; }
            void setCvxTermRIdx(int cvx_term_ridx) { convex.terminal_ridx = cvx_term_ridx; }

            int cvx_term_LIdx() { return convex.terminal_lidx; }
            void setCvxTermLIdx(int cvx_term_lidx) { convex.terminal_lidx = cvx_term_lidx; }

            float cvx_term_RDist() { return convex.terminal_rdist; }
            void setCvxTermRDist(float cvx_term_rdist) { convex.terminal_rdist = cvx_term_rdist; }

            float cvx_term_LDist() { return convex.terminal_ldist; }
            void setCvxTermLDist(float cvx_term_ldist) { convex.terminal_ldist = cvx_term_ldist; }

            // Concluding the Gap after constructing with left information
            void addLeftInformation(int left_idx, float ldist) 
            {
                _left_idx = left_idx;
                _ldist = ldist;
                right_type = _rdist < _ldist;

                if (!_axial)
                {
                    _axial = isAxial();
                }

                convex.convex_ridx = _right_idx;
                convex.convex_lidx = _left_idx;
                convex.convex_rdist = _rdist;
                convex.convex_ldist = _ldist;
            }

            void addTerminalRightInformation()
            {
                terminal_right_type = terminal_rdist < terminal_ldist;

                if (!_terminal_axial)
                {
                    _terminal_axial = isAxial();
                }

                convex.terminal_ridx = terminal_ridx;
                convex.terminal_lidx = terminal_lidx;
                convex.terminal_rdist = terminal_rdist;
                convex.terminal_ldist = terminal_ldist;
            }

            // Get Left Cartesian Distance
            void getRCartesian(float &x, float &y)
            {
                x = (_rdist) * cos(-((float) half_scan - _right_idx) / half_scan * M_PI);
                y = (_rdist) * sin(-((float) half_scan - _right_idx) / half_scan * M_PI);
            }

            // Get Right Cartesian Distance
            // edited by Max: float &x, float &y
            void getLCartesian(float &x, float &y)
            {
                x = (_ldist) * cos(-((float) half_scan - _left_idx) / half_scan * M_PI);
                y = (_ldist) * sin(-((float) half_scan - _left_idx) / half_scan * M_PI);
            }

            void getSimplifiedRCartesian(float &x, float &y){
                // std::cout << "convex_ldist: " << convex_ldist << ", convex_lidx: " << convex_lidx << ", half_scan: " << half_scan << std::endl;
                x = (convex.convex_rdist) * cos(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
                y = (convex.convex_rdist) * sin(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
            }

            void getSimplifiedLCartesian(float &x, float &y){
                // std::cout << "convex_rdist: " << convex_rdist << ", convex_ridx: " << convex_ridx << ", half_scan: " << half_scan << std::endl;
                x = (convex.convex_ldist) * cos(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
                y = (convex.convex_ldist) * sin(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
            }

            // Decimate Gap 
            void segmentGap2Vec(std::vector<dynamic_gap::Gap>& gap, int min_resoln)
            {
                double gap_idx_size;
                if (_left_idx > _right_idx) {
                    gap_idx_size = (_left_idx - _right_idx);
                } else {
                    gap_idx_size = (_left_idx - _right_idx) + 2*half_scan;
                }

                int num_gaps = gap_idx_size / min_resoln + 1;
                int idx_step = gap_idx_size / num_gaps;
                float dist_step = (_ldist - _rdist) / num_gaps;
                int sub_gap_lidx = _right_idx;
                float sub_gap_ldist = _rdist;
                int sub_gap_ridx = _right_idx;

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
                        detected_gap.addLeftInformation(_left_idx, _ldist);
                    } else {
                        detected_gap.addLeftInformation(sub_gap_lidx - 1, sub_gap_ldist);
                    }
                    gap.push_back(detected_gap);
                }
            }

            void initManipIndices() {
                convex.convex_ridx = _right_idx;
                convex.convex_rdist = _rdist;
                convex.convex_lidx = _left_idx;
                convex.convex_ldist = _ldist;

                convex.terminal_ridx = terminal_ridx;
                convex.terminal_rdist = terminal_rdist;
                convex.terminal_lidx = terminal_lidx;
                convex.terminal_ldist = terminal_ldist;
            }

            bool isAxial(bool initial = true)
            {
                // does resoln here imply 360 deg FOV?
                // ROS_INFO_STREAM("running isAxial");
                int check_r_idx = initial ? _right_idx : terminal_ridx;
                int check_l_idx = initial ? _left_idx : terminal_lidx;
                float check_r_dist = initial ? _rdist : terminal_rdist;
                float check_l_dist = initial ? _ldist : terminal_ldist;

                float resoln = M_PI / half_scan;
                float gap_angle = (check_l_idx - check_r_idx) * resoln;
                if (gap_angle < 0) {
                    gap_angle += 2*M_PI;
                }
                // ROS_INFO_STREAM("gap_angle: " << gap_angle);
                float short_side = right_type ? check_r_dist : check_l_dist;
                // law of cosines
                float opp_side = (float) sqrt(pow(check_r_dist, 2) + pow(check_l_dist, 2) - 2 * check_r_dist * check_l_dist * (float)cos(gap_angle));
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

            bool isRightType(bool initial = true)
            {
                if (initial) { 
                    return right_type;
                } else {
                    return terminal_right_type;
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
                ROS_INFO_STREAM("setting crossing point to: " << x << ", " << y);
                crossing_pt << x,y;
            }

            Eigen::Vector2f getCrossingPoint() {
                return crossing_pt;
            }

            void setClosingPoint(float x, float y) {
                ROS_INFO_STREAM("setting closing point to: " << x << ", " << y);
                closing_pt << x,y;
            }

            Eigen::Vector2f getClosingPoint() {
                return closing_pt;
            }

            // used in calculating alpha, the angle formed between the two gap lines and the robot. (angle of the gap).
            float get_dist_side() {
                int idx_diff = _left_idx - _right_idx;
                if (idx_diff < 0) {
                    idx_diff += (2*half_scan);
                } 
                return sqrt(pow(_rdist, 2) + pow(_ldist, 2) - 2 * _rdist * _ldist * (cos(float(idx_diff) / float(half_scan) * M_PI)));
            }

            void setTerminalPoints(float _terminal_ridx, float _terminal_rdist, float _terminal_lidx, float _terminal_ldist) {
                terminal_ridx = _terminal_ridx;
                terminal_rdist = _terminal_rdist;
                terminal_lidx = _terminal_lidx;
                terminal_ldist = _terminal_ldist;
                ROS_INFO_STREAM("setting terminal points to, left: (" << terminal_ridx << ", " << terminal_rdist << "), right: (" << terminal_lidx << ", " << terminal_ldist << ")");
                if (terminal_lidx < terminal_ridx) {
                    ROS_INFO_STREAM("potentially incorrect terminal points");
                }
            }

            void printCartesianPoints(bool initial, bool simplified) {
                float x_r,y_r,x_l,y_l;

                if (initial) {
                    if (simplified) {
                        x_r = (_rdist) * cos(-((float) half_scan - _right_idx) / half_scan * M_PI);
                        y_r = (_rdist) * sin(-((float) half_scan - _right_idx) / half_scan * M_PI);
                        x_l = (_ldist) * cos(-((float) half_scan - _left_idx) / half_scan * M_PI);
                        y_l = (_ldist) * sin(-((float) half_scan - _left_idx) / half_scan * M_PI);
                    } else {
                        x_r = (convex.convex_rdist) * cos(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
                        y_r = (convex.convex_rdist) * sin(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
                        x_l = (convex.convex_ldist) * cos(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
                        y_l = (convex.convex_ldist) * sin(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
                    }
                } else {
                    if (simplified) {
                        x_r = (terminal_rdist) * cos(-((float) half_scan - terminal_ridx) / half_scan * M_PI);
                        y_r = (terminal_rdist) * sin(-((float) half_scan - terminal_ridx) / half_scan * M_PI);
                        x_l = (terminal_ldist) * cos(-((float) half_scan - terminal_lidx) / half_scan * M_PI);
                        y_l = (terminal_ldist) * sin(-((float) half_scan - terminal_lidx) / half_scan * M_PI);
                    } else {
                        x_r = (convex.terminal_rdist) * cos(-((float) half_scan - convex.terminal_ridx) / half_scan * M_PI);
                        y_r = (convex.terminal_rdist) * sin(-((float) half_scan - convex.terminal_ridx) / half_scan * M_PI);
                        x_l = (convex.terminal_ldist) * cos(-((float) half_scan - convex.terminal_lidx) / half_scan * M_PI);
                        y_l = (convex.terminal_ldist) * sin(-((float) half_scan - convex.terminal_lidx) / half_scan * M_PI);
                    }
                }

                ROS_INFO_STREAM("x_l, y_l: (" << x_l << ", " << y_l << "), x_r,y_r: (" << x_r << ", " << y_r << ")");
            }   
            
            double gap_lifespan = 5.0   ;

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
            bool _axial = false;
            bool _terminal_axial = false;
            bool right_type = false;
            bool terminal_right_type = false;

            double peak_velocity_x = 0.0;
            double peak_velocity_y = 0.0;

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

            cart_model *right_model;
            cart_model *left_model;
            int _index;
            std::string category;
            Eigen::Vector2f crossing_pt;
            Eigen::Vector2f closing_pt;

            bool gap_crossed = false;
            bool gap_closed = false;

            bool pivoted_right = false;
            bool artificial = false;

            double left_weight = 0.0;
            double right_weight = 0.0;
            Eigen::MatrixXd left_right_centers;
        // private:
    };
}

#endif