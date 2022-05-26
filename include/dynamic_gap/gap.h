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
            Gap(std::string frame, int left_idx, float ldist, bool axial = false, float half_scan = 256) : _frame(frame), _left_idx(left_idx), _ldist(ldist), _axial(axial), half_scan(half_scan)
            {};

            ~Gap() {};
            
            void setLIdx(int lidx)
            {
                _left_idx = lidx;
            }

            // Setter and Getter for LR Distance and Index
            void setLDist(float ldist) 
            {
                _ldist = ldist;
            }

            void setRDist(float rdist)
            {
                _rdist = rdist;
            }

            int LIdx()
            {
                return _left_idx;
            }

            int RIdx()
            {
                return _right_idx;
            }

            float LDist()
            {
                return _ldist;
            }

            float RDist()
            {
                return _rdist;
            }

            void getLRIdx(int &l, int &r)
            {
                l = _left_idx;
                r = _right_idx;
            }

            // Concluding the Gap after constructing with left information
            void addRightInformation(int right_idx, float rdist) 
            {
                _right_idx = right_idx;
                _rdist = rdist;
                left_type = _ldist < _rdist;

                if (!_axial)
                {
                    _axial = isAxial();
                }

                convex.convex_lidx = _left_idx;
                convex.convex_ridx = _right_idx;
                convex.convex_ldist = _ldist;
                convex.convex_rdist = _rdist;
            }

            void addTerminalRightInformation()
            {
                terminal_left_type = terminal_ldist < terminal_rdist;

                if (!_terminal_axial)
                {
                    _terminal_axial = isAxial();
                }

                convex.terminal_lidx = terminal_lidx;
                convex.terminal_ridx = terminal_ridx;
                convex.terminal_ldist = terminal_ldist;
                convex.terminal_rdist = terminal_rdist;
            }

            // Get Left Cartesian Distance
            void getLCartesian(float &x, float &y)
            {
                x = (_ldist) * cos(-((float) half_scan - _left_idx) / half_scan * M_PI);
                y = (_ldist) * sin(-((float) half_scan - _left_idx) / half_scan * M_PI);
            }

            // Get Right Cartesian Distance
            // edited by Max: float &x, float &y
            void getRCartesian(float &x, float &y)
            {
                x = (_rdist) * cos(-((float) half_scan - _right_idx) / half_scan * M_PI);
                y = (_rdist) * sin(-((float) half_scan - _right_idx) / half_scan * M_PI);
            }

            void getSimplifiedLCartesian(float &x, float &y){
                // std::cout << "convex_ldist: " << convex_ldist << ", convex_lidx: " << convex_lidx << ", half_scan: " << half_scan << std::endl;
                x = (convex.convex_ldist) * cos(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
                y = (convex.convex_ldist) * sin(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
            }

            void getSimplifiedRCartesian(float &x, float &y){
                // std::cout << "convex_rdist: " << convex_rdist << ", convex_ridx: " << convex_ridx << ", half_scan: " << half_scan << std::endl;
                x = (convex.convex_rdist) * cos(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
                y = (convex.convex_rdist) * sin(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
            }

            void setAGCIdx(int lidx, int ridx) {
                agc_lidx = lidx;
                agc_ridx = ridx;
                int new_l_idx_delta = lidx - _left_idx;
                int new_r_idx_delta = ridx - _left_idx;
                if (new_l_idx_delta < 0) { new_l_idx_delta += int(2*half_scan);}
                if (new_r_idx_delta < 0) { new_r_idx_delta += int(2*half_scan);}
                
                double gap_idx_size;
                if (_right_idx > _left_idx) {
                    gap_idx_size = (_right_idx - _left_idx);
                } else {
                    gap_idx_size = (_right_idx - _left_idx) + 2*half_scan;
                }
                agc_ldist = float(new_l_idx_delta) / float(gap_idx_size) * (_rdist - _ldist) + _ldist;
                agc_rdist = float(new_r_idx_delta) / float(gap_idx_size) * (_rdist - _ldist) + _ldist;
            }

            void getAGCLCartesian(float &x, float &y){
                x = (agc_ldist) * cos(-((float) half_scan - agc_lidx) / half_scan * M_PI);
                y = (agc_ldist) * sin(-((float) half_scan - agc_lidx) / half_scan * M_PI);
            }

            void getAGCRCartesian(float &x, float &y){
                x = (agc_rdist) * cos(-((float) half_scan - agc_ridx) / half_scan * M_PI);
                y = (agc_rdist) * sin(-((float) half_scan - agc_ridx) / half_scan * M_PI);
            }

            // Decimate Gap 
            void segmentGap2Vec(std::vector<dynamic_gap::Gap>& gap, int min_resoln)
            {
                double gap_idx_size;
                if (_right_idx > _left_idx) {
                    gap_idx_size = (_right_idx - _left_idx);
                } else {
                    gap_idx_size = (_right_idx - _left_idx) + 2*half_scan;
                }

                int num_gaps = gap_idx_size / min_resoln + 1;
                int idx_step = gap_idx_size / num_gaps;
                float dist_step = (_rdist - _ldist) / num_gaps;
                int sub_gap_lidx = _left_idx;
                float sub_gap_ldist = _ldist;
                int sub_gap_ridx = _left_idx;

                if (num_gaps < 3) {
                    gap.push_back(*this);
                    return;
                }
                
                for (int i = 0; i < num_gaps; i++) {
                    Gap detected_gap(_frame, sub_gap_lidx, sub_gap_ldist);
                    // ROS_DEBUG_STREAM("lidx: " << sub_gap_lidx << "ldist: " << sub_gap_ldist);
                    if (i != 0) {
                        detected_gap.setLeftObs();
                    }

                    if (i != num_gaps - 1) {
                        detected_gap.setRightObs();
                    }

                    sub_gap_lidx = (sub_gap_lidx + idx_step) % 2*half_scan;
                    sub_gap_ldist += dist_step;
                    // ROS_DEBUG_STREAM("ridx: " << sub_gap_lidx << "rdist: " << sub_gap_ldist);
                    if (i == num_gaps - 1)
                    {
                        detected_gap.addRightInformation(_right_idx, _rdist);
                    } else {
                        detected_gap.addRightInformation(sub_gap_lidx - 1, sub_gap_ldist);
                    }
                    gap.push_back(detected_gap);
                }
            }

            void compareGoalDist(double goal_dist) {
                goal_within = goal_dist < _ldist && goal_dist < _rdist;
            }

            // Getter and Setter for if side is an obstacle
            void setLeftObs() {
                left_obs = false;
            }

            void setRightObs() {
                right_obs = false;
            }

            bool getLeftObs() {
                return left_obs;
            }


            bool getRightObs() {
                return right_obs;
            }

            bool isAxial(bool initial = true)
            {
                // does resoln here imply 360 deg FOV?
                int check_l_idx = initial ? _left_idx : terminal_lidx;
                int check_r_idx = initial ? _right_idx : terminal_ridx;
                float check_l_dist = initial ? _ldist : terminal_ldist;
                float check_r_dist = initial ? _rdist : terminal_rdist;

                float resoln = M_PI / half_scan;
                float gap_angle = (check_r_idx - check_l_idx) * resoln;
                if (gap_angle < 0) {
                    gap_angle += 2*M_PI;
                }
                // std::cout << "gap_angle: " << gap_angle << std::endl;
                float short_side = left_type ? check_l_dist : check_r_dist;
                // law of cosines
                float opp_side = (float) sqrt(pow(check_l_dist, 2) + pow(check_r_dist, 2) - 2 * check_l_dist * check_r_dist * (float)cos(gap_angle));
                // law of sines
                float small_angle = (float) asin(short_side / (opp_side * (float) sin(gap_angle)));
                // std::cout << "small angle: " << small_angle << std::endl;
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

            bool isLeftType(bool initial = true)
            {
                if (initial) { 
                    return left_type;
                } else {
                    return terminal_left_type;
                }
            }

            bool isTerminalLeftType() {
                return terminal_left_type;
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
                int idx_diff = _right_idx - _left_idx;
                if (idx_diff < 0) {
                    idx_diff += (2*half_scan);
                } 
                return sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (cos(float(idx_diff) / float(half_scan) * M_PI)));
            }

            void setTerminalPoints(float _terminal_lidx, float _terminal_ldist, float _terminal_ridx, float _terminal_rdist) {
                terminal_lidx = _terminal_lidx;
                terminal_ldist = _terminal_ldist;
                terminal_ridx = _terminal_ridx;
                terminal_rdist = _terminal_rdist;
            }

            void printCartesianPoints(bool initial, bool simplified) {
                float x1,y1,x2,y2;

                if (initial) {
                    if (simplified) {
                        x1 = (_ldist) * cos(-((float) half_scan - _left_idx) / half_scan * M_PI);
                        y1 = (_ldist) * sin(-((float) half_scan - _left_idx) / half_scan * M_PI);
                        x2 = (_rdist) * cos(-((float) half_scan - _right_idx) / half_scan * M_PI);
                        y2 = (_rdist) * sin(-((float) half_scan - _right_idx) / half_scan * M_PI);
                    } else {
                        x1 = (convex.convex_ldist) * cos(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
                        y1 = (convex.convex_ldist) * sin(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
                        x2 = (convex.convex_rdist) * cos(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
                        y2 = (convex.convex_rdist) * sin(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
                    }
                } else {
                    if (simplified) {
                        x1 = (terminal_ldist) * cos(-((float) half_scan - terminal_lidx) / half_scan * M_PI);
                        y1 = (terminal_ldist) * sin(-((float) half_scan - terminal_lidx) / half_scan * M_PI);
                        x2 = (terminal_rdist) * cos(-((float) half_scan - terminal_ridx) / half_scan * M_PI);
                        y2 = (terminal_rdist) * sin(-((float) half_scan - terminal_ridx) / half_scan * M_PI);
                    } else {
                        x1 = (convex.terminal_ldist) * cos(-((float) half_scan - convex.terminal_lidx) / half_scan * M_PI);
                        y1 = (convex.terminal_ldist) * sin(-((float) half_scan - convex.terminal_lidx) / half_scan * M_PI);
                        x2 = (convex.terminal_rdist) * cos(-((float) half_scan - convex.terminal_ridx) / half_scan * M_PI);
                        y2 = (convex.terminal_rdist) * sin(-((float) half_scan - convex.terminal_ridx) / half_scan * M_PI);
                    }
                }

                ROS_INFO_STREAM("x1,y1: (" << x1 << ", " << y1 << "), x2,y2: (" << x2 << ", " << y2 << ")");
            }   
            
            bool no_valid_slice = false;
            bool goal_within = false;
            bool goal_dir_within = false;
            double gap_lifespan = 5.0   ;
            bool agc = false;

            int _left_idx = 0;
            float _ldist = 5;
            int _right_idx = 511;
            float _rdist = 5;
            bool wrap = false;
            bool reduced = false;
            bool convexified = false;
            int convex_lidx;
            int convex_ridx;
            float convex_ldist;
            float convex_rdist;
            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            Eigen::Vector2f terminal_qB;
            float half_scan = 256;

            int agc_lidx;
            int agc_ridx;
            float agc_ldist;
            float agc_rdist;
            bool no_agc_coor = false;

            std::string _frame = "";
            bool left_obs = true;
            bool right_obs = true;
            bool _axial = false;
            bool _terminal_axial = false;
            bool left_type = false;
            bool terminal_left_type = false;

            int swept_lidx = 0;
            int swept_ridx = 511;
            float swept_ldist = 5;
            float swept_rdist = 5;

            struct converted {
                int convex_lidx = 0;
                int convex_ridx = 511;
                float convex_ldist = 5;
                float convex_rdist = 5;
                int terminal_lidx = 0;
                int terminal_ridx = 511;
                float terminal_ldist = 5;
                float terminal_rdist = 5;
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

            cart_model *left_model;
            cart_model *right_model;
            int _index;
            std::string category;
            Eigen::Vector2f crossing_pt;
            Eigen::Vector2f closing_pt;

            bool gap_crossed = false;
            bool gap_closed = false;
            int terminal_lidx = 0;
            int terminal_ridx = 511;
            float terminal_ldist = 5.0;
            float terminal_rdist = 5.0;

            bool gap_chosen = false;
            bool pivoted_left = false;
        // private:
    };
}

#endif