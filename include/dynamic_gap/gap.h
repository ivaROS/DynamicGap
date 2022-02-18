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
#include <dynamic_gap/mp_model.h>

namespace dynamic_gap
{
    class Gap
    {
        public:
            Gap() {};

            // colon used here is an initialization list. helpful for const variables.
            Gap(std::string frame, int left_idx, float ldist, bool axial = false, float half_scan = 256) : _frame(frame), _left_idx(left_idx), _ldist(ldist), _axial(axial), half_scan(half_scan)
            {
                left_model = new dynamic_gap::MP_model("left");
                right_model = new dynamic_gap::MP_model("right");
            };
            /*
            Gap(const dynamic_gap::Gap& gap) {
                convex.convex_lidx = gap.convex.convex_lidx;
                convex.convex_ridx = gap.convex.convex_ridx;
                convex.convex_ldist = gap.convex.convex_ldist;
                convex.convex_rdist = gap.convex.convex_rdist;

                left_model = gap.left_model;
                right_model = gap.right_model;
                // set indices and distances
                // copy models
                // copy some sort of gap index to check against alter
            }
            */

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
                    float resoln = M_PI / half_scan;
                    float angle1 = (_right_idx - _left_idx) * resoln;
                    float short_side = left_type ? _ldist : _rdist;
                    float opp_side = (float) sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (float)cos(angle1));
                    // checking if alpha of gap is greater than 3*pi/4
                    float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                    if (M_PI - small_angle - angle1 > 0.75 * M_PI) 
                    {
                        _axial = true;
                    }
                }

                convex.convex_lidx = _left_idx;
                convex.convex_ridx = _right_idx;
                convex.convex_ldist = _ldist;
                convex.convex_rdist = _rdist;
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

            void getRadialExLCartesian(float &x, float &y){
                // std::cout << "convex_ldist: " << convex_ldist << ", convex_lidx: " << convex_lidx << ", half_scan: " << half_scan << std::endl;
                x = (convex_ldist) * cos(-((float) half_scan - convex_lidx) / half_scan * M_PI);
                y = (convex_ldist) * sin(-((float) half_scan - convex_lidx) / half_scan * M_PI);
            }

            void getRadialExRCartesian(float &x, float &y){
                // std::cout << "convex_rdist: " << convex_rdist << ", convex_ridx: " << convex_ridx << ", half_scan: " << half_scan << std::endl;
                x = (convex_rdist) * cos(-((float) half_scan - convex_ridx) / half_scan * M_PI);
                y = (convex_rdist) * sin(-((float) half_scan - convex_ridx) / half_scan * M_PI);
            }

            void setAGCIdx(int lidx, int ridx) {
                agc_lidx = lidx;
                agc_ridx = ridx;
                agc_ldist = float(lidx - _left_idx) / float(_right_idx - _left_idx) * (_rdist - _ldist) + _ldist;
                agc_rdist = float(ridx - _left_idx) / float(_right_idx - _left_idx) * (_rdist - _ldist) + _ldist;
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
                int num_gaps = (_right_idx - _left_idx) / min_resoln + 1;
                int idx_step = (_right_idx - _left_idx) / num_gaps;
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

                    sub_gap_lidx += idx_step;
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

            bool isAxial()
            {
                // does resoln here imply 360 deg FOV?
                float resoln = M_PI / half_scan;
                float angle1 = (_right_idx - _left_idx) * resoln;
                float short_side = left_type ? _ldist : _rdist;
                float opp_side = (float) sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (float)cos(angle1));
                float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                _axial = (M_PI - small_angle - angle1 > 0.75 * M_PI); 
                return _axial;
            }

            void setRadial()
            {
                _axial = false;
            }

            bool isLeftType()
            {
                return left_type;
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

            // used in calculating alpha, the angle formed between the two gap lines and the robot. (angle of the gap).
            float get_dist_side() {
                return sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (cos(float(_right_idx - _left_idx) / float(half_scan) * M_PI)));
            }
            
            bool no_valid_slice = false;
            bool goal_within = false;
            bool goal_dir_within = false;
            float life_time = 1.0;
            double gap_lifespan = 0.0;
            bool agc = false;

            int _left_idx = 0;
            float _ldist = 3;
            int _right_idx = 511;
            float _rdist = 3;
            bool wrap = false;
            bool reduced = false;
            bool convexified = false;
            int convex_lidx;
            int convex_ridx;
            float convex_ldist;
            float convex_rdist;
            float min_safe_dist = -1;
            Eigen::Vector2f qB;
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
            bool left_type = false;

            int swept_convex_lidx = 0;
            int swept_convex_ridx = 0;
            float swept_convex_ldist = 3;
            float swept_convex_rdist = 3;

            struct converted {
                int convex_lidx = 0;
                int convex_ridx = 511;
                float convex_ldist = 3;
                float convex_rdist = 3;
            } convex;

            struct GapMode {
                bool reduced = false;
                bool convex = false;
                bool agc = false;
            } mode;

            struct Goal {
                float x, y;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } goal;

            MP_model *left_model;
            MP_model *right_model;
        // private:
    };
}

#endif