#pragma once

#include <ros/ros.h>
#include <dynamic_gap/GapFeatureObservation.h>
#include <dynamic_gap/GapFeaturePrediction.h>
#include <dynamic_gap/utils/Gap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread/mutex.hpp>
#include <fstream>
#include <map>
#include <unordered_map>
#include <unordered_set>

namespace dynamic_gap
{
// An endpoint is counted once per raw gap side, including sector boundaries.
struct DensityEndpoint
{
    int scan_index;
    Eigen::Vector2f position;
    float speed;
    bool ground_truth_dynamic;
    bool left;
};

struct DensitySector
{
    int count = 0, dynamic_count = 0, ground_truth_count = 0;
    float radius = 0, angle = 0, area = 0, density = 0, ground_truth_density = 0;
};

bool densityIndexInside(int index, int right, int left, int scan_size);
DensitySector computeDensitySector(const std::vector<DensityEndpoint>& endpoints,
                                   int right, int left, int scan_size, float angle_increment,
                                   float dynamic_threshold = 0.05f);

// Independent from learned velocity. The raw endpoint Kalman models supply inputs.
class GapDensity
{
public:
    void initialize(ros::NodeHandle& topics, ros::NodeHandle& parameters, const std::string& frame);
    void update(const std::vector<Gap*>& raw, const std::vector<Gap*>& simplified,
                const sensor_msgs::LaserScan& scan,
                const std::map<std::string, geometry_msgs::Pose>& agent_poses,
                const std::map<std::string, geometry_msgs::Vector3Stamped>& agent_velocities,
                bool ground_truth_available);
    void prediction(const GapFeaturePrediction& message);
    float cost(int model_id, const ros::Time& now) const;

private:
    struct Estimate { ros::Time stamp; float density; };
    void predictionCB(const GapFeaturePrediction::ConstPtr& message) { prediction(*message); }
    mutable boost::mutex mutex_;
    std::unordered_map<int, Estimate> estimates_;
    std::unordered_set<int> active_ids_;
    ros::Time last_scan_stamp_;
    bool enabled_ = true;
    double weight_ = 5.0, max_age_ = 3.0, max_ground_truth_age_ = 0.5;
    std::string source_ = "ground_truth";
    double dynamic_threshold_ = 0.05, match_threshold_ = 0.6;
    std::string frame_;
    ros::Publisher observations_, markers_;
    ros::Subscriber predictions_;
    std::ofstream csv_;
    unsigned long long sample_ = 0;
};
}
