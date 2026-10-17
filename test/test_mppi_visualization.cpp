#include <dynamic_gap/visualization/TrajectoryVisualizer.h>
#include <gtest/gtest.h>

#include <map>
#include <set>

namespace dynamic_gap
{
namespace
{
geometry_msgs::PoseArray makePath(double y)
{
    geometry_msgs::PoseArray path;
    path.header.frame_id = "laser";
    for (int i = 0; i < 6; ++i)
    {
        geometry_msgs::Pose pose;
        pose.position.x = 0.2 * i;
        pose.position.y = y;
        pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    return path;
}

std::vector<MppiGapVisualization> makeGaps()
{
    std::vector<MppiGapVisualization> gaps(4);
    for (std::size_t i = 0; i < gaps.size(); ++i)
    {
        auto& gap = gaps[i];
        gap.gap_index = static_cast<int>(i);
        gap.left_id = static_cast<int>(i);
        gap.right_id = static_cast<int>(i + 1);
        gap.nominal = makePath(0.5 * i);
        gap.cost = 10.0 + i;
        for (int j = 0; j < 5; ++j)
            gap.samples.push_back(makePath(0.5 * i + 0.02 * j));
        gap.sample_feasible = {true, true, false, true, true};
        gap.total_samples = 5;
        gap.feasible_samples = 4;
        gap.progress_samples = 3;
    }
    return gaps;
}

std::vector<std_msgs::ColorRGBA> makeColors()
{
    std::vector<std_msgs::ColorRGBA> colors(4);
    for (std::size_t i = 0; i < colors.size(); ++i)
    {
        colors[i].r = 0.2 * (i + 1);
        colors[i].g = 1.0 - colors[i].r;
        colors[i].b = 0.5;
        colors[i].a = 1.0;
    }
    return colors;
}

TEST(MppiVisualization, AllFourGapsShareOneFrameWithUniqueSampleMarkers)
{
    const auto gaps = makeGaps();
    MppiMarkerKeys previous;
    const ros::Time stamp(123, 0);
    const auto markers = buildMppiMarkers(gaps, makeColors(), stamp, previous);
    ASSERT_EQ(markers.markers.size(), 4 * (5 + 1 + 1));
    MppiMarkerKeys unique;
    std::set<float> sample_colors;
    std::size_t samples = 0, nominals = 0, labels = 0;
    for (const auto& marker : markers.markers)
    {
        EXPECT_TRUE(unique.emplace(marker.ns, marker.id).second);
        EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);
        EXPECT_EQ(marker.header.frame_id, "laser");
        EXPECT_EQ(marker.header.stamp, stamp);
        EXPECT_GT(marker.lifetime.toSec(), 0.0);
        if (marker.ns.find("/samples") != std::string::npos)
        {
            ++samples;
            sample_colors.insert(marker.color.r);
            EXPECT_EQ(marker.points.size(), 6u);
            EXPECT_LT(marker.scale.x, 0.02);
            EXPECT_LT(marker.color.a, 1.0);
        }
        else if (marker.ns.find("/nominal") != std::string::npos)
        {
            ++nominals;
            EXPECT_GT(marker.scale.x, 0.05);
            EXPECT_FLOAT_EQ(marker.color.a, 1.0);
        }
        else
        {
            ++labels;
            EXPECT_NE(marker.text.find("samples=5 drawn=5 feasible=4 progress=3"), std::string::npos);
        }
    }
    EXPECT_EQ(samples, 20u);
    EXPECT_EQ(nominals, 4u);
    EXPECT_EQ(labels, 4u);
    EXPECT_EQ(sample_colors.size(), 4u);
    EXPECT_EQ(unique, previous);
}

TEST(MppiVisualization, RemovedGapsAndShrunkPopulationsHaveScopedDeletes)
{
    auto gaps = makeGaps();
    MppiMarkerKeys previous;
    const auto initial = buildMppiMarkers(gaps, makeColors(), ros::Time(123, 0), previous);
    std::map<std::pair<std::string, int>, visualization_msgs::Marker> displayed;
    for (const auto& marker : initial.markers)
        displayed[{marker.ns, marker.id}] = marker;

    gaps.pop_back();
    gaps.front().samples.resize(2);
    const auto next = buildMppiMarkers(gaps, makeColors(), ros::Time(124, 0), previous);
    std::size_t deletes = 0;
    for (const auto& marker : next.markers)
    {
        EXPECT_NE(marker.action, visualization_msgs::Marker::DELETEALL);
        const auto key = std::make_pair(marker.ns, marker.id);
        if (marker.action == visualization_msgs::Marker::DELETE)
        {
            ++deletes;
            EXPECT_EQ(displayed.erase(key), 1u);
        }
        else
            displayed[key] = marker;
    }
    EXPECT_EQ(deletes, 7u + 3u);
    EXPECT_EQ(displayed.size(), 3u * 7u - 3u);
    EXPECT_EQ(displayed.size(), previous.size());

    const auto empty = buildMppiMarkers({}, makeColors(), ros::Time(125, 0), previous);
    EXPECT_EQ(empty.markers.size(), displayed.size());
    for (const auto& marker : empty.markers)
    {
        EXPECT_EQ(marker.action, visualization_msgs::Marker::DELETE);
        EXPECT_EQ(displayed.erase({marker.ns, marker.id}), 1u);
    }
    EXPECT_TRUE(displayed.empty());
    EXPECT_TRUE(previous.empty());
}

TEST(MppiVisualization, DisplaySubsamplingRetainsAllGapStatisticsAndPathEnds)
{
    const auto gaps = makeGaps();
    MppiMarkerKeys previous;
    const auto markers = buildMppiMarkers(gaps, makeColors(), ros::Time(123, 0), previous, 2, 4);
    ASSERT_EQ(markers.markers.size(), 4 * (2 + 1 + 1));
    for (const auto& marker : markers.markers)
    {
        if (marker.type == visualization_msgs::Marker::LINE_STRIP)
        {
            ASSERT_EQ(marker.points.size(), 3u);
            EXPECT_DOUBLE_EQ(marker.points.front().x, 0.0);
            EXPECT_DOUBLE_EQ(marker.points.back().x, 1.0);
        }
        else
            EXPECT_NE(marker.text.find("samples=5 drawn=2 feasible=4 progress=3"), std::string::npos);
    }
}
}  // namespace
}  // namespace dynamic_gap
