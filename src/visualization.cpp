#include <dynamic_gap/visualization.h>

namespace dynamic_gap{
    GapVisualizer::GapVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg) {
        initialize(nh, cfg);
    }

    void GapVisualizer::initialize(ros::NodeHandle& nh, const DynamicGapConfig& cfg) {
        cfg_ = &cfg;
        gaparc_publisher = nh.advertise<visualization_msgs::MarkerArray>("pg_arcs", 1000);
        gapside_publisher = nh.advertise<visualization_msgs::MarkerArray>("pg_sides", 100);
        gapgoal_publisher = nh.advertise<visualization_msgs::MarkerArray>("pg_markers", 10);
        gapmodel_publisher = nh.advertise<visualization_msgs::MarkerArray>("dg_models", 10);

        std_msgs::ColorRGBA std_color;
        std::vector<std_msgs::ColorRGBA> raw_radial;
        std::vector<std_msgs::ColorRGBA> raw_axial;
        std::vector<std_msgs::ColorRGBA> fin_radial;
        std::vector<std_msgs::ColorRGBA> fin_axial;
        std::vector<std_msgs::ColorRGBA> extent;
        std::vector<std_msgs::ColorRGBA> agc;
        std::vector<std_msgs::ColorRGBA> gap_model;

        // Raw Therefore Alpha halved
        std_color.a = 0.5;
        std_color.r = 0.7;
        std_color.g = 0.1;
        std_color.b = 0.5;
        raw_radial.push_back(std_color);
        raw_radial.push_back(std_color);
        std_color.r = 0.6;
        std_color.g = 0.2;
        std_color.b = 0.1;
        raw_axial.push_back(std_color);
        raw_axial.push_back(std_color);
        std_color.a = 1;
        std_color.r = 1;
        std_color.g = 0.9;
        std_color.b = 0.3;
        fin_radial.push_back(std_color);
        fin_radial.push_back(std_color);
        std_color.r = 0.4;
        std_color.g = 0;
        std_color.b = 0.9;
        fin_axial.push_back(std_color);
        fin_axial.push_back(std_color);
        std_color.r = 0;
        std_color.g = 1;
        std_color.b = 0;
        extent.push_back(std_color);
        extent.push_back(std_color);
        std_color.r = 1;
        std_color.g = 0;
        std_color.b = 0;
        agc.push_back(std_color);
        agc.push_back(std_color);
        std_color.r = 1;
        std_color.g = 0;
        std_color.b = 1;
        gap_model.push_back(std_color);
        gap_model.push_back(std_color);
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw_axial", raw_axial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw_radial", raw_radial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_radial", fin_radial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_axial", fin_axial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_extent", extent));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_agc", agc));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("gap_model", gap_model));
    }

    void GapVisualizer::drawGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g, std::string ns, std::string color) {
        // ROS_INFO_STREAM(g._left_idx << ", " << g._ldist << ", " << g._right_idx << ", " << g._rdist << ", " << g._frame);
        if (!cfg_->gap_viz.debug_viz) return;
        // std::cout << "in draw gap" << std::endl;
        int viz_offset = 0;
        double viz_jitter = cfg_->gap_viz.viz_jitter;
        if (viz_jitter > 0 && g.isAxial()){
            viz_offset = g.isLeftType() ? -2 : 2;
        }

        int num_gaps = (g._right_idx - g._left_idx) / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (g._rdist - g._ldist) / num_gaps;
        int sub_gap_lidx = g._left_idx + viz_offset;
        float sub_gap_ldist = g._ldist;

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = ns;
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        std::string local_ns = ns;
        if (g.isAxial()) {
            local_ns.append("_axial");
        } else {
            local_ns.append("_radial");
        }

        auto color_value = colormap.find(local_ns);
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.1;
        this_marker.scale.z = 0.1;
        bool finNamespace = (ns.compare("fin") == 0);

        geometry_msgs::Point linel;
        geometry_msgs::Point liner;
        std::vector<geometry_msgs::Point> lines;

        if (finNamespace) {
            this_marker.colors.at(0).a = 1;
            this_marker.colors.at(1).a = 1;
            linel.z = 0.1;
            liner.z = 0.1;
        }

        int id = (int) vis_arr.markers.size();

        for (int i = 0; i < num_gaps - 1; i++)
        {
            lines.clear();
            linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            sub_gap_lidx += cfg_->gap_viz.min_resoln;
            sub_gap_ldist += dist_step;
            liner.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            liner.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            this_marker.lifetime = ros::Duration(g.life_time);
            if (i == 0 || i == (num_gaps - 2)) {
                this_marker.scale.x = 10*thickness;
            } else {
                this_marker.scale.x = thickness;
            }
            vis_arr.markers.push_back(this_marker);
        }

        // close the last
        this_marker.scale.x = 10*thickness;
        lines.clear();
        linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        liner.x = (g._rdist + viz_jitter) * cos(-( (float) g.half_scan - g._right_idx) / g.half_scan * M_PI);
        liner.y = (g._rdist + viz_jitter) * sin(-( (float) g.half_scan - g._right_idx) / g.half_scan * M_PI);
        lines.push_back(linel);
        lines.push_back(liner);
        this_marker.points = lines;
        this_marker.id = id++;
        this_marker.lifetime = ros::Duration(g.life_time);
        vis_arr.markers.push_back(this_marker);
    }
    
    void GapVisualizer::drawGaps(std::vector<dynamic_gap::Gap> g, std::string ns, std::string color) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;
        for (auto & gap : g) {
            drawGap(vis_arr, gap, ns);
        }
        gaparc_publisher.publish(vis_arr);
    }

    void GapVisualizer::drawGapsModels(std::vector<dynamic_gap::Gap> g) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;
        for (auto & gap : g) {
            drawGapModels(vis_arr, gap, "gap_models");
        }
        gapmodel_publisher.publish(vis_arr);
    }

    void GapVisualizer::drawGapModels(visualization_msgs::MarkerArray & model_arr,dynamic_gap::Gap g, std::string ns) {
        int model_id = (int) model_arr.markers.size();
        
        visualization_msgs::Marker model_pts;
        model_pts.header.frame_id = g._frame;
        model_pts.header.stamp = ros::Time();
        model_pts.ns = ns;
        model_pts.type = visualization_msgs::Marker::CYLINDER;
        model_pts.action = visualization_msgs::Marker::ADD;
        
        auto model_color_value = colormap.find("gap_model");
        if (model_color_value == colormap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }
        model_pts.colors = model_color_value->second;
        std::cout << "model pts colors: " << model_pts.colors.r << std::endl;
        model_pts.scale.x = 0.05;
        model_pts.scale.y = 0.1;
        model_pts.scale.z = 0.1;

        std::vector<geometry_msgs::Point> left_right_pts;
        geometry_msgs::Point left_pt;
        left_pt.x = g.left_model->get_cartesian_state()[0];
        left_pt.y = g.left_model->get_cartesian_state()[1];
        left_pt.z = 0.5;
        std::cout << "left point: " << left_pt.x << ", " << left_pt.y << ", " << left_pt.z << std::endl;
        geometry_msgs::Point right_pt;
        right_pt.x = g.right_model->get_cartesian_state()[0];
        right_pt.y = g.right_model->get_cartesian_state()[1];
        right_pt.z = 0.5;
        std::cout << "right point: " << right_pt.x << ", " << right_pt.y << ", " << right_pt.z << std::endl;
        
        model_pts.lifetime = ros::Duration(0.25);

        left_right_pts.push_back(left_pt);
        left_right_pts.push_back(right_pt);
        model_pts.points = left_right_pts;
        model_pts.id = model_id++;
        model_arr.markers.push_back(model_pts);
    }

    void GapVisualizer::drawManipGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g, bool & circle) {
        // if AGC: Color is Red
        // if Convex: color is Brown, viz_jitter + 0.1
        // if RadialExtension: color is green, draw additional circle
        if (!cfg_->gap_viz.debug_viz) return;

        if (!g.mode.reduced && !g.mode.convex && !g.mode.agc) {
            return;
        }

        float viz_jitter = (float) cfg_->gap_viz.viz_jitter;
        int viz_offset = 0;
        if (viz_jitter > 0 && g.isAxial()){
            viz_offset = g.isLeftType() ? -2 : 2;
        }

        std::string ns;
        if (g.mode.reduced) {
            ns = "fin_radial";
            viz_jitter += 0.1;
        }
        
        if (g.mode.convex) {
            ns = "fin_extent";
        }

        if (g.mode.agc) {
            ns = "fin_agc";
        }

        
        // num gaps really means segments within a gap
        int num_segments = std::abs(g.convex.convex_ridx - g.convex.convex_lidx) / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (g.convex.convex_rdist - g.convex.convex_ldist) / num_segments;
        int sub_gap_lidx = g.convex.convex_lidx + viz_offset;
        float sub_gap_ldist = g.convex.convex_ldist;

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = ns;
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        auto color_value = colormap.find(ns);
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("[drawManipGaps] Visualization Color not found, return without drawing");
            return;
        }

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.1;
        this_marker.scale.z = 0.1;

        geometry_msgs::Point linel;
        geometry_msgs::Point liner;
        liner.z = 0.5;
        linel.z = 0.5;
        std::vector<geometry_msgs::Point> lines;

        int id = (int) vis_arr.markers.size();
        // ROS_INFO_STREAM("ID: "<< id);

        this_marker.lifetime = ros::Duration(0.25);

        for (int i = 0; i < num_segments - 1; i++)
        {
            lines.clear();
            linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            sub_gap_lidx = (sub_gap_lidx + cfg_->gap_viz.min_resoln) % int(g.half_scan * 2);
            sub_gap_ldist += dist_step;
            liner.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            liner.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            if (i == 0 || i == (num_segments - 2)) {
                this_marker.scale.x = 10*thickness;
            } else {
                this_marker.scale.x = thickness;
            }
            vis_arr.markers.push_back(this_marker);
        }

        // this_marker.scale.x = 10*thickness;
        // close the last
        lines.clear();
        linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        liner.x = (g.convex.convex_rdist + viz_jitter) * cos(-( (float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
        liner.y = (g.convex.convex_rdist + viz_jitter) * sin(-( (float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
        lines.push_back(linel);
        lines.push_back(liner);
        this_marker.scale.x = thickness;
        this_marker.points = lines;
        this_marker.id = id++;
        this_marker.lifetime = ros::Duration(0.25);
        vis_arr.markers.push_back(this_marker);
        
        // MAX: removing just for visualizing
        /*
        if (g.mode.convex) {
            float r = g.getMinSafeDist();
            if (r < 0) {
                ROS_WARN_STREAM("Gap min safe dist not recorded");
            }

            auto convex_color = colormap["fin_extent"];

            this_marker.ns = "fin_extent";

            // The Circle
            if (!circle) {
                for (int i = 0; i < 50; i++) {
                    lines.clear();
                    linel.x = r * cos(M_PI / 25 * float(i));
                    linel.y = r * sin(M_PI / 25 * float(i));
                    liner.x = r * cos(M_PI / 25 * float(i + 1));
                    liner.y = r * sin(M_PI / 25 * float(i + 1));
                    lines.push_back(linel);
                    lines.push_back(liner);
                    this_marker.points = lines;
                    this_marker.colors = convex_color;
                    this_marker.id = id++;
                    this_marker.lifetime = ros::Duration();
                    vis_arr.markers.push_back(this_marker);
                }
                circle = true;
            }

            auto getline = [] (int idx, float dist,
                                Eigen::Vector2f& qB,
                                std::vector<geometry_msgs::Point>& lines,
                                geometry_msgs::Point& linel,
                                geometry_msgs::Point& liner,
                                visualization_msgs::Marker& this_marker,
                                std::vector<std_msgs::ColorRGBA> & convex_color,
                                visualization_msgs::MarkerArray& vis_arr,
                                int half_num_scan,
                                int id
                                ) -> void {
                                    lines.clear();
                                    linel.x = qB(0);
                                    linel.y = qB(1);
                                    linel.z = 0.1;
                                    liner.x = dist * cos(M_PI / half_num_scan * (idx - half_num_scan));
                                    liner.y = dist * sin(M_PI / half_num_scan * (idx - half_num_scan));
                                    liner.z = 0.1;
                                    lines.push_back(linel);
                                    lines.push_back(liner);
                                    this_marker.points = lines;
                                    this_marker.colors = convex_color;
                                    this_marker.id = id;
                                    this_marker.lifetime = ros::Duration(0.2);
                                    vis_arr.markers.push_back(this_marker);
                                };

            {
                this_marker.ns = "extent_line";
                getline(g.convex.convex_lidx, g.convex.convex_ldist, g.qB, 
                    lines, linel, liner, this_marker, convex_color, vis_arr, g.half_scan, id++);
                getline(g.convex.convex_ridx, g.convex.convex_rdist, g.qB, 
                    lines, linel, liner, this_marker, convex_color, vis_arr, g.half_scan, id++);
                Eigen::Vector2f origin(0, 0);


                this_marker.ns = "orig_line";
                getline(g._left_idx, g._ldist, origin, 
                    lines, linel, liner, this_marker, colormap["fin_agc"], vis_arr, g.half_scan, id++);
                getline(g._right_idx, g._rdist, origin, 
                    lines, linel, liner, this_marker, colormap["fin_agc"], vis_arr, g.half_scan, id++);
            }
        }
        */

    }

    void GapVisualizer::drawManipGaps(std::vector<dynamic_gap::Gap> vec) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;

        bool circle = false;
        for (auto & gap : vec) {
            drawManipGap(vis_arr, gap, circle);
        }
        gapside_publisher.publish(vis_arr);
    }

    TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        goal_selector_traj_vis = nh.advertise<geometry_msgs::PoseArray>("goal_select_traj", 1000);
        trajectory_score = nh.advertise<visualization_msgs::MarkerArray>("traj_score", 1000);
        all_traj_viz = nh.advertise<visualization_msgs::MarkerArray>("all_traj_vis", 1000);
    }

    void TrajectoryVisualizer::globalPlanRbtFrame(const std::vector<geometry_msgs::PoseStamped> & plan) {
        if (!cfg_->gap_viz.debug_viz) return;
        if (plan.size() < 1) {
            ROS_WARN_STREAM("Goal Selector Returned Trajectory Size " << plan.size() << " < 1");
        }

        geometry_msgs::PoseArray vis_arr;
        vis_arr.header = plan.at(0).header;
        for (auto & pose : plan) {
            vis_arr.poses.push_back(pose.pose);
        }
        goal_selector_traj_vis.publish(vis_arr);
    }

    void TrajectoryVisualizer::trajScore(geometry_msgs::PoseArray p_arr, std::vector<double> p_score) {
        if (!cfg_->gap_viz.debug_viz) return;

        ROS_FATAL_STREAM_COND(!p_score.size() == p_arr.poses.size(), "trajScore size mismatch, p_arr: "
            << p_arr.poses.size() << ", p_score: " << p_score.size());

        visualization_msgs::MarkerArray score_arr;
        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = p_arr.header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "trajScore";
        lg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;

        lg_marker.color.a = 1;
        lg_marker.color.r = 1;
        lg_marker.color.g = 1;
        lg_marker.color.b = 1;

        for (int i = 0; i < p_score.size(); i++) {
            lg_marker.id = int (score_arr.markers.size());
            lg_marker.pose.position.x = p_arr.poses.at(i).position.x;
            lg_marker.pose.position.y = p_arr.poses.at(i).position.y;
            lg_marker.pose.position.z = 0.5;
            lg_marker.text = std::to_string(p_score.at(i));
            score_arr.markers.push_back(lg_marker);
        }

        trajectory_score.publish(score_arr);
    }

    void TrajectoryVisualizer::pubAllScore(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<double>> cost) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray score_arr;
        visualization_msgs::Marker lg_marker;
        if (prr.size() == 0)
        {
            ROS_WARN_STREAM("traj count length 0");
            return;
        }

        // The above ensures this is safe
        lg_marker.header.frame_id = prr.at(0).header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "trajScore";
        lg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.05;

        lg_marker.color.a = 1;
        lg_marker.color.r = 1;
        lg_marker.color.g = 1;
        lg_marker.color.b = 1;
        lg_marker.lifetime = ros::Duration(0.25);


        ROS_FATAL_STREAM_COND(!prr.size() == cost.size(), "pubAllScore size mismatch, prr: "
            << prr.size() << ", cost: " << cost.size());

        for (int i = 0; i < prr.size(); i++) {

            ROS_FATAL_STREAM_COND(!prr.at(i).poses.size() == cost.at(i).size(), "pubAllScore size mismatch," << i << "th "
                << prr.at(i).poses.size() << ", cost: " << cost.at(i).size());
            
            for (int j = 0; j < prr.at(i).poses.size(); j++) {
                lg_marker.id = int (score_arr.markers.size());
                lg_marker.pose = prr.at(i).poses.at(j);

                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << cost.at(i).at(j);
                lg_marker.text = stream.str();

                score_arr.markers.push_back(lg_marker);
            }
        }
        trajectory_score.publish(score_arr);
    }

    void TrajectoryVisualizer::pubAllTraj(std::vector<geometry_msgs::PoseArray> prr) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_traj_arr;
        visualization_msgs::Marker lg_marker;
        if (prr.size() == 0)
        {
            ROS_WARN_STREAM("traj count length 0");
            return;
        }

        // The above makes this safe
        lg_marker.header.frame_id = prr.at(0).header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "allTraj";
        lg_marker.type = visualization_msgs::Marker::ARROW;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = cfg_->gap_viz.fig_gen ? 0.02 : 0.01;// 0.01;
        lg_marker.scale.z = 0.1;
        lg_marker.color.a = 1;
        lg_marker.color.r = 0.5;
        lg_marker.color.g = 0.5;
        lg_marker.lifetime = ros::Duration(0.25);

        for (auto & arr : prr) {
            for (auto pose : arr.poses) {
                lg_marker.id = int (vis_traj_arr.markers.size());
                lg_marker.pose = pose;
                vis_traj_arr.markers.push_back(lg_marker);
            }
        }
        all_traj_viz.publish(vis_traj_arr);
    }

    GoalVisualizer::GoalVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        goal_pub = nh.advertise<visualization_msgs::Marker>("goals", 1000);
        gapwp_pub = nh.advertise<visualization_msgs::MarkerArray>("gap_goals", 1000);
        gapwp_color.r = 0.5;
        gapwp_color.g = 1;
        gapwp_color.b = 0.5;
        gapwp_color.a = 1;
        localGoal_color.r = 0;
        localGoal_color.g = 1;
        localGoal_color.b = 0;
        localGoal_color.a = 1;
    }

    void GoalVisualizer::localGoal(geometry_msgs::PoseStamped localGoal)
    {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = localGoal.header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "local_goal";
        lg_marker.id = 0;
        lg_marker.type = visualization_msgs::Marker::SPHERE;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.position.x = localGoal.pose.position.x;
        lg_marker.pose.position.y = localGoal.pose.position.y;
        lg_marker.pose.position.z = 2;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;
        lg_marker.color = localGoal_color;
        goal_pub.publish(lg_marker);
    }

    void GoalVisualizer::drawGapGoal(visualization_msgs::MarkerArray& vis_arr, dynamic_gap::Gap g) {
        if (!cfg_->gap_viz.debug_viz) return;
        if (!g.goal.set) {
            return;
        }

        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = g._frame;
        // std::cout << "g frame in draw gap goal: " << g._frame << std::endl;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "gap_goal";
        lg_marker.id = int (vis_arr.markers.size());
        lg_marker.type = visualization_msgs::Marker::SPHERE;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.position.x = g.goal.x;
        lg_marker.pose.position.y = g.goal.y;
        lg_marker.pose.position.z = 0.5;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;
        lg_marker.color = gapwp_color;
        lg_marker.lifetime = ros::Duration(0.5);
        vis_arr.markers.push_back(lg_marker);

    }

    void GoalVisualizer::drawGapGoals(std::vector<dynamic_gap::Gap> gs) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;
        for (auto & gap : gs) {
            drawGapGoal(vis_arr, gap);
        }
        gapwp_pub.publish(vis_arr);
        return;
    }

}