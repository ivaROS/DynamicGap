#include <dynamic_gap/visualization/GapVisualizer.h>

namespace dynamic_gap
{
    GapVisualizer::GapVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg) 
    {
        initialize(nh, cfg);
    }

    void GapVisualizer::initialize(ros::NodeHandle& nh, const DynamicGapConfig& cfg) 
    {
        cfg_ = &cfg;
        rawGapsPublisher = nh.advertise<visualization_msgs::MarkerArray>("raw_gaps", 10);
        simpGapsPublisher = nh.advertise<visualization_msgs::MarkerArray>("simp_gaps", 10);
        manipGapsPublisher = nh.advertise<visualization_msgs::MarkerArray>("manip_gaps", 10);
        reachableGapsPublisher = nh.advertise<visualization_msgs::MarkerArray>("reachable_gaps", 10);
        ahpfCentersPublisher = nh.advertise<visualization_msgs::MarkerArray>("reachable_gap_centers", 10);
        // reachable_gap_no_RGE_publisher = nh.advertise<visualization_msgs::MarkerArray>("reachable_gap_no_RGE", 10);
        gapSplinesPublisher = nh.advertise<visualization_msgs::MarkerArray>("gap_splines", 10);

        gapModelPosPublisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_pos", 10);
        gapModelVelPublisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_vel", 10);
        // gapmodel_vel_error_publisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_vel_error", 10);
        // gapmodel_pos_GT_publisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_pos_GT", 10);
        // gapmodel_vel_GT_publisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_vel_GT", 10);


        // std_msgs::ColorRGBA std_color;
        // std_msgs::ColorRGBA raw;
        std_msgs::ColorRGBA rawInitial, rawTerminal, simpInitial, simpTerminal,
                            manipInitial, manipTerminal, gap_model, reachableGapCenters, gapSplines;

        // std_msgs::ColorRGBA simp_expanding;
        // std_msgs::ColorRGBA simp_closing;
        // std_msgs::ColorRGBA simp_translating;
        // std_msgs::ColorRGBA manip_expanding;
        // std_msgs::ColorRGBA manip_closing;
        // std_msgs::ColorRGBA manip_translating;
        // std_msgs::ColorRGBA ;



        // std_msgs::ColorRGBA reachableGapCenters;
        // std_msgs::ColorRGBA gap_splines;

        // prev_num_gaps = 0;
        // prev_num_reachable_gaps = 0;
        // prev_num_manip_gaps = 0;
        // prev_num_models = 0;

        // Raw Therefore Alpha halved
        // RAW: RED
        // RAW RADIAL
        /*
        std_color.a = 0.5;
        std_color.r = 1.0; //std_color.r = 0.7;
        std_color.g = 0.0; //std_color.g = 0.1;
        std_color.b = 0.0; //std_color.b = 0.5;
        raw_radial.push_back(std_color);
        raw_radial.push_back(std_color);
        */
        // RAW
        // std_color.a = 1.0;
        // std_color.r = 0.0; //std_color.r = 0.6;
        // std_color.g = 0.0; //std_color.g = 0.2;
        // std_color.b = 0.0; //std_color.b = 0.1;
        // raw.push_back(std_color);
        // raw.push_back(std_color);

        rawInitial.a = 1.0;
        rawInitial.r = 0.3; //std_color.r = 0.6;
        rawInitial.g = 0.3; //std_color.g = 0.2;
        rawInitial.b = 0.3; //std_color.b = 0.1;
        // rawInitial.push_back(std_color);
        // rawInitial.push_back(std_color);
        
        rawTerminal.a = 0.5;
        rawTerminal.r = 0.3; //std_color.r = 0.6;
        rawTerminal.g = 0.3; //std_color.g = 0.2;
        rawTerminal.b = 0.3; //std_color.b = 0.1;
        // rawTerminal.push_back(std_color);
        // rawTerminal.push_back(std_color);

        simpInitial.a = 1.0;
        simpInitial.r = 0.0;
        simpInitial.g = 0.0;
        simpInitial.b = 0.0; 
        // simpInitial.push_back(std_color);
        // simpInitial.push_back(std_color);

        simpTerminal.a = 0.50;
        simpTerminal.r = 0.0;
        simpTerminal.g = 0.0; 
        simpTerminal.b = 0.0; 
        // simpTerminal.push_back(std_color);
        // simpTerminal.push_back(std_color);

        manipInitial.a = 1.0;
        manipInitial.r = 0.0; 
        manipInitial.g = 1.0;
        manipInitial.b = 0.0;
        // manipInitial.push_back(std_color);
        // manipInitial.push_back(std_color);

        manipTerminal.a = 0.5;
        manipTerminal.r = 0.0; 
        manipTerminal.g = 1.0; 
        manipTerminal.b = 0.0; 
        // manipTerminal.push_back(std_color);
        // manipTerminal.push_back(std_color);

        // MODELS: PURPLE
        gap_model.a = 1.0;
        gap_model.r = 1;
        gap_model.g = 0;
        gap_model.b = 1;
        // gap_model.push_back(std_color);
        // gap_model.push_back(std_color);

        reachableGapCenters.a = 1.0;
        reachableGapCenters.r = 1;
        reachableGapCenters.g = 0.65;
        reachableGapCenters.b = 0;
        // reachableGapCenters.push_back(std_color);
        // reachableGapCenters.push_back(std_color);

        gapSplines.a = 1.0;
        gapSplines.r = 0.8;
        gapSplines.g = 1;
        gapSplines.b = 0;
        // gapSplines.push_back(std_color);
        // gapSplines.push_back(std_color);        

        // colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw", raw));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_initial", rawInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_terminal", rawTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_initial", simpInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_terminal", simpTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("manip_initial", manipInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("manip_terminal", manipTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("reachable_gap_centers", reachableGapCenters));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("gap_splines", gapSplines));
    }

    void GapVisualizer::drawGaps(const std::vector<dynamic_gap::Gap> & gaps, std::string ns) 
    {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = ns;
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);

        visualization_msgs::MarkerArray markerArray;
        for (const dynamic_gap::Gap & gap : gaps) 
        {
            drawGap(markerArray, gap, ns, true);
        }
        
        if (ns == "raw") 
        {
            rawGapsPublisher.publish(clearMarkerArray);
            rawGapsPublisher.publish(markerArray);
        } else if (ns == "simp") 
        {
            simpGapsPublisher.publish(clearMarkerArray);
            simpGapsPublisher.publish(markerArray);
        }

    }

    void GapVisualizer::drawGap(visualization_msgs::MarkerArray & markerArray, const dynamic_gap::Gap & gap, 
                                std::string ns, bool initial) 
    {
        // ROS_INFO_STREAM(g._left_idx << ", " << g._leftDist << ", " << g._right_idx << ", " << g._rdist << ", " << g.frame_);
        if (!cfg_->gap_viz.debug_viz) return;
        //ROS_INFO_STREAM("in draw gap");
        int viz_offset = 0;
        float viz_jitter = cfg_->gap_viz.viz_jitter;
        
        if (viz_jitter > 0 && gap.isRadial(initial))
            viz_offset = gap.isRightType(initial) ? -2 : 2;

        int leftIdx = initial ? gap.LIdx() : gap.termLIdx(); // initial ? gap.RIdx() : gap.termRIdx(); //
        int rightIdx = initial ? gap.RIdx() : gap.termRIdx(); // initial ? gap.LIdx() : gap.termLIdx(); //
        float leftDist = initial ? gap.LDist() : gap.termLDist(); // initial ? gap.RDist() : gap.termRDist();
        float rightDist = initial ? gap.RDist() : gap.termRDist(); // initial ? gap.LDist() : gap.termLDist();

        //ROS_INFO_STREAM("leftIdx: " << leftIdx << ", ldist: " << ldist << ", rightIdx: " << rightIdx << ", rightDist: " << rightDist);
        int gapIdxSpan = (leftIdx - rightIdx);
        if (gapIdxSpan < 0)
            gapIdxSpan += cfg_->scan.full_scan; // 2*gap.half_scan; // taking off int casting here

        visualization_msgs::Marker marker;
        marker.header.frame_id = gap.frame_;
        marker.header.stamp = ros::Time();
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // std::cout << "ns coming in: " << ns << std::endl;
        std::string local_ns = ns;        
        if (initial)
            local_ns.append("_initial");
        else
            local_ns.append("_terminal");

        // std::cout << "gap category: " << g.getCategory() << std::endl;
        //ROS_INFO_STREAM("ultimate local ns: " << local_ns);
        auto colorIter = colorMap.find(local_ns);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        // marker.colors.push_back(colorIter->second);
        // marker.colors.push_back(colorIter->second);
        marker.colors.insert(marker.colors.begin(), 2, colorIter->second); // add it twice

        float thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        marker.scale.x = thickness;
        marker.scale.y = 0.0001;
        marker.scale.z = 0.0001;

        float pointZ = (ns == "raw") ? 0.0001 : 0.001;
        geometry_msgs::Point linel, liner;
        linel.z = pointZ;
        liner.z = pointZ;
        std::vector<geometry_msgs::Point> lines;

        int id = (int) markerArray.markers.size();

        int num_segments = gapIdxSpan / cfg_->gap_viz.min_resoln + 1;
        float distIncrement = (leftDist - rightDist) / num_segments;
        int midGapRightIdx = rightIdx + viz_offset;
        float midGapRightDist = rightDist;

        float midGapRightTheta;
        for (int i = 0; i < num_segments; i++)
        {
            lines.clear();
            midGapRightTheta = idx2theta(midGapRightIdx);
            liner.x = (midGapRightDist + viz_jitter) * cos(midGapRightTheta);
            liner.y = (midGapRightDist + viz_jitter) * sin(midGapRightTheta);
            lines.push_back(liner);
            
            midGapRightIdx = (midGapRightIdx + cfg_->gap_viz.min_resoln) % cfg_->scan.full_scan; // int(2*gap.half_scan);
            midGapRightDist += distIncrement;
            midGapRightTheta = idx2theta(midGapRightIdx);
            linel.x = (midGapRightDist + viz_jitter) * cos(midGapRightTheta);
            linel.y = (midGapRightDist + viz_jitter) * sin(midGapRightTheta);
            lines.push_back(linel);

            marker.points = lines;
            marker.id = id++;
            marker.lifetime = ros::Duration(0);

            markerArray.markers.push_back(marker);
        }

        // close the last
        // marker.scale.x = 10*thickness;
        // lines.clear();
        // midGapRightTheta = idx2theta(midGapRightIdx);
        // liner.x = (midGapRightDist + viz_jitter) * cos(midGapRightTheta);
        // liner.y = (midGapRightDist + viz_jitter) * sin(midGapRightTheta);
        // lines.push_back(liner);

        // float leftTheta = idx2theta(leftIdx);
        // linel.x = (leftDist + viz_jitter) * cos(leftTheta);
        // linel.y = (leftDist + viz_jitter) * sin(leftTheta);
        // lines.push_back(linel);

        marker.points = lines;
        marker.id = id++;
        marker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(marker);
    }
    
    void GapVisualizer::drawGapSpline(visualization_msgs::MarkerArray & markerArray, const dynamic_gap::Gap & gap) 
    {
        // discretize gap lifespan

        // plug in x/y coefficients to get list of points

        visualization_msgs::Marker marker;
        marker.header.frame_id = gap.frame_;
        marker.header.stamp = ros::Time();
        marker.ns = "gap_splines";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        auto colorIter = colorMap.find("gap_splines");
        if (colorIter == colorMap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }        

        // marker.colors = colorIter->second;
        // marker.colors.push_back(colorIter->second);
        // marker.colors.push_back(colorIter->second);
        marker.colors.insert(marker.colors.begin(), 2, colorIter->second);

        float thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        marker.scale.x = thickness;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        geometry_msgs::Point line;
        line.z = 0.0005;

        std::vector<geometry_msgs::Point> lines;
        int id = (int) markerArray.markers.size();
        marker.lifetime = ros::Duration(0);

        float num_spline_pts = float(cfg_->traj.num_curve_points + cfg_->traj.num_extended_gap_origin_points);
        float t, x, y;
        for (int i = 0.0; i < (num_spline_pts - 1); i++) {            
            lines.clear();  

            t = (i / num_spline_pts) * gap.gapLifespan_;
            x = gap.splineXCoefs_[3] * pow(t, 3) + 
                       gap.splineXCoefs_[2] * pow(t, 2) + 
                       gap.splineXCoefs_[1] * t +
                       gap.splineXCoefs_[0];
            y = gap.splineYCoefs_[3] * pow(t, 3) + 
                       gap.splineYCoefs_[2] * pow(t, 2) + 
                       gap.splineYCoefs_[1] * t +
                       gap.splineYCoefs_[0];
            
            line.x = x;
            line.y = y;
            lines.push_back(line);

            t = ((i + 1) / num_spline_pts) * gap.gapLifespan_;
            x = gap.splineXCoefs_[3] * pow(t, 3) + 
                       gap.splineXCoefs_[2] * pow(t, 2) + 
                       gap.splineXCoefs_[1] * t +
                       gap.splineXCoefs_[0];
            y = gap.splineYCoefs_[3] * pow(t, 3) + 
                       gap.splineYCoefs_[2] * pow(t, 2) + 
                       gap.splineYCoefs_[1] * t +
                       gap.splineYCoefs_[0];            
            line.x = x;
            line.y = y;
            lines.push_back(line);

            marker.points = lines;
            marker.id = id++;
            markerArray.markers.push_back(marker);
        }
    }


    void GapVisualizer::drawGapSplines(const std::vector<dynamic_gap::Gap> & gaps) 
    {
        if (!cfg_->gap_viz.debug_viz) return;
        
        // First, clearing topic.
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = "clear";
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);
        gapSplinesPublisher.publish(clearMarkerArray);
        
        visualization_msgs::MarkerArray markerArray;
        for (const dynamic_gap::Gap & gap : gaps) 
        {
            if (gap.crossed_ || gap.closed_) 
            {
                drawGapSpline(markerArray, gap);
            }
        }
        gapSplinesPublisher.publish(markerArray);
    }

    void GapVisualizer::drawReachableGapNoRGE(visualization_msgs::MarkerArray & markerArray, const dynamic_gap::Gap & gap) 
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = gap.frame_;
        marker.header.stamp = ros::Time();
        marker.ns = "reachable_gap_centers";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        auto colorIter = colorMap.find("reachable_gap_centers");
        if (colorIter == colorMap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }        

        // marker.colors = colorIter->second;
        // marker.colors.push_back(colorIter->second);
        // marker.colors.push_back(colorIter->second);
        marker.colors.insert(marker.colors.begin(), 2, colorIter->second);

        float thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        marker.scale.x = thickness;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // marker.scale.y = 0.05;
        // marker.scale.z = 0.1;

        float s;
        geometry_msgs::Point linel, liner;
        linel.z = 0.0005;
        liner.z = 0.0005;

        std::vector<geometry_msgs::Point> lines;
        int id = (int) markerArray.markers.size();
        marker.lifetime = ros::Duration(0);

        lines.clear();

        int num_curve_points = cfg_->traj.num_curve_points;
        float pos_val0, pos_val1, pos_val2;
        Eigen::Vector2d curr_left_pt, curr_right_pt;
        for (float i = 0; i < (num_curve_points - 1); i++) {
            lines.clear();  
            s = (i) / (num_curve_points - 1);

            // ROS_INFO_STREAM("s: " << s << ", s_left: " << s_left << ", s_right: " << s_right);
            pos_val0 = (1 - s) * (1 - s);
            pos_val1 = 2*(1 - s)*s;
            pos_val2 = s*s;

            curr_left_pt = pos_val1*gap.leftPt0_ + pos_val2*gap.leftPt1_;

            s = ((i+1)) / (num_curve_points - 1);
            
            linel.x = curr_left_pt[0];
            linel.y = curr_left_pt[1];
            lines.push_back(linel);

            pos_val0 = (1 - s) * (1 - s);
            pos_val1 = 2*(1 - s)*s;
            pos_val2 = s*s;

            curr_left_pt = pos_val1*gap.leftPt0_ + pos_val2*gap.leftPt1_;

            linel.x = curr_left_pt[0];
            linel.y = curr_left_pt[1];
            lines.push_back(linel);

            marker.points = lines;
            marker.id = id++;
            markerArray.markers.push_back(marker);
        }

        for (float i = 0; i < (num_curve_points - 1); i++) 
        {
            lines.clear();  
            s = (i) / (num_curve_points - 1);

            // ROS_INFO_STREAM("s: " << s << ", s_left: " << s_left << ", s_right: " << s_right);
            pos_val0 = (1 - s) * (1 - s);
            pos_val1 = 2*(1 - s)*s;
            pos_val2 = s*s;

            curr_right_pt = pos_val1*gap.rightPt0_ + pos_val2*gap.rightPt1_;

            s = ((i+1)) / (num_curve_points - 1);
            
            liner.x = curr_right_pt[0];
            liner.y = curr_right_pt[1];
            lines.push_back(liner);

            pos_val0 = (1 - s) * (1 - s);
            pos_val1 = 2*(1 - s)*s;
            pos_val2 = s*s;

            curr_right_pt = pos_val1*gap.rightPt0_ + pos_val2*gap.rightPt1_;

            liner.x = curr_right_pt[0];
            liner.y = curr_right_pt[1];
            lines.push_back(liner);

            marker.points = lines;
            marker.id = id++;
            markerArray.markers.push_back(marker);
        }
    }

    void GapVisualizer::drawReachableGap(visualization_msgs::MarkerArray & markerArray, const dynamic_gap::Gap & gap) 
    {
        // ROS_INFO_STREAM("in drawReachableGap");

        float num_curve_points = cfg_->traj.num_curve_points;
        // float num_extended_gap_origin_points = (cfg_->gap_manip.radial_extend) ? cfg_->traj.num_extended_gap_origin_points : 0.0;
        // float half_num_scan = gap.half_scan;

        Eigen::Vector2f left_gap_origin(gap.leftBezierOrigin_[0],       
                                        gap.leftBezierOrigin_[1]);
        Eigen::Vector2f right_gap_origin(gap.rightBezierOrigin_[0],
                                        gap.rightBezierOrigin_[1]);

        // ROS_INFO_STREAM("left_gap_origin: (" << left_gap_origin[0] << ", " << left_gap_origin[1] << "), left_pt_0: (" << left_pt_0[0] << ", " << left_pt_0[1] << "), weighted_left_pt_0: (" << weighted_left_pt_0[0] << ", " << weighted_left_pt_0[1] << "), left_pt_1: (" << left_pt_1[0] << ", " << left_pt_1[1] << ")");
        // ROS_INFO_STREAM("right_gap_origin: (" << right_gap_origin[0] << ", " << right_gap_origin[1] << "), right_pt_0: (" << right_pt_0[0] << ", " << right_pt_0[1] << "), weighted_right_pt_0: (" << weighted_right_pt_0[0] << ", " << weighted_right_pt_0[1] << "), right_pt_1: (" << right_pt_1[0] << ", " << right_pt_1[1] << ")");

        visualization_msgs::Marker marker;
        marker.header.frame_id = gap.frame_;
        marker.header.stamp = ros::Time();
        marker.ns = "manip_initial";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0075;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        auto colorIter = colorMap.find("manip_initial");
        if (colorIter == colorMap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }        

        // marker.colors.push_back(colorIter->second);
        // marker.colors.push_back(colorIter->second);
        marker.colors.insert(marker.colors.begin(), 2, colorIter->second);

        // marker.colors = colorIter->second;
        float thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        marker.scale.x = thickness;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // marker.scale.y = 0.05;
        // marker.scale.z = 0.1;

        float s;
        geometry_msgs::Point linel, liner;
        linel.z = marker.pose.position.z;
        liner.z = marker.pose.position.z;

        std::vector<geometry_msgs::Point> lines;
        int id = (int) markerArray.markers.size();
        marker.lifetime = ros::Duration(0);

        // populate left curve

        lines.clear();  
        // ROS_INFO_STREAM("i: " << i << ", s: " << s);
        Eigen::Vector2f left_pt = gap.extendedGapOrigin_;
        
        linel.x = left_pt[0];
        linel.y = left_pt[1];
        lines.push_back(linel);

        left_pt = left_gap_origin;
        
        linel.x = left_pt[0];
        linel.y = left_pt[1];
        lines.push_back(linel);

        marker.points = lines;
        marker.id = id++;
        markerArray.markers.push_back(marker);

        // populate right curve
        lines.clear();  
        // ROS_INFO_STREAM("i: " << i << ", s: " << s);
        Eigen::Vector2f right_pt = gap.extendedGapOrigin_;
        
        liner.x = right_pt[0];
        liner.y = right_pt[1];
        lines.push_back(liner);

        right_pt = right_gap_origin;
        
        liner.x = right_pt[0];
        liner.y = right_pt[1];
        lines.push_back(liner);

        marker.points = lines;
        marker.id = id++;
        markerArray.markers.push_back(marker);
        
        // int num_pts_per_side = g.allCurvePts_.rows() / 2;
        // ROS_INFO_STREAM("number of all_curve_pts: " << g.allCurvePts_.rows());
        // ROS_INFO_STREAM("drawReachableGap from " << g.numLeftRGEPoints_ << " to " << g.numLeftRGEPoints_ + num_curve_points - 1);
        for (int i = gap.numLeftRGEPoints_; i < (gap.numLeftRGEPoints_ + num_curve_points - 1); i++) {            
            lines.clear();  

            Eigen::Vector2d left_pt = gap.allCurvePts_.row(i);
            
            linel.x = left_pt[0];
            linel.y = left_pt[1];
            lines.push_back(linel);

            left_pt = gap.allCurvePts_.row(i + 1);
        
            // ROS_INFO_STREAM("connecting " << i << " to " << (i + 1));

            linel.x = left_pt[0];
            linel.y = left_pt[1];
            lines.push_back(linel);

            marker.points = lines;
            marker.id = id++;
            markerArray.markers.push_back(marker);
        }

        //populate right curve
        // ROS_INFO_STREAM("drawReachableGap from " << g.numLeftRGEPoints_ + num_curve_points << " to " << g.allCurvePts_.rows() - 1);
        for (int i = (gap.numLeftRGEPoints_ + num_curve_points); i < (gap.allCurvePts_.rows() - 1); i++) 
        {
            lines.clear();  

            Eigen::Vector2d right_pt = gap.allCurvePts_.row(i);
            
            liner.x = right_pt[0];
            liner.y = right_pt[1];
            lines.push_back(liner);

            right_pt = gap.allCurvePts_.row(i + 1);
            
            liner.x = right_pt[0];
            liner.y = right_pt[1];
            lines.push_back(liner);

            marker.points = lines;
            marker.id = id++;
            markerArray.markers.push_back(marker);  
        }
    }

    void GapVisualizer::drawReachableGaps(const std::vector<dynamic_gap::Gap> & gaps) 
    {
        if (!cfg_->gap_viz.debug_viz) return;
        
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = "clear";
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);
        reachableGapsPublisher.publish(clearMarkerArray);
        // reachable_gap_no_RGE_publisher.publish(clearMarkerArray);

        // First, clearing topic.
        
        visualization_msgs::MarkerArray markerArray, markerArray1;
        // int counter = 0;
        for (const dynamic_gap::Gap & gap : gaps) 
        {
            // if (counter == 1) {
            drawReachableGap(markerArray, gap);
                // drawReachableGapNoRGE(markerArray1, gap);
            // }
            // counter++;
        }
        reachableGapsPublisher.publish(markerArray);
        // reachable_gap_no_RGE_publisher.publish(markerArray1);
    }

    void GapVisualizer::drawReachableGapCenters(visualization_msgs::MarkerArray & markerArray, const dynamic_gap::Gap & gap)
    {
        // ROS_INFO_STREAM("in drawReachableGapCenters");
        // ROS_INFO_STREAM("left right centers number of rows: " << g.leftRightCenters_.rows());

        float num_curve_points = cfg_->traj.num_curve_points;
        float num_extended_gap_origin_points = cfg_->traj.num_extended_gap_origin_points;
        visualization_msgs::Marker marker;
        marker.header.frame_id = gap.frame_;
        marker.header.stamp = ros::Time();
        marker.ns = "reachable_gap_centers";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        auto colorIter = colorMap.find("reachable_gap_centers");
        if (colorIter == colorMap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }
        // marker.colors = colorIter->second;
        // marker.colors.push_back(colorIter->second);
        // marker.colors.push_back(colorIter->second);
        marker.colors.insert(marker.colors.begin(), 2, colorIter->second);


        float thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        marker.scale.x = thickness;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // marker.scale.y = 0.05;
        // marker.scale.z = 0.1;

        float s;
        geometry_msgs::Point linel, liner;
        linel.z = 0.0005;
        liner.z = 0.0005;

        std::vector<geometry_msgs::Point> lines;

        // populate left curve
        int id = (int) markerArray.markers.size();
        marker.lifetime = ros::Duration(0);

        int num_pts_per_side = gap.leftRightCenters_.rows() / 2;
        for (int i = 0; i < (num_pts_per_side - 1); i++) {
            lines.clear();  

            Eigen::Vector2d left_pt = gap.leftRightCenters_.row(i);
            // ROS_INFO_STREAM("i: " << i << ", left_pt: " << left_pt[0] << ", " << left_pt[1]);
            linel.x = left_pt[0];
            linel.y = left_pt[1];
            lines.push_back(linel);

            left_pt = gap.leftRightCenters_.row(i + 1);
            
            linel.x = left_pt[0];
            linel.y = left_pt[1];
            lines.push_back(linel);

            marker.points = lines;
            marker.id = id++;
            markerArray.markers.push_back(marker);
        }

        for (int i = num_pts_per_side; i < (2*num_pts_per_side - 1); i++) {
            lines.clear();  

            Eigen::Vector2d right_pt = gap.leftRightCenters_.row(i);
            // ROS_INFO_STREAM("i: " << i << ", right_pt: " << right_pt[0] << ", " << right_pt[1]);

            liner.x = right_pt[0];
            liner.y = right_pt[1];
            lines.push_back(liner);

            right_pt = gap.leftRightCenters_.row(i + 1);
            
            liner.x = right_pt[0];
            liner.y = right_pt[1];
            lines.push_back(liner);

            marker.points = lines;
            marker.id = id++;
            markerArray.markers.push_back(marker);
        }

    }

    void GapVisualizer::drawReachableGapsCenters(const std::vector<dynamic_gap::Gap> & gaps) 
    {
        if (!cfg_->gap_viz.debug_viz) return;
        
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = "clear";
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);
        ahpfCentersPublisher.publish(clearMarkerArray);

        // First, clearing topic.
        
        visualization_msgs::MarkerArray markerArray;
        // int counter = 0;
        for (const dynamic_gap::Gap & gap : gaps) 
        {
            //if (counter == 0) {
            drawReachableGapCenters(markerArray, gap);
            // }
            // counter++;
        }
        ahpfCentersPublisher.publish(markerArray);
    }

    void GapVisualizer::drawGapsModels(const std::vector<dynamic_gap::Gap> & gaps) 
    {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = "gap_models";
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);
        gapModelPosPublisher.publish(clearMarkerArray);
        gapModelVelPublisher.publish(clearMarkerArray);
        // gapmodel_pos_GT_publisher.publish(clearMarkerArray);
        // gapmodel_vel_GT_publisher.publish(clearMarkerArray);
        // gapmodel_vel_error_publisher.publish(clearMarkerArray);

        visualization_msgs::MarkerArray gap_vel_arr, gap_vel_error_arr, gap_pos_GT_arr, gap_vel_GT_arr;
        for (const dynamic_gap::Gap & gap : gaps) 
        {
            drawGapModels(gap_vel_arr, gap_vel_error_arr, gap, "gap_models");
            // drawGapGroundTruthModels(gap_pos_GT_arr, gap_vel_GT_arr, gap, "gap_GT_models");
        }
        // gapModelPosPublisher.publish(gap_pos_arr);
        gapModelVelPublisher.publish(gap_vel_arr);
        // gapmodel_vel_error_publisher.publish(gap_vel_error_arr);
        // gapmodel_pos_GT_publisher.publish(gap_pos_GT_arr);
        // gapmodel_vel_GT_publisher.publish(gap_vel_GT_arr);
        // prev_num_models = 2* gaps.size();
    }

    void GapVisualizer::drawGapGroundTruthModels(visualization_msgs::MarkerArray & gap_vel_arr, 
                                                    const dynamic_gap::Gap & gap, std::string ns) 
    {
        int model_id = (int) gap_vel_arr.markers.size();

        visualization_msgs::Marker leftGapPtModel_vel_pt, rightGapPtModel_vel_pt;

        draw_model_pt_head(leftGapPtModel_vel_pt, gap, true, model_id, ns, true);
        gap_vel_arr.markers.push_back(leftGapPtModel_vel_pt);

        draw_model_pt_head(rightGapPtModel_vel_pt, gap, false, model_id, ns, true);
        gap_vel_arr.markers.push_back(rightGapPtModel_vel_pt);
    }

    void GapVisualizer::drawGapModels(visualization_msgs::MarkerArray & gap_vel_arr,   
                                        visualization_msgs::MarkerArray & gap_vel_error_arr, 
                                        const dynamic_gap::Gap & gap, std::string ns) 
    {
        int model_id = (int) gap_vel_arr.markers.size();

        visualization_msgs::Marker leftGapPtModel_vel_pt, rightGapPtModel_vel_pt;
        visualization_msgs::Marker leftGapPtModel_vel_error_pt, rightGapPtModel_vel_error_pt;

        draw_model_pt_head(leftGapPtModel_vel_pt, gap, true, model_id, ns, false);
        gap_vel_arr.markers.push_back(leftGapPtModel_vel_pt);
        // draw_model_vel_error(leftGapPtModel_vel_error_pt, leftGapPtModel_vel_pt, g, true, ns);
        // gap_vel_error_arr.markers.push_back(leftGapPtModel_vel_error_pt);

        draw_model_pt_head(rightGapPtModel_vel_pt, gap, false, model_id, ns, false);
        gap_vel_arr.markers.push_back(rightGapPtModel_vel_pt);
        // draw_model_vel_error(rightGapPtModel_vel_error_pt, rightGapPtModel_vel_pt, g, true, ns);
        // gap_vel_error_arr.markers.push_back(rightGapPtModel_vel_error_pt);
    }

    void GapVisualizer::draw_model_pt_head(visualization_msgs::Marker & model_vel_pt, 
                                           const dynamic_gap::Gap & gap, bool left, int & model_id, std::string ns,
                                           bool ground_truth) {
        model_vel_pt.header.frame_id = gap.frame_;
        model_vel_pt.header.stamp = ros::Time();
        model_vel_pt.ns = ns;
        model_vel_pt.id = model_id++;
        model_vel_pt.type = visualization_msgs::Marker::ARROW;
        model_vel_pt.action = visualization_msgs::Marker::ADD;
        
        Eigen::Vector4f leftGapPtModel_state = (ground_truth) ? gap.leftGapPtModel_->getTrueState() : gap.leftGapPtModel_->getState();
        Eigen::Vector4f rightGapPtModel_state = (ground_truth) ? gap.rightGapPtModel_->getTrueState() : gap.rightGapPtModel_->getState();
        
        geometry_msgs::Point vel_pt;
        model_vel_pt.pose.position.x = (left) ? leftGapPtModel_state[0] : rightGapPtModel_state[0];
        model_vel_pt.pose.position.y = (left) ? leftGapPtModel_state[1] : rightGapPtModel_state[1];
        model_vel_pt.pose.position.z = 0.01;
   
        Eigen::Vector2f vel;
        if (left) {
            vel << leftGapPtModel_state[2] + gap.leftGapPtModel_->getRobotVel().twist.linear.x,
                   leftGapPtModel_state[3] + gap.leftGapPtModel_->getRobotVel().twist.linear.y;
        } else {
            vel << rightGapPtModel_state[2] + gap.rightGapPtModel_->getRobotVel().twist.linear.x, 
                   rightGapPtModel_state[3] + gap.rightGapPtModel_->getRobotVel().twist.linear.y;
        }

        float model_vel_theta = std::atan2(vel[1], vel[0]);
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, model_vel_theta);

        model_vel_pt.pose.orientation.x = quat.getX();
        model_vel_pt.pose.orientation.y = quat.getY();
        model_vel_pt.pose.orientation.z = quat.getZ();
        model_vel_pt.pose.orientation.w = quat.getW();

        model_vel_pt.scale.x = vel.norm() + 0.000001;
        model_vel_pt.scale.y = 0.1;
        model_vel_pt.scale.z = 0.000001;

        model_vel_pt.color.a = 1.0;
        model_vel_pt.color.r = 1.0;
        model_vel_pt.color.b = 1.0;
        model_vel_pt.lifetime = ros::Duration(0);
    }

    void GapVisualizer::draw_model_vel_error(visualization_msgs::Marker & model_vel_error_pt, 
                                                visualization_msgs::Marker model_vel_pt, 
                                                const dynamic_gap::Gap & gap, bool left, std::string ns)
    {
        model_vel_error_pt.header = model_vel_pt.header;
        model_vel_error_pt.ns = ns;
        model_vel_error_pt.id = model_vel_pt.id;

        model_vel_error_pt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        model_vel_error_pt.action = visualization_msgs::Marker::ADD;

        model_vel_error_pt.pose = model_vel_pt.pose;

        model_vel_error_pt.scale.z = 0.3;
        model_vel_error_pt.color.a = 1.0; // Don't forget to set the alpha!
        model_vel_error_pt.color.r = 0.0;
        model_vel_error_pt.color.g = 0.0;
        model_vel_error_pt.color.b = 0.0;

        float vel_error;
        if (left)
        {
            vel_error = sqrt(pow(gap.leftGapPtModel_->getState()[2] - gap.leftGapPtModel_->getTrueState()[2], 2) + pow(gap.leftGapPtModel_->getState()[3] - gap.leftGapPtModel_->getTrueState()[3], 2));
            ROS_INFO_STREAM("model: (" << gap.leftGapPtModel_->getState()[2] << ", " << gap.leftGapPtModel_->getState()[3] << 
                            "), GT: (" << gap.leftGapPtModel_->getTrueState()[2] << ", " << gap.leftGapPtModel_->getTrueState()[3] << "), error: " << vel_error);
        } else 
        {
            vel_error = sqrt(pow(gap.rightGapPtModel_->getState()[2] - gap.rightGapPtModel_->getTrueState()[2], 2) + pow(gap.rightGapPtModel_->getState()[3] - gap.rightGapPtModel_->getTrueState()[3], 2));
            ROS_INFO_STREAM("model: (" << gap.rightGapPtModel_->getState()[2] << ", " << gap.rightGapPtModel_->getState()[3] << 
                            "), GT: (" << gap.rightGapPtModel_->getTrueState()[2] << ", " << gap.rightGapPtModel_->getTrueState()[3] << "), error: " << vel_error);        
        }
        model_vel_error_pt.text = "vel_error: " + std::to_string(vel_error);
    }

    void GapVisualizer::drawManipGap(visualization_msgs::MarkerArray & markerArray, const dynamic_gap::Gap & gap, 
                                        bool & circle, std::string ns, bool initial) 
    {
        // if AGC: Color is Red
        // if Convex: color is Brown, viz_jitter + 0.1
        // if RadialExtension: color is green, draw additional circle
        if (!cfg_->gap_viz.debug_viz) return;

        if ((initial && !gap.mode.reduced_ && !gap.mode.convex_ && !gap.mode.RGC_) ||
            (!initial && !gap.mode.termReduced_ && !gap.mode.termConvex_ && !gap.mode.termRGC_)) {
            return;
        }

        float viz_jitter = cfg_->gap_viz.viz_jitter;
        int viz_offset = 0;
        
        if (viz_jitter > 0 && gap.isRadial(initial))
        {
            viz_offset = gap.isRightType(initial) ? -2 : 2;
        }

        int leftIdx = initial ? gap.cvxRightIdx() : gap.cvxTermRightIdx();
        int rightIdx = initial ? gap.cvxLeftIdx() : gap.cvxTermLeftIdx();
        float leftDist = initial ? gap.cvxRightDist() : gap.cvxTermRightDist();
        float rightDist = initial ? gap.cvxLeftDist() : gap.cvxTermLeftDist();

        std::string local_ns = ns;

        if (initial) {
            local_ns.append("_initial");
        } else {
            local_ns.append("_terminal");
        }

        // num gaps really means segments within a gap
        int gapIdxSpan = (rightIdx - leftIdx);
        if (gapIdxSpan < 0)
            gapIdxSpan += cfg_->scan.full_scan; // int(2*gap.half_scan);

        int num_segments = gapIdxSpan / cfg_->gap_viz.min_resoln + 1;
        float distIncrement = (rightDist - leftDist) / num_segments;
        int sub_gap_leftIdx = leftIdx + viz_offset;
        float sub_gap_leftDist = leftDist;

        visualization_msgs::Marker marker;
        marker.header.frame_id = gap.frame_;
        marker.header.stamp = ros::Time();
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;


        // ROS_INFO_STREAM("drawManipGap local_ns: " << local_ns);
        auto colorIter = colorMap.find(local_ns);
        if (colorIter == colorMap.end()) {
            ROS_FATAL_STREAM("[drawManipGaps] Visualization Color not found, return without drawing");
            return;
        }

        // marker.colors = colorIter->second;
        // marker.colors.push_back(colorIter->second);
        // marker.colors.push_back(colorIter->second);
        marker.colors.insert(marker.colors.begin(), 2, colorIter->second);

        float thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        marker.scale.x = thickness;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        geometry_msgs::Point linel;
        geometry_msgs::Point liner;
        liner.z = 0.0005;
        linel.z = 0.0005;
        std::vector<geometry_msgs::Point> lines;

        int id = (int) markerArray.markers.size();
        // ROS_INFO_STREAM("ID: "<< id);

        marker.lifetime = ros::Duration(0);

        float sub_gap_ltheta;
        for (int i = 0; i < num_segments - 1; i++)
        {
            lines.clear();
            sub_gap_ltheta = idx2theta(sub_gap_leftIdx);
            linel.x = (sub_gap_leftDist + viz_jitter) * cos(sub_gap_ltheta);
            linel.y = (sub_gap_leftDist + viz_jitter) * sin(sub_gap_ltheta);
            sub_gap_leftIdx = (sub_gap_leftIdx + cfg_->gap_viz.min_resoln) % cfg_->scan.full_scan; // int(gap.half_scan * 2);
            sub_gap_leftDist += distIncrement;
            
            sub_gap_ltheta = idx2theta(sub_gap_leftIdx);
            liner.x = (sub_gap_leftDist + viz_jitter) * cos(sub_gap_ltheta);
            liner.y = (sub_gap_leftDist + viz_jitter) * sin(sub_gap_ltheta);
            lines.push_back(linel);
            lines.push_back(liner);

            marker.points = lines;
            marker.id = id++;
            markerArray.markers.push_back(marker);
        }

        // close the last
        lines.clear();
        sub_gap_ltheta = idx2theta(sub_gap_leftIdx);
        linel.x = (sub_gap_leftDist + viz_jitter) * cos(sub_gap_ltheta);
        linel.y = (sub_gap_leftDist + viz_jitter) * sin(sub_gap_ltheta);
        float rtheta = idx2theta(rightIdx);
        liner.x = (rightDist + viz_jitter) * cos(rtheta);
        liner.y = (rightDist + viz_jitter) * sin(rtheta);
        lines.push_back(linel);
        lines.push_back(liner);
        marker.scale.x = thickness;
        marker.points = lines;
        marker.id = id++;
        marker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(marker);
    }

    void GapVisualizer::drawManipGaps(const std::vector<dynamic_gap::Gap> & gaps, std::string ns) 
    {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = ns;
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);
        manipGapsPublisher.publish(clearMarkerArray);

        visualization_msgs::MarkerArray markerArray;

        bool circle = false;
        for (const dynamic_gap::Gap & gap : gaps) 
        {
            drawManipGap(markerArray, gap, circle, ns, true); // initial
            drawManipGap(markerArray, gap, circle, ns, false); // false
        }
        manipGapsPublisher.publish(markerArray);
    }
}
