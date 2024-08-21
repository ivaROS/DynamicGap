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
        rawGapsPublisher = nh.advertise<visualization_msgs::Marker>("raw_gaps", 10);
        simpGapsPublisher = nh.advertise<visualization_msgs::Marker>("simp_gaps", 10);
        
        manipGapsPublisher = nh.advertise<visualization_msgs::Marker>("manip_gaps", 10);
        navigableGapsPublisher = nh.advertise<visualization_msgs::Marker>("reachable_gaps", 10);
        
        gapModelsPublisher = nh.advertise<visualization_msgs::MarkerArray>("gap_models", 10);
        
        std_msgs::ColorRGBA rawInitial, rawTerminal, 
                            simpInitial, simpTerminal,
                            manipInitial, manipTerminal, 
                            reachable, 
                            gap_model, 
                            gapSplines;

        // Raw gaps
        std::vector<double> rawGapColorTriplet = {1.0, 0.0, 0.0};
        rawInitial.a = 1.0;
        rawInitial.r = rawGapColorTriplet[0];
        rawInitial.g = rawGapColorTriplet[1];
        rawInitial.b = rawGapColorTriplet[2];
        
        rawTerminal.a = 0.5;
        rawTerminal.r = rawGapColorTriplet[0];
        rawTerminal.g = rawGapColorTriplet[1];
        rawTerminal.b = rawGapColorTriplet[2];

        // Simplified gaps
        std::vector<double> simpGapColorTriplet = {1.0, 1.0, 0.0};
        simpInitial.a = 1.0;
        simpInitial.r = simpGapColorTriplet[0];
        simpInitial.g = simpGapColorTriplet[1];
        simpInitial.b = simpGapColorTriplet[2]; 

        simpTerminal.a = 0.5;
        simpTerminal.r = simpGapColorTriplet[0];
        simpTerminal.g = simpGapColorTriplet[1]; 
        simpTerminal.b = simpGapColorTriplet[2]; 

        // Manipulated gaps
        std::vector<double> manipGapColorTriplet = {0.0, 1.0, 0.0};
        manipInitial.a = 1.0;
        manipInitial.r = manipGapColorTriplet[0]; 
        manipInitial.g = manipGapColorTriplet[1];
        manipInitial.b = manipGapColorTriplet[2];

        manipTerminal.a = 0.5;
        manipTerminal.r = manipGapColorTriplet[0]; 
        manipTerminal.g = manipGapColorTriplet[1]; 
        manipTerminal.b = manipGapColorTriplet[2]; 

        // Reachable gaps
        reachable.a = 1.0;
        reachable.r = 1.0;
        reachable.g = 0.65;
        reachable.b = 0.0;

        // MODELS: PURPLE
        gap_model.a = 1.0;
        gap_model.r = 1;
        gap_model.g = 0;
        gap_model.b = 1;

        gapSplines.a = 1.0;
        gapSplines.r = 0.8;
        gapSplines.g = 1;
        gapSplines.b = 0;

        // colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw", raw));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_initial", rawInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_terminal", rawTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_initial", simpInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_terminal", simpTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("manip_initial", manipInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("manip_terminal", manipTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("reachable", reachable));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("gap_splines", gapSplines));
    }

    void GapVisualizer::drawGaps(const std::vector<dynamic_gap::Gap *> & gaps, const std::string & ns) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        // visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = ns;
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        // clearMarkerArray.markers.push_back(clearMarker);


        visualization_msgs::Marker marker;
        drawGap(marker, gaps, ns, true);

        // visualization_msgs::MarkerArray markerArray;
        // for (dynamic_gap::Gap * gap : gaps) 
        //     drawGap(markerArray, gap, ns, true);

        if (ns == "raw") 
        {
            rawGapsPublisher.publish(clearMarker);
            rawGapsPublisher.publish(marker);
        } else if (ns == "simp") 
        {
            simpGapsPublisher.publish(clearMarker);
            simpGapsPublisher.publish(marker);
        }
    }

    void GapVisualizer::drawGap(visualization_msgs::Marker & marker, const std::vector<dynamic_gap::Gap *> & gaps, 
                                const std::string & ns, const bool & initial)     
    {
        // ROS_INFO_STREAM("[drawGap] start");

        // visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;        

        float thickness = 0.05;
        marker.scale.x = thickness;     

        std::string fullNamespace = ns;        
        if (initial)
            fullNamespace.append("_initial");
        else
            fullNamespace.append("_terminal");

        // std::cout << "gap category: " << g.getCategory() << std::endl;
        //ROS_INFO_STREAM("ultimate local ns: " << fullNamespace);
        auto colorIter = colorMap.find(fullNamespace);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        marker.color = colorIter->second;
   
        for (dynamic_gap::Gap * gap : gaps) 
        {
            marker.header.frame_id = gap->frame_;

            int leftIdx = initial ? gap->LIdx() : gap->termLIdx(); // initial ? gap->RIdx() : gap->termRIdx(); //
            int rightIdx = initial ? gap->RIdx() : gap->termRIdx(); // initial ? gap->LIdx() : gap->termLIdx(); //
            float leftDist = initial ? gap->LDist() : gap->termLDist(); // initial ? gap->RDist() : gap->termRDist();
            float rightDist = initial ? gap->RDist() : gap->termRDist(); // initial ? gap->LDist() : gap->termLDist();

            //ROS_INFO_STREAM("leftIdx: " << leftIdx << ", ldist: " << ldist << ", rightIdx: " << rightIdx << ", rightDist: " << rightDist);
            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

            int num_segments = gapIdxSpan / min_resoln + 1;
            float distIncrement = (leftDist - rightDist) / num_segments;
            int midGapIdx = rightIdx; //  + viz_offset;
            float midGapDist = rightDist;

            float midGapTheta = 0.0;
            for (int i = 0; i < num_segments; i++)
            {
                geometry_msgs::Point p1;
                midGapTheta = idx2theta(midGapIdx);
                p1.x = midGapDist * cos(midGapTheta);
                p1.y = midGapDist * sin(midGapTheta);
                marker.points.push_back(p1);
                
                midGapIdx = (midGapIdx + min_resoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
                midGapDist += distIncrement;

                geometry_msgs::Point p2;
                midGapTheta = idx2theta(midGapIdx);
                p2.x = midGapDist * cos(midGapTheta);
                p2.y = midGapDist * sin(midGapTheta);
                marker.points.push_back(p2);
            }
        }
        // ROS_INFO_STREAM("[drawGap] end");
    }

    void GapVisualizer::drawManipGaps(const std::vector<dynamic_gap::Gap *> & gaps, const std::string & ns) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        // visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = ns;
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        manipGapsPublisher.publish(clearMarker);

        // visualization_msgs::MarkerArray markerArray;

        // bool circle = false;
        // for (dynamic_gap::Gap * gap : gaps) 
        // {
        //     // drawManipGap(markerArray, gap, ns, true); // initial
        //     drawManipGap(markerArray, gap, ns, false); // false
        // }
        // manipGapsPublisher.publish(markerArray);

        visualization_msgs::Marker marker;
        drawManipGap(marker, gaps, ns, true);

        manipGapsPublisher.publish(marker);

        // visualization_msgs::MarkerArray markerArray;
        // for (dynamic_gap::Gap * gap : gaps) 
        //     drawGap(markerArray, gap, ns, true)
    }

    void GapVisualizer::drawManipGap(visualization_msgs::Marker & marker, const std::vector<dynamic_gap::Gap *> & gaps, 
                                        const std::string & ns, const bool & initial)     
    {
        // ROS_INFO_STREAM("[drawManipGap] start");
        // visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;        

        float thickness = 0.05;
        marker.scale.x = thickness;     

        std::string fullNamespace = ns;        
        if (initial)
            fullNamespace.append("_initial");
        else
            fullNamespace.append("_terminal");

        // std::cout << "gap category: " << g.getCategory() << std::endl;
        //ROS_INFO_STREAM("ultimate local ns: " << fullNamespace);
        auto colorIter = colorMap.find(fullNamespace);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        marker.color = colorIter->second;
   
        for (dynamic_gap::Gap * gap : gaps) 
        {
            marker.header.frame_id = gap->frame_;

            int leftIdx = initial ? gap->cvxLeftIdx() : gap->cvxTermLeftIdx();
            int rightIdx = initial ? gap->cvxRightIdx() : gap->cvxTermRightIdx();
            float leftDist = initial ? gap->cvxLeftDist() : gap->cvxTermLeftDist();
            float rightDist = initial ? gap->cvxRightDist() : gap->cvxTermRightDist();

            //ROS_INFO_STREAM("leftIdx: " << leftIdx << ", ldist: " << ldist << ", rightIdx: " << rightIdx << ", rightDist: " << rightDist);
            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

            int num_segments = gapIdxSpan / min_resoln + 1;
            float distIncrement = (leftDist - rightDist) / num_segments;
            int midGapIdx = rightIdx; //  + viz_offset;
            float midGapDist = rightDist;

            float midGapTheta = 0.0;
            for (int i = 0; i < num_segments; i++)
            {
                geometry_msgs::Point p1;
                midGapTheta = idx2theta(midGapIdx);
                p1.x = midGapDist * cos(midGapTheta);
                p1.y = midGapDist * sin(midGapTheta);
                marker.points.push_back(p1);
                
                midGapIdx = (midGapIdx + min_resoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
                midGapDist += distIncrement;

                geometry_msgs::Point p2;
                midGapTheta = idx2theta(midGapIdx);
                p2.x = midGapDist * cos(midGapTheta);
                p2.y = midGapDist * sin(midGapTheta);
                marker.points.push_back(p2);
            }
        }
        // ROS_INFO_STREAM("[drawManipGap] end");
    }

    /*
    void GapVisualizer::drawManipGap(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap, 
                                        const std::string & ns, const bool & initial) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        // if ((initial && !gap->mode.reduced_ && !gap->mode.convex_ && !gap->mode.RGC_) ||
        //     (!initial && !gap->mode.termReduced_ && !gap->mode.termConvex_ && !gap->mode.termRGC_))
        //     return;

        // float viz_jitter = cfg_->gap_viz.viz_jitter;
        // int viz_offset = 0;
        
        // if (viz_jitter > 0 && gap->isRadial(initial))
        // {
        //     viz_offset = gap->isRightType(initial) ? -2 : 2;
        // }

        int leftIdx = initial ? gap->cvxLeftIdx() : gap->cvxTermLeftIdx();
        int rightIdx = initial ? gap->cvxRightIdx() : gap->cvxTermRightIdx();
        float leftDist = initial ? gap->cvxLeftDist() : gap->cvxTermLeftDist();
        float rightDist = initial ? gap->cvxRightDist() : gap->cvxTermRightDist();

        std::string fullNamespace = ns;
        if (initial)
            fullNamespace.append("_initial");
        else
            fullNamespace.append("_terminal");

        int gapIdxSpan = (leftIdx - rightIdx);
        if (gapIdxSpan < 0)
            gapIdxSpan += cfg_->scan.full_scan; // int(2*gap->half_scan);

        int num_segments = gapIdxSpan / min_resoln + 1;
        float distIncrement = (leftDist - rightDist) / num_segments;
        int midGapIdx = rightIdx; //  + viz_offset;
        float midGapDist = rightDist;

        visualization_msgs::Marker marker;
        marker.header.frame_id = gap->frame_;
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


        // ROS_INFO_STREAM("drawManipGap fullNamespace: " << fullNamespace);
        auto colorIter = colorMap.find(fullNamespace);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("[drawManipGaps] Visualization Color not found, return without drawing");
            return;
        }

        // marker.colors = colorIter->second;
        // marker.colors.push_back(colorIter->second);
        // marker.colors.push_back(colorIter->second);
        marker.colors.insert(marker.colors.begin(), 2, colorIter->second);

        float thickness = 0.05;
        marker.scale.x = thickness;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // geometry_msgs::Point linel;
        geometry_msgs::Point midGapPt;
        midGapPt.z = 0.0005;
        // linel.z = 0.0005;
        std::vector<geometry_msgs::Point> midGapPts;

        int id = (int) markerArray.markers.size();
        // ROS_INFO_STREAM("ID: "<< id);

        marker.lifetime = ros::Duration(0);

        float midGapTheta = 0.0;
        for (int i = 0; i < num_segments; i++)
        {
            midGapPts.clear();
            midGapTheta = idx2theta(midGapIdx);
            midGapPt.x = midGapDist * cos(midGapTheta);
            midGapPt.y = midGapDist * sin(midGapTheta);
            midGapPts.push_back(midGapPt);

            midGapIdx = (midGapIdx + min_resoln) % cfg_->scan.full_scan; // int(gap->half_scan * 2);
            midGapDist += distIncrement;

            midGapTheta = idx2theta(midGapIdx);
            midGapPt.x = midGapDist * cos(midGapTheta);
            midGapPt.y = midGapDist * sin(midGapTheta);
            midGapPts.push_back(midGapPt);

            marker.points = midGapPts;
            marker.id = id++;
            markerArray.markers.push_back(marker);
        }

        // close the last
        // midGapPts.clear();
        // midGapTheta = idx2theta(midGapIdx);
        // midGapPt.x = midGapDist * cos(midGapTheta);
        // midGapPt.y = midGapDist * sin(midGapTheta);
        // midGapPts.push_back(midGapPt);

        // float rtheta = idx2theta(rightIdx);
        // midGapPt.x = rightDist * cos(rtheta);
        // midGapPt.y = rightDist * sin(rtheta);
        // midGapPts.push_back(midGapPt);

        marker.scale.x = thickness;
        marker.points = midGapPts;
        marker.id = id++;
        marker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(marker);
    }
    */

    void GapVisualizer::drawGapsModels(const std::vector<dynamic_gap::Gap *> & gaps) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = "gap_models";
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);
        // gapModelPosPublisher.publish(clearMarkerArray);
        gapModelsPublisher.publish(clearMarkerArray);
        // gapmodel_pos_GT_publisher.publish(clearMarkerArray);
        // gapmodel_vel_GT_publisher.publish(clearMarkerArray);
        // gapmodel_vel_error_publisher.publish(clearMarkerArray);

        visualization_msgs::MarkerArray gapModelMarkerArray; // , gap_vel_error_arr , gap_pos_GT_arr, gap_vel_GT_arr;
        for (dynamic_gap::Gap * gap : gaps) 
        {
            drawGapModels(gapModelMarkerArray, gap, "gap_models"); // gap_vel_error_arr
            // drawGapGroundTruthModels(gap_pos_GT_arr, gap_vel_GT_arr, gap, "gap_GT_models");
        }
        // gapModelPosPublisher.publish(gap_pos_arr);
        gapModelsPublisher.publish(gapModelMarkerArray);
        // gapmodel_vel_error_publisher.publish(gap_vel_error_arr);
        // gapmodel_pos_GT_publisher.publish(gap_pos_GT_arr);
        // gapmodel_vel_GT_publisher.publish(gap_vel_GT_arr);
        // prev_num_models = 2* gaps.size();
    }

    void GapVisualizer::drawGapModels(visualization_msgs::MarkerArray & gapModelMarkerArray,
                                        dynamic_gap::Gap * gap, const std::string & ns) // visualization_msgs::MarkerArray & gap_vel_error_arr,  
    {
        int id = (int) gapModelMarkerArray.markers.size();
        bool left = true;

        visualization_msgs::Marker leftModelMarker, rightModelMarker;
        // visualization_msgs::Marker leftGapPtModel_vel_error_pt, rightGapPtModel_vel_error_pt;

        drawModel(leftModelMarker, gap, left, id, ns);
        gapModelMarkerArray.markers.push_back(leftModelMarker);
        // draw_model_vel_error(leftGapPtModel_vel_error_pt, leftModelMarker, g, true, ns);
        // gap_vel_error_arr.markers.push_back(leftGapPtModel_vel_error_pt);

        drawModel(rightModelMarker, gap, !left, id, ns);
        gapModelMarkerArray.markers.push_back(rightModelMarker);
        // draw_model_vel_error(rightGapPtModel_vel_error_pt, rightModelMarker, g, true, ns);
        // gap_vel_error_arr.markers.push_back(rightGapPtModel_vel_error_pt);
    }

    /*
    void GapVisualizer::drawGapGroundTruthModels(visualization_msgs::MarkerArray & gapModelMarkerArray, 
                                                    dynamic_gap::Gap * gap, std::string ns) 
    {
        int id = (int) gapModelMarkerArray.markers.size();

        visualization_msgs::Marker leftModelMarker, rightModelMarker;

        drawModel(leftModelMarker, gap, true, id, ns, true);
        gapModelMarkerArray.markers.push_back(leftModelMarker);

        drawModel(rightModelMarker, gap, false, id, ns, true);
        gapModelMarkerArray.markers.push_back(rightModelMarker);
    }
    */    

    void GapVisualizer::drawModel(visualization_msgs::Marker & modelMarker, 
                                    dynamic_gap::Gap * gap, const bool & left, int & id, const std::string & ns) 
    {
        modelMarker.header.frame_id = gap->frame_;
        modelMarker.header.stamp = ros::Time();
        modelMarker.ns = ns;
        modelMarker.id = id++;
        modelMarker.type = visualization_msgs::Marker::ARROW;
        modelMarker.action = visualization_msgs::Marker::ADD;
        
        Eigen::Vector4f leftModelState = gap->leftGapPtModel_->getState();
        Eigen::Vector4f rightModelState = gap->rightGapPtModel_->getState();
           
        Eigen::Vector2f gapVel(0.0, 0.0);
        if (left)
        {
            modelMarker.pose.position.x = leftModelState[0];
            modelMarker.pose.position.y = leftModelState[1];
            gapVel << leftModelState[2] + gap->leftGapPtModel_->getRobotVel().twist.linear.x,
                      leftModelState[3] + gap->leftGapPtModel_->getRobotVel().twist.linear.y;
        } else
        {
            modelMarker.pose.position.x = rightModelState[0];
            modelMarker.pose.position.y = rightModelState[1];            
            gapVel << rightModelState[2] + gap->rightGapPtModel_->getRobotVel().twist.linear.x, 
                      rightModelState[3] + gap->rightGapPtModel_->getRobotVel().twist.linear.y;
        }
        modelMarker.pose.position.z = 0.01;

        float gapVelTheta = std::atan2(gapVel[1], gapVel[0]);
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, gapVelTheta);

        modelMarker.pose.orientation.x = quat.getX();
        modelMarker.pose.orientation.y = quat.getY();
        modelMarker.pose.orientation.z = quat.getZ();
        modelMarker.pose.orientation.w = quat.getW();

        modelMarker.scale.x = gapVel.norm() + 0.000001;
        modelMarker.scale.y = 0.1;
        modelMarker.scale.z = 0.000001;

        modelMarker.color.a = 1.0;
        modelMarker.color.r = 1.0;
        modelMarker.color.b = 1.0;
        modelMarker.lifetime = ros::Duration(0);
    }

    void GapVisualizer::drawNavigableGaps(const std::vector<dynamic_gap::Gap *> & gaps,
                                            const int & highestScoreTrajIdx) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;
        
        // visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = "clear";
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        // clearMarkerArray.markers.push_back(clearMarker);
        navigableGapsPublisher.publish(clearMarker);

        // First, clearing topic.
        visualization_msgs::Marker marker;
        drawNavigableGap(marker, gaps, highestScoreTrajIdx);

        navigableGapsPublisher.publish(marker);
    }
    
    void GapVisualizer::drawNavigableGap(visualization_msgs::Marker & marker, 
                                            const std::vector<dynamic_gap::Gap *> & gaps,
                                            const int & highestScoreTrajIdx)     
    {
        // ROS_INFO_STREAM("[drawNavigableGap] start");

        // visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.ns = "navigableGaps";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0075;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;        

        float thickness = 0.05;
        marker.scale.x = thickness;

        // std::cout << "gap category: " << g.getCategory() << std::endl;
        //ROS_INFO_STREAM("ultimate local ns: " << fullNamespace);
        auto colorIter = colorMap.find("reachable");
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }        
        marker.color = colorIter->second;
   
        for (int i = 0; i < gaps.size(); i++) 
        {
            if (i != highestScoreTrajIdx)
                continue;

            dynamic_gap::Gap * gap = gaps.at(i);
            marker.header.frame_id = gap->frame_;
            
            Eigen::Vector2d curvePt(0.0, 0.0);        
            for (int j = 0; j < gap->allCurvePts_.rows() - 1; j++) 
            {  
                curvePt = gap->allCurvePts_.row(j);
                
                geometry_msgs::Point p1;
                p1.x = curvePt[0];
                p1.y = curvePt[1];
                marker.points.push_back(p1);

                // ROS_INFO_STREAM("connecting " << i << " to " << (i + 1));

                curvePt = gap->allCurvePts_.row(j + 1);
        
                geometry_msgs::Point p2;        
                p2.x = curvePt[0];
                p2.y = curvePt[1];
                marker.points.push_back(p2);
            }
        }
        // ROS_INFO_STREAM("[drawNavigableGap] end");
    }

}
