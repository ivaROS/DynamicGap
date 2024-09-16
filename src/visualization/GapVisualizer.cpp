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
        rawGapsFigPublisher = nh.advertise<visualization_msgs::MarkerArray>("raw_gaps_fig", 10);

        simpGapsPublisher = nh.advertise<visualization_msgs::Marker>("simp_gaps", 10);
        simpGapsFigPublisher = nh.advertise<visualization_msgs::MarkerArray>("simp_gaps_fig", 10);

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

        std_msgs::ColorRGBA gapwiseColor;
        gapwiseColor.a = 1.0;

        // (28, 124, 84)
        gapwiseColor.r = 28./255.; gapwiseColor.g = 124./255.; gapwiseColor.b = 84./255.;
        gapwiseColors.push_back(gapwiseColor);

        // (115, 226, 167)
        gapwiseColor.r = 115./255.; gapwiseColor.g = 226./255.; gapwiseColor.b = 167./255.;
        gapwiseColors.push_back(gapwiseColor);       

        // (53, 86, 16)
        gapwiseColor.r = 53./255.; gapwiseColor.g = 86./255.; gapwiseColor.b = 16./255.;
        gapwiseColors.push_back(gapwiseColor);    

        // (153, 188, 56)
        gapwiseColor.r = 153./255.; gapwiseColor.g = 188./255.; gapwiseColor.b = 56./255.;
        gapwiseColors.push_back(gapwiseColor);          

        // (27, 81, 45)
        gapwiseColor.r = 27./255.; gapwiseColor.g = 81./255.; gapwiseColor.b = 45./255.;
        gapwiseColors.push_back(gapwiseColor);    

        // (20, 235, 211)
        gapwiseColor.r = 20./255.; gapwiseColor.g = 235./255.; gapwiseColor.b = 211./255.;
        gapwiseColors.push_back(gapwiseColor);            
    }

    void GapVisualizer::drawGaps(const std::vector<dynamic_gap::Gap *> & gaps, const std::string & ns, const bool & initial) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = ns;
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);

        // visualization_msgs::Marker marker;
        // drawGap(marker, gaps, ns, true);

        visualization_msgs::MarkerArray markerArray;
        // for (dynamic_gap::Gap * gap : gaps) 
        drawGapMarkerArray(markerArray, gaps, ns, initial);

        if (ns == "raw") 
        {
            // rawGapsPublisher.publish(clearMarker);
            // rawGapsPublisher.publish(marker);
            rawGapsFigPublisher.publish(clearMarkerArray);
            rawGapsFigPublisher.publish(markerArray);
        } else if (ns == "simp") 
        {
            // simpGapsPublisher.publish(clearMarker);
            // simpGapsPublisher.publish(marker);
            simpGapsFigPublisher.publish(clearMarkerArray);
            simpGapsFigPublisher.publish(markerArray);
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
            float leftRange = initial ? gap->LRange() : gap->termLRange(); // initial ? gap->RDist() : gap->termRRange();
            float rightRange = initial ? gap->RDist() : gap->termRRange(); // initial ? gap->LRange() : gap->termLRange();

            //ROS_INFO_STREAM("leftIdx: " << leftIdx << ", ldist: " << ldist << ", rightIdx: " << rightIdx << ", rightRange: " << rightRange);
            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

            int num_segments = gapIdxSpan / min_resoln + 1;
            float distIncrement = (leftRange - rightRange) / num_segments;
            int midGapIdx = rightIdx; //  + viz_offset;
            float midGapDist = rightRange;

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

    void GapVisualizer::drawGapMarkerArray(visualization_msgs::MarkerArray & markerArray,
                                            const std::vector<dynamic_gap::Gap *> & gaps, 
                                            const std::string & ns, const bool & initial) 
    {
        visualization_msgs::Marker marker;

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

        float thickness = 0.05;
        marker.scale.x = thickness;     


        int counter = 0;
        for (dynamic_gap::Gap * gap : gaps) 
        {
            int id = (int) markerArray.markers.size();
            marker.header.frame_id = gap->frame_;

            // ROS_INFO_STREAM("ID: "<< id);

            gap->leftGapPtModel_->isolateGapDynamics();
            gap->rightGapPtModel_->isolateGapDynamics();

            Eigen::Vector4f leftGapState = gap->leftGapPtModel_->getGapState();
            Eigen::Vector4f rightGapState = gap->rightGapPtModel_->getGapState();


            int leftIdx, rightIdx;
            float leftRange, rightRange;

            if (initial)
            {
                leftIdx = gap->LIdx();
                rightIdx = gap->RIdx();
                leftRange = gap->LRange();
                rightRange = gap->RDist();
            } else
            {
                Eigen::Vector2f leftPtTerminal = leftGapState.head(2) + leftGapState.tail(2) * gap->gapLifespan_; 
                Eigen::Vector2f rightPtTerminal = rightGapState.head(2) + rightGapState.tail(2) * gap->gapLifespan_; 

                leftIdx = theta2idx( std::atan2(leftPtTerminal[1], leftPtTerminal[0]) );
                rightIdx = theta2idx( std::atan2(rightPtTerminal[1], rightPtTerminal[0]) );
                leftRange = leftPtTerminal.norm();
                rightRange = rightPtTerminal.norm();
            }

            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // int(2*gap->half_scan);

            int num_segments = gapIdxSpan / min_resoln + 1;
            float distIncrement = (leftRange - rightRange) / num_segments;
            int midGapIdx = rightIdx; //  + viz_offset;
            float midGapDist = rightRange;

            std_msgs::ColorRGBA ithGapColor = gapwiseColors.at(counter);
            if (!initial)
                ithGapColor.a = 0.5;

            marker.colors.clear();
            marker.colors.insert(marker.colors.begin(), 2, ithGapColor);

            float thickness = 0.05;
            marker.scale.x = thickness;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            geometry_msgs::Point midGapPt;
            midGapPt.z = 0.0005;
            std::vector<geometry_msgs::Point> midGapPts;

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

            counter++;
        }
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

            int leftIdx = initial ? gap->manipLeftIdx() : gap->manipTermLeftIdx();
            int rightIdx = initial ? gap->manipRightIdx() : gap->manipTermRightIdx();
            float leftRange = initial ? gap->manipLeftRange() : gap->manipTermLeftRange();
            float rightRange = initial ? gap->manipRightRange() : gap->manipTermRightRange();

            // ROS_INFO_STREAM("leftIdx: " << leftIdx << ", leftRange: " << leftRange);
            // ROS_INFO_STREAM("rightIdx: " << rightIdx << ", rightRange: " << rightRange);

            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

            // ROS_INFO_STREAM("gapIdxSpan: " << gapIdxSpan);

            int num_segments = gapIdxSpan / min_resoln + 1;
            float distIncrement = (leftRange - rightRange) / num_segments;
            int midGapIdx = rightIdx; //  + viz_offset;
            float midGapDist = rightRange;

            // ROS_INFO_STREAM("num_segments: " << num_segments);
            // ROS_INFO_STREAM("distIncrement: " << distIncrement);
            // ROS_INFO_STREAM("midGapIdx: " << midGapIdx);
            // ROS_INFO_STREAM("midGapDist: " << midGapDist);

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

                // ROS_INFO_STREAM("midGapIdx: " << midGapIdx);
                // ROS_INFO_STREAM("midGapDist: " << midGapDist);

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

        // float viz_jitter = cfg_->gap_viz.viz_jitter;
        // int viz_offset = 0;
        
        // if (viz_jitter > 0 && gap->isRadial(initial))
        // {
        //     viz_offset = gap->isRightType(initial) ? -2 : 2;
        // }

        int leftIdx = initial ? gap->manipLeftIdx() : gap->manipTermLeftIdx();
        int rightIdx = initial ? gap->manipRightIdx() : gap->manipTermRightIdx();
        float leftRange = initial ? gap->manipLeftRange() : gap->manipTermLeftRange();
        float rightRange = initial ? gap->manipRightRange() : gap->manipTermRightRange();

        std::string fullNamespace = ns;
        if (initial)
            fullNamespace.append("_initial");
        else
            fullNamespace.append("_terminal");

        int gapIdxSpan = (leftIdx - rightIdx);
        if (gapIdxSpan < 0)
            gapIdxSpan += cfg_->scan.full_scan; // int(2*gap->half_scan);

        int num_segments = gapIdxSpan / min_resoln + 1;
        float distIncrement = (leftRange - rightRange) / num_segments;
        int midGapIdx = rightIdx; //  + viz_offset;
        float midGapDist = rightRange;

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
        // midGapPt.x = rightRange * cos(rtheta);
        // midGapPt.y = rightRange * sin(rtheta);
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
        int gap_counter = 0;
        for (dynamic_gap::Gap * gap : gaps) 
        {
            drawGapModels(gapModelMarkerArray, gap, "gap_models", gap_counter); // gap_vel_error_arr
            // drawGapGroundTruthModels(gap_pos_GT_arr, gap_vel_GT_arr, gap, "gap_GT_models");
            gap_counter++;
        }
        // gapModelPosPublisher.publish(gap_pos_arr);
        gapModelsPublisher.publish(gapModelMarkerArray);
        // gapmodel_vel_error_publisher.publish(gap_vel_error_arr);
        // gapmodel_pos_GT_publisher.publish(gap_pos_GT_arr);
        // gapmodel_vel_GT_publisher.publish(gap_vel_GT_arr);
        // prev_num_models = 2* gaps.size();
    }

    void GapVisualizer::drawGapModels(visualization_msgs::MarkerArray & gapModelMarkerArray,
                                        dynamic_gap::Gap * gap, const std::string & ns,
                                        const int & gap_counter) // visualization_msgs::MarkerArray & gap_vel_error_arr,  
    {
        int id = (int) gapModelMarkerArray.markers.size();
        bool left = true;

        visualization_msgs::Marker leftModelMarker, rightModelMarker;
        // visualization_msgs::Marker leftGapPtModel_vel_error_pt, rightGapPtModel_vel_error_pt;

        drawModel(leftModelMarker, gap, left, id, ns, gap_counter);
        gapModelMarkerArray.markers.push_back(leftModelMarker);
        // draw_model_vel_error(leftGapPtModel_vel_error_pt, leftModelMarker, g, true, ns);
        // gap_vel_error_arr.markers.push_back(leftGapPtModel_vel_error_pt);

        drawModel(rightModelMarker, gap, !left, id, ns, gap_counter);
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
                                    dynamic_gap::Gap * gap, const bool & left, int & id, const std::string & ns,
                                    const int & gap_counter) 
    {
        modelMarker.header.frame_id = gap->frame_;
        modelMarker.header.stamp = ros::Time();
        modelMarker.ns = ns;
        modelMarker.id = id++;
        modelMarker.type = visualization_msgs::Marker::ARROW;
        modelMarker.action = visualization_msgs::Marker::ADD;
        
        gap->leftGapPtModel_->isolateGapDynamics();
        gap->rightGapPtModel_->isolateGapDynamics();

        Eigen::Vector4f leftModelState = gap->leftGapPtModel_->getGapState();
        Eigen::Vector4f rightModelState = gap->rightGapPtModel_->getGapState();

        Eigen::Vector2f gapVel(0.0, 0.0);
        if (left)
        {
            modelMarker.pose.position.x = leftModelState[0];
            modelMarker.pose.position.y = leftModelState[1];
            gapVel << leftModelState[2],
                      leftModelState[3];
        } else
        {
            modelMarker.pose.position.x = rightModelState[0];
            modelMarker.pose.position.y = rightModelState[1];            
            gapVel << rightModelState[2], 
                      rightModelState[3];
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

        modelMarker.color = gapwiseColors.at(gap_counter);
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

        /*
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
        */
    }

}
