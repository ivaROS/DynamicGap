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
        
        rawGapModelPositionsPublisher = nh.advertise<visualization_msgs::MarkerArray>("raw_gap_model_positions", 10);
        simpGapModelPositionsPublisher = nh.advertise<visualization_msgs::MarkerArray>("simp_gap_model_positions", 10);
        manipGapModelPositionsPublisher = nh.advertise<visualization_msgs::MarkerArray>("manip_gap_model_positions", 10);

        rawGapModelVelocitiesPublisher = nh.advertise<visualization_msgs::MarkerArray>("raw_gap_model_velocities", 10);
        simpGapModelVelocitiesPublisher = nh.advertise<visualization_msgs::MarkerArray>("simp_gap_model_velocities", 10);
        manipGapModelVelocitiesPublisher = nh.advertise<visualization_msgs::MarkerArray>("manip_gap_model_velocities", 10);

        gapTubePublisher = nh.advertise<visualization_msgs::MarkerArray>("gap_tubes", 10);

        std_msgs::ColorRGBA rawInitial, rawTerminal, rawTmin1, 
                            simpInitial, simpTerminal, simpTmin1,
                            manipInitial, manipTerminal, 
                            reachable, 
                            gap_model, 
                            gapSplines;

        // Raw gaps
        std::vector<double> rawGapColorTriplet = {0.08235294117, 0.47450980392, 0.08235294117};
        rawTmin1.a = 0.5;
        rawTmin1.r = rawGapColorTriplet[0];
        rawTmin1.g = rawGapColorTriplet[1];
        rawTmin1.b = rawGapColorTriplet[2];         
        
        rawInitial.a = 1.0;
        rawInitial.r = rawGapColorTriplet[0];
        rawInitial.g = rawGapColorTriplet[1];
        rawInitial.b = rawGapColorTriplet[2];
        
        rawTerminal.a = 0.5;
        rawTerminal.r = rawGapColorTriplet[0];
        rawTerminal.g = rawGapColorTriplet[1];
        rawTerminal.b = rawGapColorTriplet[2];

        // Simplified gaps
        std::vector<double> simpGapColorTriplet = {0.6, 0.0, 1.0};
        simpTmin1.a = 0.5;
        simpTmin1.r = simpGapColorTriplet[0];
        simpTmin1.g = simpGapColorTriplet[1];
        simpTmin1.b = simpGapColorTriplet[2]; 

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
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_tmin1", rawTmin1));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_initial", rawInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_terminal", rawTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_tmin1", simpTmin1));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_initial", simpInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_terminal", simpTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("manip_initial", manipInitial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("manip_terminal", manipTerminal));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("reachable", reachable));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("gap_splines", gapSplines));        
    }

    void GapVisualizer::drawGaps(const std::vector<Gap *> & gaps, const std::string & ns) 
    {
        // First, clearing topic.
        clearMarkerPublisher(rawGapsPublisher);
        clearMarkerPublisher(simpGapsPublisher);

        visualization_msgs::Marker marker;
        drawGap(marker, gaps, ns); // , true);

        if (ns.find("raw") != std::string::npos) 
        {
            rawGapsPublisher.publish(marker);
        } else if (ns.find("simp") != std::string::npos) 
        {
            simpGapsPublisher.publish(marker);
        }
    }

    void GapVisualizer::drawGap(visualization_msgs::Marker & marker, const std::vector<Gap *> & gaps, 
                                const std::string & ns) // , const bool & initial)     
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
        
        if (fullNamespace == "simp_tmin1" || fullNamespace ==  "raw_tmin1")
        {
            marker.id = 0;
        } else
        {
            fullNamespace.append("_initial");
            marker.id = 1;
        } 

        // if (initial)
        //     fullNamespace.append("_initial");
        // else
        //     fullNamespace.append("_terminal");

        // std::cout << "gap category: " << g.getCategory() << std::endl;
        //ROS_INFO_STREAM("ultimate local ns: " << fullNamespace);
        auto colorIter = colorMap.find(fullNamespace);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        marker.color = colorIter->second;
   
        for (Gap * gap : gaps) 
        {
            if (gap->getFrame().empty())
            {
                ROS_WARN_STREAM("[drawGap] Gap frame is empty");
                return;
            }

            marker.header.frame_id = gap->getFrame();

            int leftIdx = gap->LIdx(); // initial ?  : gap->termLIdx(); // initial ? gap->RIdx() : gap->termRIdx(); //
            int rightIdx = gap->RIdx(); // initial ?  : gap->termRIdx(); // initial ? gap->LIdx() : gap->termLIdx(); //
            float leftRange = gap->LRange(); // initial ?  : gap->termLRange(); // initial ? gap->RRange() : gap->termRRange();
            float rightRange = gap->RRange(); // initial ?  : gap->termRRange(); // initial ? gap->LRange() : gap->termLRange();

            //ROS_INFO_STREAM("leftIdx: " << leftIdx << ", ldist: " << ldist << ", rightIdx: " << rightIdx << ", rightRange: " << rightRange);
            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

            int num_segments = int(invGapSpanResoln * gapIdxSpan) + 1;
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
                
                midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
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


    void GapVisualizer::drawGapModels(const std::vector<Gap *> & gaps, const std::string & ns) 
    {
        /////////////////////////////
        // Draw gap goal positions //
        /////////////////////////////
        visualization_msgs::MarkerArray gapModelPositionMarkerArray;
        for (Gap * gap : gaps) 
        {
            drawGapModelPositions(gapModelPositionMarkerArray, gap, ns);
        }

        if (ns.find("raw") != std::string::npos) 
        {
            // First, clearing topic.
            clearMarkerArrayPublisher(rawGapModelPositionsPublisher);

            rawGapModelPositionsPublisher.publish(gapModelPositionMarkerArray);
        } else if (ns.find("simp") != std::string::npos) 
        {
            // First, clearing topic.
            clearMarkerArrayPublisher(simpGapModelPositionsPublisher);

            simpGapModelPositionsPublisher.publish(gapModelPositionMarkerArray);
        }

        //////////////////////////////
        // Draw gap goal velocities // 
        //////////////////////////////        
        visualization_msgs::MarkerArray gapModelVelocityMarkerArray;
        for (Gap * gap : gaps) 
        {
            drawGapModelVelocities(gapModelVelocityMarkerArray, gap, ns);
        }

        if (ns.find("raw") != std::string::npos) 
        {
            // First, clearing topic.
            clearMarkerArrayPublisher(rawGapModelVelocitiesPublisher);

            rawGapModelVelocitiesPublisher.publish(gapModelVelocityMarkerArray);
        } else if (ns.find("simp") != std::string::npos) 
        {
            // First, clearing topic.
            clearMarkerArrayPublisher(simpGapModelVelocitiesPublisher);

            simpGapModelVelocitiesPublisher.publish(gapModelVelocityMarkerArray);
        }
    }

    void GapVisualizer::drawGapModelPositions(visualization_msgs::MarkerArray & gapModelMarkerArray,
                                                Gap * gap, const std::string & ns)
    {
        int id = (int) gapModelMarkerArray.markers.size();
        bool left = true;

        visualization_msgs::Marker leftModelMarker, rightModelMarker;

        drawModelPosition(leftModelMarker, gap, left, id, ns);
        gapModelMarkerArray.markers.push_back(leftModelMarker);

        drawModelPosition(rightModelMarker, gap, !left, id, ns);
        gapModelMarkerArray.markers.push_back(rightModelMarker);
    }


    void GapVisualizer::drawModelPosition(visualization_msgs::Marker & modelMarker, 
                                            Gap * gap, const bool & left, int & id, const std::string & ns) 
    {
        if (gap->getFrame().empty())
        {
            ROS_WARN_STREAM("[drawModel] Gap frame is empty");
            return;
        }
        
        std::string fullNamespace = ns;
        fullNamespace.append("_initial");

        auto colorIter = colorMap.find(fullNamespace);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        modelMarker.color = colorIter->second;

        // ROS_INFO_STREAM("[drawModel()]");
        modelMarker.header.frame_id = gap->getFrame();
        modelMarker.header.stamp = ros::Time();
        modelMarker.ns = ns;
        modelMarker.id = id++;
        modelMarker.type = visualization_msgs::Marker::CYLINDER;
        modelMarker.action = visualization_msgs::Marker::ADD;
        
        gap->getLeftGapPt()->getModel()->isolateGapDynamics();
        gap->getRightGapPt()->getModel()->isolateGapDynamics();

        Eigen::Vector4f leftModelState = gap->getLeftGapPt()->getModel()->getGapState();
        Eigen::Vector4f rightModelState = gap->getRightGapPt()->getModel()->getGapState();

        // ROS_INFO_STREAM("   leftModelState: " << leftModelState.transpose());
        // ROS_INFO_STREAM("   rightModelState: " << rightModelState.transpose());

        // ROS_INFO_STREAM("   gap->getLeftGapPt()->getModel()->getRobotVel(): " << gap->getLeftGapPt()->getModel()->getRobotVel());
        // ROS_INFO_STREAM("   gap->getRightGapPt()->getModel()->getRobotVel(): " << gap->getRightGapPt()->getModel()->getRobotVel());

        Eigen::Vector2f gapVel(0.0, 0.0);
        if (left)
        {
            modelMarker.pose.position.x = leftModelState[0];
            modelMarker.pose.position.y = leftModelState[1];
            gapVel = leftModelState.tail(2);
        } else
        {
            modelMarker.pose.position.x = rightModelState[0];
            modelMarker.pose.position.y = rightModelState[1];            
            gapVel << rightModelState.tail(2); 
        }
        modelMarker.pose.position.z = 0.01;
        
        modelMarker.pose.orientation.x = 0.0;
        modelMarker.pose.orientation.y = 0.0;
        modelMarker.pose.orientation.z = 0.0;
        modelMarker.pose.orientation.w = 1.0;

        modelMarker.scale.x = 0.2;
        modelMarker.scale.y = 0.2;
        modelMarker.scale.z = 0.000001;

        // modelMarker.color.a = 1.0;
        // modelMarker.color.r = 1.0;
        // modelMarker.color.b = 1.0;
        // modelMarker.lifetime = ros::Duration(0);
    }

    void GapVisualizer::drawGapModelVelocities(visualization_msgs::MarkerArray & gapModelMarkerArray,
                                                Gap * gap, const std::string & ns)
    {
        int id = (int) gapModelMarkerArray.markers.size();
        bool left = true;

        visualization_msgs::Marker leftModelMarker, rightModelMarker;

        drawModelVelocity(leftModelMarker, gap, left, id, ns);
        gapModelMarkerArray.markers.push_back(leftModelMarker);

        drawModelVelocity(rightModelMarker, gap, !left, id, ns);
        gapModelMarkerArray.markers.push_back(rightModelMarker);
    }


    void GapVisualizer::drawModelVelocity(visualization_msgs::Marker & modelMarker, 
                                            Gap * gap, const bool & left, int & id, const std::string & ns) 
    {
        if (gap->getFrame().empty())
        {
            ROS_WARN_STREAM("[drawModel] Gap frame is empty");
            return;
        }
        
        std::string fullNamespace = ns;
        fullNamespace.append("_initial");

        auto colorIter = colorMap.find(fullNamespace);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        modelMarker.color = colorIter->second;

        // ROS_INFO_STREAM("[drawModel()]");
        modelMarker.header.frame_id = gap->getFrame();
        modelMarker.header.stamp = ros::Time();
        modelMarker.ns = ns;
        // modelMarker.id = id++; // this is not my social cost code, it's just for visualization, but I"m going to reuse it for quick testing 
        if (left)
            {
                modelMarker.id = gap->getLeftGapPt()->getModel()->getID();
            }
            else
            {
                modelMarker.id = gap->getRightGapPt()->getModel()->getID();
            }

        modelMarker.type = visualization_msgs::Marker::ARROW;
        modelMarker.action = visualization_msgs::Marker::ADD;
        
        gap->getLeftGapPt()->getModel()->isolateGapDynamics();
        gap->getRightGapPt()->getModel()->isolateGapDynamics();

        Eigen::Vector4f leftModelState = gap->getLeftGapPt()->getModel()->getGapState();
        Eigen::Vector4f rightModelState = gap->getRightGapPt()->getModel()->getGapState();

        // ROS_INFO_STREAM("   leftModelState: " << leftModelState.transpose());
        // ROS_INFO_STREAM("   rightModelState: " << rightModelState.transpose());

        // ROS_INFO_STREAM("   gap->getLeftGapPt()->getModel()->getRobotVel(): " << gap->getLeftGapPt()->getModel()->getRobotVel());
        // ROS_INFO_STREAM("   gap->getRightGapPt()->getModel()->getRobotVel(): " << gap->getRightGapPt()->getModel()->getRobotVel());

        Eigen::Vector2f gapVel(0.0, 0.0);
        if (left)
        {
            modelMarker.pose.position.x = leftModelState[0];
            modelMarker.pose.position.y = leftModelState[1];
            gapVel = leftModelState.tail(2);

            bool isRaw  = ns.find("raw")  != std::string::npos;
            bool isSimp = ns.find("simp") != std::string::npos;

            // ROS_ERROR_STREAM_NAMED("Visualizer",
            //     "[drawModelVelocity] gapVel=" << gapVel.transpose()
            //     << "  type=" << (isRaw ? "RAW" : (isSimp ? "SIMP" : "OTHER"))
            //     << "  ns=" << ns);


        } else
        {
            modelMarker.pose.position.x = rightModelState[0];
            modelMarker.pose.position.y = rightModelState[1];            
            gapVel << rightModelState.tail(2); 

            bool isRaw  = ns.find("raw")  != std::string::npos;
            bool isSimp = ns.find("simp") != std::string::npos;

             ROS_ERROR_STREAM_NAMED("Visualizer",
                "[drawModelVelocity] right gapVel=" << gapVel.transpose()
                << "  type=" << (isRaw ? "RAW" : (isSimp ? "SIMP" : "OTHER"))
                << "  ns=" << ns);
        }
        modelMarker.pose.position.z = 0.01;

        float gapVelTheta;
        if (gapVel.norm() < std::numeric_limits<float>::epsilon())
        {
            gapVelTheta = 0.0;
        } else
        {
            gapVelTheta = std::atan2(gapVel[1], gapVel[0]);

        }

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, gapVelTheta);
        
        modelMarker.pose.orientation.x = quat.getX();
        modelMarker.pose.orientation.y = quat.getY();
        modelMarker.pose.orientation.z = quat.getZ();
        modelMarker.pose.orientation.w = quat.getW();

        modelMarker.scale.x = gapVel.norm() + 0.000001;
        modelMarker.scale.y = 0.1;
        modelMarker.scale.z = 0.000001;

        // modelMarker.color.a = 1.0;
        // modelMarker.color.r = 1.0;
        // modelMarker.color.b = 1.0;
        // modelMarker.lifetime = ros::Duration(0);
    }

    void GapVisualizer::drawGapTubes(const std::vector<GapTube *> & gapTubes)
    {
        ROS_INFO_STREAM_NAMED("GapVisualizer", "[drawGapTubes]");

        clearMarkerArrayPublisher(gapTubePublisher);

        visualization_msgs::MarkerArray gapTubeMarkerArray;
        
        ros::Time time = ros::Time::now();

        std::string ns = "manip_initial";

        auto colorIter = colorMap.find(ns);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        ////////////////////////////////////
        // Visualize all gaps in gap tube //
        ////////////////////////////////////

        for (int i = 0; i < gapTubes.size(); i++)
        {
            ROS_INFO_STREAM_NAMED("GapVisualizer", "    gapTube: " << i);

            GapTube * gapTube = gapTubes.at(i);
            int gapTubeSize = gapTube->size();

            for (int j = 0; j < gapTubeSize; j++)
            {
                ROS_INFO_STREAM_NAMED("GapVisualizer", "        gap: " << j);

                Gap * gap = gapTube->at(j);

                // make a marker for each gap
                visualization_msgs::Marker marker;

                marker.header.frame_id = gap->getFrame();
                marker.header.stamp = time;
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

                float eps = 0.000000001;
                marker.color = visionColors_.at(i % visionColors_.size());
                marker.color.a = 1.0 - 0.5 * (float (j) / (float(gapTubeSize - 1) + eps));

                ROS_INFO_STREAM_NAMED("GapVisualizer", "        color: " << marker.color.r << ", " << marker.color.g << ", " << marker.color.b << ", " << marker.color.a);
    
                int leftIdx = gap->manipLeftIdx(); // initial ?  : gap->manipTermLeftIdx();
                int rightIdx = gap->manipRightIdx(); // initial ?  : gap->manipTermRightIdx();
                float leftRange = gap->manipLeftRange(); // initial ?  : gap->manipTermLeftRange();
                float rightRange = gap->manipRightRange(); // initial ?  : gap->manipTermRightRange();
    
                ROS_INFO_STREAM_NAMED("GapVisualizer", "        leftIdx: " << leftIdx << ", leftRange: " << leftRange);
                ROS_INFO_STREAM_NAMED("GapVisualizer", "        rightIdx: " << rightIdx << ", rightRange: " << rightRange);
    
                int gapIdxSpan = (leftIdx - rightIdx);
                if (gapIdxSpan < 0)
                    gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here
    
                ROS_INFO_STREAM_NAMED("GapVisualizer", "        gapIdxSpan: " << gapIdxSpan);
    
                int num_segments = int(invGapSpanResoln * gapIdxSpan) + 1;
                float distIncrement = (leftRange - rightRange) / num_segments;
                int midGapIdx = rightIdx; //  + viz_offset;
                float midGapDist = rightRange;
    
                ROS_INFO_STREAM_NAMED("GapVisualizer", "        num_segments: " << num_segments);
                ROS_INFO_STREAM_NAMED("GapVisualizer", "        distIncrement: " << distIncrement);
                ROS_INFO_STREAM_NAMED("GapVisualizer", "        midGapIdx: " << midGapIdx);
                ROS_INFO_STREAM_NAMED("GapVisualizer", "        midGapDist: " << midGapDist);
    
                float midGapTheta = 0.0;
                for (int k = 0; k < num_segments; k++)
                {
                    if (!gap->isAvailable() && (k % 2 == 1))
                    {
                        // skip the middle point
                        midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
                        midGapDist += distIncrement;
                        continue;
                    }


                    geometry_msgs::Point p1;
                    midGapTheta = idx2theta(midGapIdx);
                    p1.x = midGapDist * cos(midGapTheta);
                    p1.y = midGapDist * sin(midGapTheta);
                    marker.points.push_back(p1);
                    
                    midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
                    midGapDist += distIncrement;
    
                    // ROS_INFO_STREAM("midGapIdx: " << midGapIdx);
                    // ROS_INFO_STREAM("midGapDist: " << midGapDist);
    
                    geometry_msgs::Point p2;
                    midGapTheta = idx2theta(midGapIdx);
                    p2.x = midGapDist * cos(midGapTheta);
                    p2.y = midGapDist * sin(midGapTheta);
                    marker.points.push_back(p2);
                }                

                // marker.lifetime = ros::Duration(0);
                marker.id = gapTubeMarkerArray.markers.size();
                gapTubeMarkerArray.markers.push_back(marker);


            }
        }

        /////////////////////////////////////////////////////////////////////////
        // Visualize initial state of first gap and terminal state of last gap //
        /////////////////////////////////////////////////////////////////////////

        // for (int i = 0; i < gapTubes.size(); i++)
        // {
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "    gapTube: " << i);

        //     GapTube * gapTube = gapTubes.at(i);
        //     int gapTubeSize = gapTube->size();

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        gap: 0");

        //     Gap * gap = gapTube->at(0);

        //     // make a marker for each gap
        //     visualization_msgs::Marker marker;

        //     marker.header.frame_id = gap->getFrame();
        //     marker.header.stamp = time;
        //     marker.ns = ns;
        //     marker.type = visualization_msgs::Marker::LINE_LIST;
        //     marker.action = visualization_msgs::Marker::ADD;
    
        //     marker.pose.position.x = 0.0;
        //     marker.pose.position.y = 0.0;
        //     marker.pose.position.z = 0.0;
        //     marker.pose.orientation.x = 0.0;
        //     marker.pose.orientation.y = 0.0;
        //     marker.pose.orientation.z = 0.0;
        //     marker.pose.orientation.w = 1.0;        
    
        //     float thickness = 0.10;
        //     marker.scale.x = thickness;     

        //     marker.color = visionColors_.at(i % visionColors_.size());
        //     marker.color.a = 1.0;

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        color: " << marker.color.r << ", " << marker.color.g << ", " << marker.color.b << ", " << marker.color.a);

        //     int leftIdx = gap->manipLeftIdx(); // initial ?  : gap->manipTermLeftIdx();
        //     int rightIdx = gap->manipRightIdx(); // initial ?  : gap->manipTermRightIdx();
        //     float leftRange = gap->manipLeftRange(); // initial ?  : gap->manipTermLeftRange();
        //     float rightRange = gap->manipRightRange(); // initial ?  : gap->manipTermRightRange();

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        leftIdx: " << leftIdx << ", leftRange: " << leftRange);
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        rightIdx: " << rightIdx << ", rightRange: " << rightRange);

        //     int gapIdxSpan = (leftIdx - rightIdx);
        //     if (gapIdxSpan < 0)
        //         gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        gapIdxSpan: " << gapIdxSpan);

        //     int num_segments = int(invGapSpanResoln * gapIdxSpan) + 1;
        //     float distIncrement = (leftRange - rightRange) / num_segments;
        //     int midGapIdx = rightIdx; //  + viz_offset;
        //     float midGapDist = rightRange;

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        num_segments: " << num_segments);
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        distIncrement: " << distIncrement);
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        midGapIdx: " << midGapIdx);
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        midGapDist: " << midGapDist);

        //     float midGapTheta = 0.0;
        //     for (int k = 0; k < num_segments; k++)
        //     {
        //         geometry_msgs::Point p1;
        //         midGapTheta = idx2theta(midGapIdx);
        //         p1.x = midGapDist * cos(midGapTheta);
        //         p1.y = midGapDist * sin(midGapTheta);
        //         marker.points.push_back(p1);
                
        //         midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
        //         midGapDist += distIncrement;

        //         // ROS_INFO_STREAM("midGapIdx: " << midGapIdx);
        //         // ROS_INFO_STREAM("midGapDist: " << midGapDist);

        //         geometry_msgs::Point p2;
        //         midGapTheta = idx2theta(midGapIdx);
        //         p2.x = midGapDist * cos(midGapTheta);
        //         p2.y = midGapDist * sin(midGapTheta);
        //         marker.points.push_back(p2);
        //     }                
            
        //     // marker.lifetime = ros::Duration(0);
        //     marker.id = gapTubeMarkerArray.markers.size();
        //     gapTubeMarkerArray.markers.push_back(marker);

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        gap: " << (gapTubeSize - 1));

        //     marker = visualization_msgs::Marker();

        //     gap = gapTube->at(gapTubeSize - 1);

        //     marker.header.frame_id = gap->getFrame();
        //     marker.header.stamp = time;
        //     marker.ns = ns;
        //     marker.type = visualization_msgs::Marker::LINE_LIST;
        //     marker.action = visualization_msgs::Marker::ADD;
    
        //     marker.pose.position.x = 0.0;
        //     marker.pose.position.y = 0.0;
        //     marker.pose.position.z = 0.0;
        //     marker.pose.orientation.x = 0.0;
        //     marker.pose.orientation.y = 0.0;
        //     marker.pose.orientation.z = 0.0;
        //     marker.pose.orientation.w = 1.0;        
    
        //     marker.scale.x = thickness;     

        //     marker.color = visionColors_.at(i % visionColors_.size());
        //     marker.color.a = 0.5;

        //     Eigen::Vector2f leftGapPos = gap->getManipulatedLPosition();
        //     Eigen::Vector2f rightGapPos = gap->getManipulatedRPosition();
        //     Eigen::Vector2f leftGapVel = gap->getManipulatedLVelocity();
        //     Eigen::Vector2f rightGapVel = gap->getManipulatedRVelocity();

        //     float gapLifespan = gap->getGapLifespan();

        //     Eigen::Vector2f termLeftGapPos = leftGapPos + gapLifespan * leftGapVel;
        //     Eigen::Vector2f termRightGapPos = rightGapPos + gapLifespan * rightGapVel;

        //     float termLeftGapTheta = std::atan2(termLeftGapPos[1], termLeftGapPos[0]);
        //     float termRightGapTheta = std::atan2(termRightGapPos[1], termRightGapPos[0]);

        //     int termLeftIdx = theta2idx(termLeftGapTheta); // initial ?  : gap->manipTermLeftIdx();
        //     int termRightIdx = theta2idx(termRightGapTheta); // initial ?  : gap->manipTermRightIdx();
        //     float termLeftRange = termLeftGapPos.norm(); // initial ?  : gap->manipTermLeftRange();
        //     float termRightRange = termRightGapPos.norm(); // initial ?  : gap->manipTermRightRange();

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        termLeftIdx: " << termLeftIdx << ", termLeftRange: " << termLeftRange);
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        termRightIdx: " << termRightIdx << ", termRightRange: " << termRightRange);

        //     int termGapIdxSpan = (termLeftIdx - termRightIdx);
        //     if (termGapIdxSpan < 0)
        //     termGapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        termGapIdxSpan: " << termGapIdxSpan);

        //     int term_num_segments = int(invGapSpanResoln * termGapIdxSpan) + 1;
        //     float termDistIncrement = (termLeftRange - termRightRange) / term_num_segments;
        //     int termMidGapIdx = termRightIdx; //  + viz_offset;
        //     float termMidGapDist = termRightRange;

        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        term_num_segments: " << term_num_segments);
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        termDistIncrement: " << termDistIncrement);
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        termMidGapIdx: " << termMidGapIdx);
        //     ROS_INFO_STREAM_NAMED("GapVisualizer", "        termMidGapDist: " << termMidGapDist);

        //     float termMidGapTheta = 0.0;
        //     for (int k = 0; k < term_num_segments; k++)
        //     {
        //         geometry_msgs::Point p1;
        //         termMidGapTheta = idx2theta(termMidGapIdx);
        //         p1.x = termMidGapDist * cos(termMidGapTheta);
        //         p1.y = termMidGapDist * sin(termMidGapTheta);
        //         marker.points.push_back(p1);
                
        //         termMidGapIdx = (termMidGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
        //         termMidGapDist += termDistIncrement;

        //         // ROS_INFO_STREAM("midGapIdx: " << midGapIdx);
        //         // ROS_INFO_STREAM("midGapDist: " << midGapDist);

        //         geometry_msgs::Point p2;
        //         termMidGapTheta = idx2theta(termMidGapIdx);
        //         p2.x = termMidGapDist * cos(termMidGapTheta);
        //         p2.y = termMidGapDist * sin(termMidGapTheta);
        //         marker.points.push_back(p2);
        //     }                
            
        //     // marker.lifetime = ros::Duration(0);
        //     marker.id = gapTubeMarkerArray.markers.size();
        //     gapTubeMarkerArray.markers.push_back(marker);

        // }

        gapTubePublisher.publish(gapTubeMarkerArray);        
    }

    void GapVisualizer::drawManipGaps(const std::vector<Gap *> & gaps, const std::string & ns) 
    {
        // First, clearing topic.
        clearMarkerPublisher(manipGapsPublisher);

        visualization_msgs::Marker marker;
        drawManipGap(marker, gaps, ns); // , true);

        manipGapsPublisher.publish(marker);
    }

    void GapVisualizer::drawManipGap(visualization_msgs::Marker & marker, const std::vector<Gap *> & gaps, 
                                        const std::string & ns) // , const bool & initial)     
    {
        // ROS_INFO_STREAM("[drawManipGap] start");
        // visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.01;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;        

        float thickness = 0.05;
        marker.scale.x = thickness;     

        std::string fullNamespace = ns;
        fullNamespace.append("_initial");        
        // if (initial)
        //     fullNamespace.append("_initial");
        // else
        //     fullNamespace.append("_terminal");

        // std::cout << "gap category: " << g.getCategory() << std::endl;
        //ROS_INFO_STREAM("ultimate local ns: " << fullNamespace);
        auto colorIter = colorMap.find(fullNamespace);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        marker.color = colorIter->second;
   
        for (Gap * gap : gaps) 
        {
            if (gap->getFrame().empty())
            {
                ROS_WARN_STREAM("[drawManipGap] Gap frame is empty");
                return;
            }

            marker.header.frame_id = gap->getFrame();

            int leftIdx = gap->manipLeftIdx(); // initial ?  : gap->manipTermLeftIdx();
            int rightIdx = gap->manipRightIdx(); // initial ?  : gap->manipTermRightIdx();
            float leftRange = gap->manipLeftRange(); // initial ?  : gap->manipTermLeftRange();
            float rightRange = gap->manipRightRange(); // initial ?  : gap->manipTermRightRange();

            // ROS_INFO_STREAM("leftIdx: " << leftIdx << ", leftRange: " << leftRange);
            // ROS_INFO_STREAM("rightIdx: " << rightIdx << ", rightRange: " << rightRange);

            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

            // ROS_INFO_STREAM("gapIdxSpan: " << gapIdxSpan);

            int num_segments = int(invGapSpanResoln * gapIdxSpan) + 1;
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
                
                midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
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


    void GapVisualizer::drawManipGapModels(const std::vector<Gap *> & gaps, const std::string & ns) 
    {
        /////////////////////////////
        // Draw gap goal positions //
        /////////////////////////////
        visualization_msgs::MarkerArray gapModelPositionMarkerArray;

        for (int i = 0; i < gaps.size(); i++)
        // for (Gap * gap : gaps) 
        {
            Gap * gap = gaps.at(i);
            drawManipGapModelPositions(gapModelPositionMarkerArray, i, gap, ns);
        }

        // First, clearing topic.
        clearMarkerArrayPublisher(manipGapModelPositionsPublisher);

        manipGapModelPositionsPublisher.publish(gapModelPositionMarkerArray);

        //////////////////////////////
        // Draw gap goal velocities //
        //////////////////////////////        
        visualization_msgs::MarkerArray gapModelVelocityMarkerArray;

        for (int i = 0; i < gaps.size(); i++)
        // for (Gap * gap : gaps) 
        {
            Gap * gap = gaps.at(i);
            drawManipGapModelVelocities(gapModelVelocityMarkerArray, i, gap, ns);
        }

        // First, clearing topic.
        clearMarkerArrayPublisher(manipGapModelVelocitiesPublisher);

        manipGapModelVelocitiesPublisher.publish(gapModelVelocityMarkerArray);
    }

    void GapVisualizer::drawManipGapModelPositions(visualization_msgs::MarkerArray & gapModelMarkerArray,
                                                    const int & gapIdx, Gap * gap, const std::string & ns)
    {
        int id = (int) gapModelMarkerArray.markers.size();
        bool left = true;

        visualization_msgs::Marker leftModelMarker, rightModelMarker;

        drawManipModelPosition(leftModelMarker, gapIdx, gap, left, id, ns);
        gapModelMarkerArray.markers.push_back(leftModelMarker);

        drawManipModelPosition(rightModelMarker, gapIdx, gap, !left, id, ns);
        gapModelMarkerArray.markers.push_back(rightModelMarker);
    }


    void GapVisualizer::drawManipModelPosition(visualization_msgs::Marker & modelMarker, 
                                                const int & gapIdx, Gap * gap, const bool & left, int & id, const std::string & ns) 
    {
        if (gap->getFrame().empty())
        {
            ROS_WARN_STREAM("[drawModel] Gap frame is empty");
            return;
        }
        
        // std::string fullNamespace = ns;
        // fullNamespace.append("_initial");

        // auto colorIter = colorMap.find(fullNamespace);
        // if (colorIter == colorMap.end()) 
        // {
        //     ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
        //     return;
        // }

        // modelMarker.color = colorIter->second;

        modelMarker.color = visionColors_.at(gapIdx % visionColors_.size());

        // ROS_INFO_STREAM("[drawModel()]");
        modelMarker.header.frame_id = gap->getFrame();
        modelMarker.header.stamp = ros::Time();
        modelMarker.ns = ns;
        modelMarker.id = id++;
        modelMarker.type = visualization_msgs::Marker::CYLINDER;
        modelMarker.action = visualization_msgs::Marker::ADD;
        
        gap->getLeftGapPt()->getModel()->isolateManipGapDynamics();
        gap->getRightGapPt()->getModel()->isolateManipGapDynamics();

        Eigen::Vector4f leftModelState = gap->getLeftGapPt()->getModel()->getManipGapState();
        Eigen::Vector4f rightModelState = gap->getRightGapPt()->getModel()->getManipGapState();

        // ROS_INFO_STREAM("   leftModelState: " << leftModelState.transpose());
        // ROS_INFO_STREAM("   rightModelState: " << rightModelState.transpose());

        // ROS_INFO_STREAM("   gap->getLeftGapPt()->getModel()->getRobotVel(): " << gap->getLeftGapPt()->getModel()->getRobotVel());
        // ROS_INFO_STREAM("   gap->getRightGapPt()->getModel()->getRobotVel(): " << gap->getRightGapPt()->getModel()->getRobotVel());

        Eigen::Vector2f gapVel(0.0, 0.0);
        if (left)
        {
            modelMarker.pose.position.x = leftModelState[0];
            modelMarker.pose.position.y = leftModelState[1];
            gapVel = leftModelState.tail(2);
        } else
        {
            modelMarker.pose.position.x = rightModelState[0];
            modelMarker.pose.position.y = rightModelState[1];            
            gapVel << rightModelState.tail(2); 
        }
        modelMarker.pose.position.z = 0.01;
        
        modelMarker.pose.orientation.x = 0.0;
        modelMarker.pose.orientation.y = 0.0;
        modelMarker.pose.orientation.z = 0.0;
        modelMarker.pose.orientation.w = 1.0;

        modelMarker.scale.x = 0.2;
        modelMarker.scale.y = 0.2;
        modelMarker.scale.z = 0.000001;

        // modelMarker.color.a = 1.0;
        // modelMarker.color.r = 1.0;
        // modelMarker.color.b = 1.0;
        // modelMarker.lifetime = ros::Duration(0);
    }

    void GapVisualizer::drawManipGapModelVelocities(visualization_msgs::MarkerArray & gapModelMarkerArray,
                                                    const int & gapIdx, Gap * gap, const std::string & ns)
    {
        ROS_INFO_STREAM_NAMED("Visualizer", "[drawManipGapModelVelocities]");
        int id = (int) gapModelMarkerArray.markers.size();
        bool left = true;

        visualization_msgs::Marker leftModelMarker, rightModelMarker;

        drawManipModelVelocity(leftModelMarker, gapIdx, gap, left, id, ns);
        gapModelMarkerArray.markers.push_back(leftModelMarker);

        drawManipModelVelocity(rightModelMarker, gapIdx, gap, !left, id, ns);
        gapModelMarkerArray.markers.push_back(rightModelMarker);
    }


    void GapVisualizer::drawManipModelVelocity(visualization_msgs::Marker & modelMarker, 
                                                const int & gapIdx, Gap * gap, const bool & left, int & id, const std::string & ns) 
    {
        ROS_INFO_STREAM_NAMED("Visualizer", "[drawManipModelVelocity]");

        if (gap->getFrame().empty())
        {
            ROS_WARN_STREAM("[drawModel] Gap frame is empty");
            return;
        }
        
        // std::string fullNamespace = ns;
        // fullNamespace.append("_initial");

        // auto colorIter = colorMap.find(fullNamespace);
        // if (colorIter == colorMap.end()) 
        // {
        //     ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
        //     return;
        // }

        // modelMarker.color = colorIter->second;

        modelMarker.color = visionColors_.at(gapIdx % visionColors_.size());

        // ROS_INFO_STREAM("[drawModel()]");
        modelMarker.header.frame_id = gap->getFrame();
        modelMarker.header.stamp = ros::Time();
        modelMarker.ns = ns;
        modelMarker.id = id++;
        modelMarker.type = visualization_msgs::Marker::ARROW;
        modelMarker.action = visualization_msgs::Marker::ADD;
        
        gap->getLeftGapPt()->getModel()->isolateManipGapDynamics();
        gap->getRightGapPt()->getModel()->isolateManipGapDynamics();

        Eigen::Vector4f leftModelState = gap->getLeftGapPt()->getModel()->getManipGapState();
        Eigen::Vector4f rightModelState = gap->getRightGapPt()->getModel()->getManipGapState();

        ROS_INFO_STREAM_NAMED("Visualizer", "   leftModelState: " << leftModelState.transpose());
        ROS_INFO_STREAM_NAMED("Visualizer", "   rightModelState: " << rightModelState.transpose());

        // ROS_INFO_STREAM("   gap->getLeftGapPt()->getModel()->getRobotVel(): " << gap->getLeftGapPt()->getModel()->getRobotVel());
        // ROS_INFO_STREAM("   gap->getRightGapPt()->getModel()->getRobotVel(): " << gap->getRightGapPt()->getModel()->getRobotVel());

        Eigen::Vector2f gapVel(0.0, 0.0);
        if (left)
        {
            modelMarker.pose.position.x = leftModelState[0];
            modelMarker.pose.position.y = leftModelState[1];
            gapVel = leftModelState.tail(2);
        } else
        {
            modelMarker.pose.position.x = rightModelState[0];
            modelMarker.pose.position.y = rightModelState[1];            
            gapVel << rightModelState.tail(2); 
        }
        modelMarker.pose.position.z = 0.01;

        float gapVelTheta;
        if (gapVel.norm() < std::numeric_limits<float>::epsilon())
        {
            gapVelTheta = 0.0;
        } else
        {
            gapVelTheta = std::atan2(gapVel[1], gapVel[0]);
        }

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, gapVelTheta);
        
        modelMarker.pose.orientation.x = quat.getX();
        modelMarker.pose.orientation.y = quat.getY();
        modelMarker.pose.orientation.z = quat.getZ();
        modelMarker.pose.orientation.w = quat.getW();

        modelMarker.scale.x = gapVel.norm() + 0.000001;
        modelMarker.scale.y = 0.1;
        modelMarker.scale.z = 0.000001;

        // modelMarker.lifetime = ros::Duration(0);
    }
}
