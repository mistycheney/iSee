#ifndef __ROBOTICS_SCANMATCHER_H__
#define __ROBOTICS_SCANMATCHER_H__

#include "icp.hpp"
#include "Utility.h"
#include "SensorModel.h"

namespace Robotics {
	
	class ScanMatcher {		
		public:
			
			// Scan matchers
			
			static void StdIcp(Geom2D::Pose &pose, const std::vector<Geom2D::Point> &obs, const std::vector<Geom2D::Point> &ref, bool isKdtree=false);									
			
			// Least square minimizers
			
			static void ComputeRelativePose(Geom2D::Pose &pose, const std::vector<Geom2D::Point> &obs, const std::vector<Geom2D::Point> &ref);			
			
			// Utility functions for geometric transformation

			static void TransformToGlobal(Geom2D::Pose &delta, const Geom2D::Pose &pose);
			static void TransformToGlobalBackward(Geom2D::Pose &delta, const Geom2D::Pose &pose);
			static void TransformToRelative(Geom2D::Pose &delta, const Geom2D::Pose &pose);			
			static void TransformToGlobal(Geom2D::Point &point, const Geom2D::Pose &pose);
			static void TransformToGlobal(std::vector<Geom2D::Point> &vecPoints, const Geom2D::Pose &pose);
			static void TransformToGlobal(std::vector<std::vector<Geom2D::Point> > &vecSegments, const Geom2D::Pose &pose);
			static void TransformToRelative(Geom2D::Point &point, const Geom2D::Pose &pose);
			static void TransformToRelative(std::vector<Geom2D::Point> &vecPoints, const Geom2D::Pose &pose);
			static void TransformToRelative(std::vector<std::vector<Geom2D::Point> > &vecSegments, const Geom2D::Pose &pose);

			// cluster the segment counter clockwise with given distance thresh and minmumSegSize
			static void PointsToSegments(std::vector<std::vector<Geom2D::Point> > &vecSegments, 
										 const std::vector<Geom2D::Point> &vecPoints, 
										 double gateSeg=0.3, int sizeSeg=10);
			static void SegmentsToPoints(std::vector<Geom2D::Point> &vecPoints, const std::vector<std::vector<Geom2D::Point> > &vecSegments);


			// Axuiliary functions

			static double Distance(const Geom2D::Point &p1, const Geom2D::Point &p2);
			static void   Bound(Geom2D::Line &bound, const std::vector<Geom2D::Point> &vecPoints);
			static void   Mean(Geom2D::Point &point, const std::vector<Geom2D::Point> &vecPoints);
			
		protected:			
			static const int    NumIter;
			static const double Gates[];
			static const int    NumGate;
			static const double GatesGlobal[];
			static const int    NumGateGlobal;				
	};
	
}

#endif
