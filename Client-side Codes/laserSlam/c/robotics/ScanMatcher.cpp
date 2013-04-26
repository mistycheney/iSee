#include "ScanMatcher.h"
#include "geometry2D.hpp"
#include "nn.hpp"
#include <algorithm>

using namespace Robotics;

const int    ScanMatcher::NumIter       = 10;
//const double ScanMatcher::Gates[]       = {1.0, 0.3, 0.1};
const double ScanMatcher::Gates[]       = {0.3, 0.1, 0.05};
//const double ScanMatcher::Gates[]       = {2.0, 1.0, 0.5, 0.3};
//const double ScanMatcher::Gates[]       = {1.0, 0.3, 0.1};
//const double ScanMatcher::Gates[]       = {1.0, 0.3, 0.1, 0.05};
//const double ScanMatcher::Gates[]       = {3.0, 2.0, 1.0, 0.5, 0.3};
const int    ScanMatcher::NumGate       = nelems(Gates);
const double ScanMatcher::GatesGlobal[] = {1.0, 0.5, 0.25};
const int    ScanMatcher::NumGateGlobal = nelems(GatesGlobal);

void ScanMatcher::StdIcp(Geom2D::Pose &pose, 
						  const std::vector<Geom2D::Point> &obs, 
						  const std::vector<Geom2D::Point> &ref,
						  bool isKdtree) {
	if (ref.size() < obs.size() / 2)
		return;	
	Geom2D::ICP icp(ref, Gates[0]);
	for (int i = 0; i < NumGate; i++) {						
		Geom2D::Pose delta = icp.align(obs, pose, Gates[i], NumIter, true);		
		if (!std::isnan(delta.phi)) {
			pose = delta;
		} else {
			return;
		}
	}	
}



void ScanMatcher::ComputeRelativePose(Geom2D::Pose &pose, const std::vector<Geom2D::Point> &obs, const std::vector<Geom2D::Point> &ref) {
	pose = Geom2D::compute_relative_pose(obs, ref);
}

void ScanMatcher::TransformToGlobal(Geom2D::Pose &delta, const Geom2D::Pose &pose) {
	Geom2D::Transform2D tr(pose);

	tr.transform_to_global(delta);
}

void ScanMatcher::TransformToGlobalBackward(Geom2D::Pose &delta, const Geom2D::Pose &pose) {
	Geom2D::Pose origin;
	origin.p.x = 0.0;
	origin.p.y = 0.0;
	origin.phi = 0.0;

	TransformToRelative(origin, delta);
	TransformToGlobal(origin, pose);

	delta = origin;
}

void ScanMatcher::TransformToRelative(Geom2D::Pose &delta, const Geom2D::Pose &pose) {
	Geom2D::Transform2D tr(pose);

	tr.transform_to_relative(delta);
}

void ScanMatcher::PointsToSegments(std::vector<std::vector<Geom2D::Point> > &vecSegments, const std::vector<Geom2D::Point> &vecPoints, double gateSeg, int sizeSeg) {
	vecSegments.clear();

	int idxSeg = 1;
	int numSeg = 1;

	while (idxSeg < vecPoints.size()) {
		if (Distance(vecPoints[idxSeg-1], vecPoints[idxSeg]) <= gateSeg 
	      || (idxSeg >= 2 && Distance(vecPoints[idxSeg-2], vecPoints[idxSeg]) <= gateSeg)) {	
			numSeg++;
		} else {
			if (numSeg >= sizeSeg) {
				std::vector<Geom2D::Point> segment;

				for (int i = idxSeg-numSeg; i < idxSeg; i++) {
					segment.push_back(vecPoints[i]);
				}

				vecSegments.push_back(segment);
			}

			numSeg = 1;
		}
		idxSeg++;
	}
	// last segment
	if (numSeg >= sizeSeg) {
		std::vector<Geom2D::Point> segment;

		for (int i = idxSeg-numSeg; i < idxSeg; i++) {
			segment.push_back(vecPoints[i]);
		}

		vecSegments.push_back(segment);
	}
}

void ScanMatcher::TransformToGlobal(Geom2D::Point &point, const Geom2D::Pose &pose) {
	Geom2D::Transform2D tr(pose);

	tr.transform_to_global(point);
}

void ScanMatcher::TransformToGlobal(std::vector<Geom2D::Point> &vecPoints, const Geom2D::Pose &pose) {
	Geom2D::Transform2D tr(pose);

	for (int i = 0; i < vecPoints.size(); i++) {
		tr.transform_to_global(vecPoints[i]);
	}
}

void ScanMatcher::TransformToGlobal(std::vector<std::vector<Geom2D::Point> > &vecSegments, const Geom2D::Pose &pose) {
	Geom2D::Transform2D tr(pose);

	for (int i = 0; i < vecSegments.size(); i++) {
		for (int j = 0; j < vecSegments[i].size(); j++) {
			tr.transform_to_global(vecSegments[i][j]);
		}
	}
}

void ScanMatcher::TransformToRelative(Geom2D::Point &point, const Geom2D::Pose &pose) {
	Geom2D::Transform2D tr(pose);

	tr.transform_to_relative(point);
}

void ScanMatcher::TransformToRelative(std::vector<Geom2D::Point> &vecPoints, const Geom2D::Pose &pose) {
	Geom2D::Transform2D tr(pose);

	for (int i = 0; i < vecPoints.size(); i++) {
		tr.transform_to_relative(vecPoints[i]);
	}
}

void ScanMatcher::TransformToRelative(std::vector<std::vector<Geom2D::Point> > &vecSegments, const Geom2D::Pose &pose) {
	Geom2D::Transform2D tr(pose);

	for (int i = 0; i < vecSegments.size(); i++) {
		for (int j = 0; j < vecSegments[i].size(); j++) {
			tr.transform_to_relative(vecSegments[i][j]);
		}
	}
}

double ScanMatcher::Distance(const Geom2D::Point &p1, const Geom2D::Point &p2) {
    return sqrt(sqr(p1.x-p2.x)+sqr(p1.y-p2.y));
}

void ScanMatcher::Bound(Geom2D::Line &bound, const std::vector<Geom2D::Point> &vecPoints) {
	bound.first.x  = (*std::min_element(vecPoints.begin(), vecPoints.end(), Utility::PointXLess)).x;
	bound.first.y  = (*std::min_element(vecPoints.begin(), vecPoints.end(), Utility::PointYLess)).y;
	bound.second.x = (*std::max_element(vecPoints.begin(), vecPoints.end(), Utility::PointXLess)).x;
	bound.second.y = (*std::max_element(vecPoints.begin(), vecPoints.end(), Utility::PointYLess)).y;
}

void ScanMatcher::Mean(Geom2D::Point &point, const std::vector<Geom2D::Point> &vecPoints) {
	Geom2D::Line bound;

	Bound(bound, vecPoints);

	point.x = (bound.first.x+bound.second.x)/2.0;
	point.y = (bound.first.y+bound.second.y)/2.0;
}

void ScanMatcher::SegmentsToPoints(std::vector<Geom2D::Point> &vecPoints, const std::vector<std::vector<Geom2D::Point> > &vecSegments)
{
	vecPoints.clear();

	for (int i = 0; i < vecSegments.size(); i++) {
		for (int j = 0; j < vecSegments[i].size(); j++) {
			vecPoints.push_back(vecSegments[i][j]);
		}	
	}

}
