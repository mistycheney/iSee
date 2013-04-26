#include "SensorModel.h"
#include "Utility.h"
#include <math.h>
#include <vector>

using namespace Robotics;

const double SensorModel::ProbOccupied         = GridMap::ProbOccupied;
const double SensorModel::ProbFree             = GridMap::ProbFree;

const double SensorModel::ProbOccupiedPrior    = 0.8;
//const double SensorModel::ProbOccupiedPrior    = 0.6;
const double SensorModel::ProbUnknownPrior     = 0.5;
const double SensorModel::ProbFreePrior        = 1.0-ProbOccupiedPrior;
const double SensorModel::LogOddsOccupiedPrior = log(ProbOccupiedPrior/ProbFreePrior);
const double SensorModel::LogOddsUnknownPrior  = log(ProbUnknownPrior/(1.0-ProbUnknownPrior));
const double SensorModel::LogOddsFreePrior     = log(ProbFreePrior/ProbOccupiedPrior);

//const double SensorModel::DistMaxUpdate        = 6.0;
//const double SensorModel::DistMaxUpdate          = 6.0;
const double SensorModel::DistMaxUpdate        = 8.0;
//const double SensorModel::DistMaxUpdate        = 40.0;

SensorModel::SensorModel(int lenScan, double phiBegin, double phiEnd, int distMax, int distMin, double scaleDist) {
	m_lenScan   = lenScan;
	m_phiBegin  = phiBegin;
	m_phiEnd    = phiEnd;
	m_distMax   = distMax;
	m_distMin   = distMin;
	m_scaleDist = scaleDist;
	
	m_phiDelta  = (m_phiEnd-m_phiBegin)/(m_lenScan-1);

	m_aScanCosPhi = new double[m_lenScan];
	m_aScanSinPhi = new double[m_lenScan];
	m_aScanPhi    = new double[m_lenScan];

	for (int i = 0; i < m_lenScan; i++) {
		double phi = m_phiBegin + i*m_phiDelta;
		
		m_aScanCosPhi[i] = cos(phi);
		m_aScanSinPhi[i] = sin(phi);		
		m_aScanPhi[i]    = phi;			// for occupancy grid mapping
	}
	
	
}

SensorModel::~SensorModel() {
	delete[] m_aScanCosPhi;
	delete[] m_aScanSinPhi;
	delete[] m_aScanPhi;
}

void SensorModel::Allocate(int *aScan[]) {
	*aScan = new int[m_lenScan];
}

void SensorModel::Deallocate(int *aScan) {
	delete[] aScan;
}

void SensorModel::CopyScan(int *aScanDest, const int *aScanSrc) {
	memcpy(aScanDest, aScanSrc, m_lenScan*sizeof(int));
}

void SensorModel::ScanToPoint(Geom2D::Point &pt, const int r, const int indx)
{
	if (indx >= 0 && indx < m_lenScan) {
		double dist = r * m_scaleDist;
		
		pt.x = dist * m_aScanCosPhi[indx];		
		pt.y = dist * m_aScanSinPhi[indx];
	}
}

void SensorModel::ScanToPoints(std::vector<Geom2D::Point> &vecPoints, const int *aScan) {
	vecPoints.clear();

	for (int i = 0; i < m_lenScan; i++) {
		if (aScan[i] >= m_distMin && aScan[i] <= m_distMax) {
			Geom2D::Point point;
			double dist = aScan[i]*m_scaleDist;

			point.x = dist*m_aScanCosPhi[i];
			point.y = dist*m_aScanSinPhi[i];
			
			point.scanId = i;

			vecPoints.push_back(point);
		}
	}
}

void SensorModel::Setup(int *lenScan, double *phiBegin, double *phiEnd, 
						int *distMax, int *distMin, double *scaleDist, 
						TypeSensor typeSensor) {
	switch (typeSensor) {
		case SickLms100:
			*lenScan   = 541;
			*phiBegin  = dtor(-135);
			*phiEnd    = dtor(+135);
			//*distMax   = 44000;
			//*distMax   = 4094;
			//*distMax   = 6000;
			*distMax	=  25000;
			//*distMax   = 8000;

			*distMin   = 20;
			*scaleDist = 0.001;
			
			break;
			
		case SickLms291:
			*lenScan   = 361;
			*phiBegin  = dtor(-90);
			*phiEnd    = dtor(+90);
			//*distMax   = 81910;
			*distMax   = 80000;
			*distMin   = 20;
			*scaleDist = 0.001;
						
			break;
			
		case HokuyoUrg04Lx:
		default:
			*lenScan   = 682;			  // length of a laser scan
			*phiBegin  = dtor(-119.5334); // first measurable angle
			*phiEnd    = dtor(+119.8850); // last measurable angle
			*distMax   = 5500; 			  // maximum measurable distance, 4094
			*distMin   = 20;			  // minimum measurable distance
			*scaleDist = 0.001;			  // from millimter to meter

	}
}


void SensorModel::UpdateGridMap(GridMap *pGridMap, const Geom2D::Pose &pose, const int *aScan, double alpha, double beta) {
	// Remark: Use meter.
	double resolution = pGridMap->GetResolution();											// meter
	int    nGrids     = (int)ceil(Min(DistMaxUpdate, m_distMax*m_scaleDist)/resolution);	// number of grids

	for (int j = -nGrids; j <= nGrids; j++) {		// y-axis
		for (int i = -nGrids; i <= nGrids; i++) {	// x-axis
			double x = i*resolution;				// meter w.r.t. robot frame
			double y = j*resolution;				// meter w.r.t. robot frame

			// TODO: Implement two local grid maps to store the distance and angle to the robot frame.
			double rho = sqrt(sqr(x)+sqr(y));				// location in meter w.r.t. robot frame
			double phi = pi_to_pi(atan2(y, x)-pose.phi);	// orientation in radian w.r.t. robot frame
			
			int    k = (int)round((phi-m_phiBegin)/m_phiDelta);
			
			if (k >= 0 && k < m_lenScan) {
				double dist		= aScan[k]*m_scaleDist;		// meter
				double logodds	= pGridMap->GetLogOdds(pose.p.x+x, pose.p.y+y);	
				
				if (dist > m_distMax * m_scaleDist) {
					logodds += LogOddsFreePrior;
				} else if (dist < m_distMin * m_scaleDist) {

				} else if (rho > Min(m_distMax*m_scaleDist, dist+alpha/2.0) || fabs(phi-m_aScanPhi[k]) > beta/2.0) {
					logodds += LogOddsUnknownPrior;
				} else if (aScan[k] <= m_distMax && aScan[k] >= m_distMin && fabs(rho-dist) < alpha/2.0) {
					logodds += LogOddsOccupiedPrior;
				} else if (aScan[k] <= m_distMax && rho < dist) {
					logodds += LogOddsFreePrior;
				} else {
					logodds += LogOddsUnknownPrior;
				}

				pGridMap->SetLogOdds(pose.p.x+x, pose.p.y+y, logodds);
			}			
		}
	}	
}


void SensorModel::RemoveNonExistPoints(int *aScan, const std::vector<Geom2D::Point> &pts)
{
	int *Indicate = new int[m_lenScan];

	for (int i = 0; i < m_lenScan; i++) {
		Indicate[i] = 0;
	}
	
	for (int i = 0; i < pts.size(); i++) {
		Indicate[pts[i].scanId] = 1;
	}

	for (int i = 0; i < m_lenScan; i++) {
		if (Indicate[i] == 1) {
			continue;
		}

		aScan[i] = 0;
	}

	delete Indicate;
}


