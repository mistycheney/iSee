

#ifndef __ROBOTICS_SENSORMODEL_H__
#define __ROBOTICS_SENSORMODEL_H__

#include "GridMap.h"
#include <float.h>

namespace Robotics {

	typedef enum {
		HokuyoUrg04Lx,
		SickLms100,
		SickLms291
	} TypeSensor;

	class SensorModel {
		public:
			SensorModel(int lenScan, double phiBegin, double phiEnd, int distMax, int distMin, double scaleDist=0.001);
			~SensorModel();
			
			void Allocate(int *aScan[]);
			void Deallocate(int *aScan);

			void CopyScan(int *aScanDest, const int *aScanSrc);
			void ScanToPoints(std::vector<Geom2D::Point> &vecPoints, const int *aScan);

			void ScanToPoint(Geom2D::Point &pt, const int r, const int indx);

			
			// Sensor modeling methods
			
			void UpdateGridMap(GridMap *pGridMap, const Geom2D::Pose &pose, 
							   const int *aScan, double alpha=0.5, 
							   double beta=DBL_MAX);					
			void RemoveNonExistPoints(int *aScan, const std::vector<Geom2D::Point> &pts);
			
			// Visualization constants
			
			const static double ProbOccupied;
			const static double ProbFree;		
			
			// Grid mapping constants
		
			const static double ProbOccupiedPrior;
			const static double ProbUnknownPrior;
			const static double ProbFreePrior;
			const static double LogOddsOccupiedPrior;
			const static double LogOddsUnknownPrior;
			const static double LogOddsFreePrior;
			
			const static double DistMaxUpdate;		// maximum distance to update on grid mapping
				
			// Setup
			
			static void Setup(int *lenScan, double *phiBegin, double *phiEnd, 
							  int *distMax, int *distMin, double *scaleDist, 
							  TypeSensor typeSensor=HokuyoUrg04Lx);
		
		protected:
			int     m_lenScan;
			double  m_phiBegin;
			double  m_phiEnd;
			int     m_distMax;
			int     m_distMin;
			double  m_scaleDist;

			double  m_phiDelta;
			double *m_aScanCosPhi;
			double *m_aScanSinPhi;
			double *m_aScanPhi;	

	};
	



	
}

#endif
