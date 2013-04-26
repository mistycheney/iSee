/**
 * Occupancy Grid Wrapper
 *
 */

#ifndef __ROBOTICS_GRIDMAP_H__
#define __ROBOTICS_GRIDMAP_H__


#include "geometry2D.hpp"
#include <vector>

namespace Robotics {

	class GridMap {
		public:
			GridMap(const char *szFile=NULL, double distResolution=0.1, double distWidth=250.0);
			~GridMap();
			
			void   Reset();	
			void   PruneOccupiedPts(const Geom2D::Pose &pose, const double& maxDis);
			double GetProb(double x, double y) const;
			double GetProb(int i, int j) const;
			void   SetProb(double x, double y, double p);
			double GetLogOdds(double x, double y) const;
			void   SetLogOdds(double x, double y, double l);
			
			bool   IsOccupied(double x, double y) const;
			bool   IsUnknown(double x, double y) const;
			bool   IsFree(double x, double y) const;			
			bool   MapSizeCheck(int rows, int cols) const;

			double GetResolution() const;				
			void   GetBound(Geom2D::Line &bound) const;	// must be used after calling GetOccupied
			void   GetBound(int& minX,int& minY, int& maxX, int& maxY) const;
			void   GetMap2Array(double x,double y,int& i,int& j) const;
				
			const std::vector<Geom2D::Point>& GetOccupied() const;
			
			const static double ProbOccupied;
			const static double ProbFree;
			const static double LogOddsOccupied;
			const static double LogOddsFree;
		
		private:				

			float      **m_pOccupancyGrid;
			
			double       m_distResolution;
			double       m_distWidth;
			double       m_distHeight;
			double       m_pointOriginX;
			double       m_pointOriginY;
			
			unsigned int m_numRows;
			unsigned int m_numCols;	
					
			unsigned int m_numOccupied;
			std::vector<Geom2D::Point> m_vecOccupied;
			
			inline void Allocate(float ***pOccupancyGrid) const;	// OS-dependent
			inline void Deallocate(float **pOccupancyGrid) const;	// OS-dependent
			
			inline bool IsValid(int i, int j) const;
			inline bool IsValid(double x, double y) const;
			
			inline void MapToArray(int *i, int *j, double x, double y) const;			
			inline void ArrayToMap(double *x, double *y, int i, int j) const;
			
			inline double GetLogOdds(int i, int j) const;
			inline void   SetLogOdds(int i, int j, double l);					
	};
	
}

#endif
