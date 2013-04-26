#include "GridMap.h"
#include "ScanMatcher.h"
#include <algorithm>
#include <math.h>
#include <stdio.h>

// #include <windows.h>	// OS-dependent implementation for virtual memory allocation
#include <fstream>


using namespace Robotics;

const double GridMap::ProbOccupied    = 0.5;
const double GridMap::ProbFree        = 1.0-ProbOccupied;
const double GridMap::LogOddsOccupied = log(1.0/(1.0-ProbOccupied)-1.0);
const double GridMap::LogOddsFree     = -LogOddsOccupied;


GridMap::GridMap(const char *szFile, double distResolution, double distWidth) {	
	m_pOccupancyGrid = NULL;
	
	m_distResolution = distResolution;
	m_distWidth      = distWidth;
	m_distHeight     = distWidth;
	m_pointOriginX   = 0.0;
	m_pointOriginY   = 0.0;
	
	m_numRows        = 1 + (unsigned int)ceil(m_distHeight/m_distResolution);
	m_numCols        = 1 + (unsigned int)ceil(m_distWidth/m_distResolution);
	
	Reset();
}

GridMap::~GridMap() {
	Deallocate(m_pOccupancyGrid);
}

double GridMap::GetProb(double x, double y) const {
	return 1.0-1.0/(1.0+exp(GetLogOdds(x, y)));
}

double GridMap::GetProb(int i, int j) const {
	return 1.0-1.0/(1.0+exp(GetLogOdds(i,j)));
}

void GridMap::SetProb(double x, double y, double p) {
	SetLogOdds(x, y, log(1.0/(1.0-p)-1.0));
}

double GridMap::GetLogOdds(double x, double y) const {
	int i, j;
	
	MapToArray(&i, &j, x, y);
	
	return GetLogOdds(i, j);
}

void GridMap::SetLogOdds(double x, double y, double l) {
	int i, j;
	
	MapToArray(&i, &j, x, y);
	
	SetLogOdds(i, j, l);
}

bool GridMap::IsOccupied(double x, double y) const {
	return GetLogOdds(x, y) > LogOddsOccupied;
}

bool GridMap::IsUnknown(double x, double y) const {
	return !IsOccupied(x, y) && !IsFree(x, y);
}

bool GridMap::IsFree(double x, double y) const {
	return GetLogOdds(x, y) < LogOddsFree;
}

double GridMap::GetResolution() const {
	return m_distResolution;
}

void GridMap::GetBound(Geom2D::Line &bound) const {
	if(!m_vecOccupied.empty())
		ScanMatcher::Bound(bound, m_vecOccupied);	
	else {
		bound.first.x = bound.first.y = 0;
		bound.second = bound.first;
	}
}

void GridMap::GetBound(int& minX,int& minY, int& maxX, int& maxY) const
{
	Geom2D::Line bound;
	GetBound(bound);
	MapToArray(&minX,&minY,bound.first.x,bound.first.y);
	MapToArray(&maxX,&maxY,bound.second.x,bound.second.y);
}

void GridMap::GetMap2Array(double x, double y,int &i,int& j) const
{
	MapToArray(&i,&j,x,y);
}
const std::vector<Geom2D::Point>& GridMap::GetOccupied() const
{
	
	return m_vecOccupied;
}

void GridMap::PruneOccupiedPts(const Geom2D::Pose &pose, const double& maxDis)
{
	// boundary check, if the robot enters
	// small overlap region, perform pruning.
	Geom2D::Line bound;
	GetBound(bound);
	double width  = abs(bound.second.x - bound.first.x);
	double height = abs(bound.second.y - bound.first.y);
	double overLapRatio = 0.3;
	if( (pose.p.x-bound.first.x)/width < overLapRatio ||
		-(pose.p.x-bound.second.x)/width < overLapRatio ||
		(pose.p.y-bound.first.y)/height < overLapRatio ||
		-(pose.p.y-bound.second.y)/height < overLapRatio) {
			overLapRatio = overLapRatio;	
	} else {
		//system("pause");
		return;
	}
	//
	double maxDisSqr = maxDis*maxDis;
	std::vector<Geom2D::Point> cp_vecOccupied(m_vecOccupied);
	m_vecOccupied.clear();
	m_vecOccupied.reserve(cp_vecOccupied.size());
	std::vector<Geom2D::Point>::iterator i = cp_vecOccupied.begin();

	for(;i!=cp_vecOccupied.end();i++) {
		if(Geom2D::dist_sqr(pose.p,*i) < maxDisSqr) {
			m_vecOccupied.push_back(*i);
		}	
	}

}
void GridMap::Reset() {
	Deallocate(m_pOccupancyGrid);
	
	Allocate(&m_pOccupancyGrid);	

	m_vecOccupied.clear();
	m_numOccupied = 0;

}

void GridMap::Allocate(float ***pOccupancyGrid) const {
    *pOccupancyGrid = (float **)
	*pOccupancyGrid = (float **)VirtualAlloc(NULL, m_numRows*sizeof(float *)+m_numRows*m_numCols*sizeof(float), MEM_RESERVE|MEM_COMMIT|MEM_TOP_DOWN, PAGE_READWRITE);

	int    i;
	float *pData;

	for (i = 0, pData = (float *)(*pOccupancyGrid+m_numRows); i < m_numRows; i++, pData+=m_numCols) {
		(*pOccupancyGrid)[i] = pData;
	}
}

void GridMap::Deallocate(float **pOccupancyGrid) const {
	VirtualFree(pOccupancyGrid, 0, MEM_RELEASE);
}

bool GridMap::IsValid(int i, int j) const {
	return i >= 0 && i < m_numRows && j >= 0 && j < m_numCols;
}

bool GridMap::IsValid(double x, double y) const {
	int i, j;
	
	MapToArray(&i, &j, x, y);
	
	return IsValid(i, j);
}

bool  GridMap::MapSizeCheck(int rows, int cols) const {
	if (rows == m_numRows && cols == m_numCols )
		return true;
	else
		return false;
}
void GridMap::MapToArray(int *i, int *j, double x, double y) const {
	*i = (int)round((y - m_pointOriginY + m_distHeight/2.0)/m_distResolution);
	*j = (int)round((x - m_pointOriginX + m_distWidth/2.0)/m_distResolution);
}

void GridMap::ArrayToMap(double *x, double *y, int i, int j) const {
	*x = j*m_distResolution + m_pointOriginX - m_distWidth/2.0;
	*y = i*m_distResolution + m_pointOriginY - m_distHeight/2.0;
}

double GridMap::GetLogOdds(int i, int j) const {	
	if (IsValid(i, j)) {
		return (double)m_pOccupancyGrid[i][j];
	} else {
		return 0.0;
	}
}

void GridMap::SetLogOdds(int i, int j, double l) {
	if (IsValid(i, j)) {	
		double x, y;
		Geom2D::Point point;
	
		ArrayToMap(&x, &y, i, j);
	
		point.x = x;
		point.y = y;
		
		float value = m_pOccupancyGrid[i][j];
		
		if (value <= LogOddsOccupied && l > LogOddsOccupied) {
			// becomes occupied		
			m_vecOccupied.push_back(point);
			m_numOccupied++;		
		} else if (value > LogOddsOccupied && l <= LogOddsOccupied) {
			// becomes not occupied
			std::vector<Geom2D::Point>::iterator iter = std::find(m_vecOccupied.begin(), m_vecOccupied.end(), point);
	
			if (iter != m_vecOccupied.end()) {
				m_vecOccupied.erase(iter);
				m_numOccupied--;
			}
		}
		
		m_pOccupancyGrid[i][j] = l;
	}
}

