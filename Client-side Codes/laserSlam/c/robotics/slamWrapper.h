#ifndef _SLAM_WRAPPER_
#define _SLAM_WRAPPER_
// Robtics library
#include <Utility.h>
#include <ScanMatcher.h>
#include <GridMap.h>
namespace Robotics {
class SlamWrapper
{
public:
	SlamWrapper();
	~SlamWrapper();

	// initlize
	void InitializeSensorModel(TypeSensor typeSensor);
	void InitializeMap(double mapRes, double mapWidth);	
	// perform slam
	void Slam();
	// IO
	const std::vector<Geom2D::Point> GetScanPts() const;
	const		 GridMap* GetMap() const;	
	void         SetScan(int* scan);
	int          GetScanLen() const;
	Geom2D::Pose GetPose() const;
	void         GetPose(int &i,int&j)const;
	// scan
	int    *m_aScan;

private:
	Geom2D::Pose  delta, pose, posePrev;
	std::vector<Geom2D::Point> pts0, pts1, gPts0, gPts1;
	std::vector<Geom2D::Point> gSegPts;
	std::vector<std::vector<Geom2D::Point> > gSegs;

	// Sensor model	
	int     m_lenScan;
	double  m_phiBegin;
	double  m_phiEnd;
	int     m_distMax;
	int     m_distMin;
	double  m_scaleDist;

	// modules
	Robotics::SensorModel	*m_pSensor;
	Robotics::GridMap		*m_pGridMap;	
};
}
#endif