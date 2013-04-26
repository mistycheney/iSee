#include <slamWrapper.h>

using namespace Robotics;

SlamWrapper::SlamWrapper()
{
	pts0.clear();
	pts1.clear();
	gPts0.clear();
	gPts1.clear();
	const Geom2D::Pose poseZero;
	delta = pose = posePrev = poseZero;	
}
SlamWrapper::~SlamWrapper()
{
	if(m_pSensor)
		delete m_pSensor;
	if(m_aScan)
		delete[] m_aScan;
	if(m_pGridMap)
		delete m_pGridMap;
}

const GridMap* SlamWrapper::GetMap() const
{
	return m_pGridMap;
}
int SlamWrapper::GetScanLen() const
{
	return m_lenScan;
}
Geom2D::Pose SlamWrapper::GetPose() const
{
	return pose;
}
void SlamWrapper::GetPose(int &i, int &j) const
{
	m_pGridMap->GetMap2Array(pose.p.x,pose.p.y,i,j);	
}
const std::vector<Geom2D::Point> SlamWrapper::GetScanPts() const
{
	return gPts1;
}
void SlamWrapper::InitializeMap(double mapRes, double mapWidth)
{
	m_pGridMap = new Robotics::GridMap(0, mapRes, mapWidth);  
	m_pGridMap->Reset();	
}
void SlamWrapper::InitializeSensorModel(TypeSensor typeSensor)
{
	// sensor
	Robotics::SensorModel::Setup(&m_lenScan, &m_phiBegin, &m_phiEnd, &m_distMax,
								 &m_distMin, &	m_scaleDist, typeSensor);
	m_pSensor = new Robotics::SensorModel(m_lenScan, m_phiBegin, m_phiEnd, m_distMax,
											  m_distMin, m_scaleDist);
	m_pSensor->Allocate(&m_aScan);	
}

void SlamWrapper::SetScan(int* aScan)
{
	memcpy(aScan, m_aScan, m_lenScan*sizeof(int));
}
void SlamWrapper::Slam()
{	
	if(pts0.empty()) { // first scan
		m_pSensor->ScanToPoints(pts0, m_aScan);		
		gPts0 = pts0;
	}
	else { // loop
		m_pSensor->ScanToPoints(pts1, m_aScan);	
	// <Step> Local Scan Matching
		delta.p.x = delta.p.y = delta.phi = 0.0;
		
		ScanMatcher::StdIcp(delta, pts1, pts0);		
		pose = delta;
		ScanMatcher::TransformToGlobal(pose, posePrev);

	// <Step> Scan Matching to Map
		ScanMatcher::StdIcp(pose, pts1, m_pGridMap->GetOccupied());	

	// <Step> Transform Current Scan to Global
		gPts1 = pts1;
		ScanMatcher::TransformToGlobal(gPts1, pose);		

	// <Step> Segmentation		
		ScanMatcher::PointsToSegments(gSegs, gPts1, 0.3, 5);	
		
		// Discard non-segmented points, assumed noise
		ScanMatcher::SegmentsToPoints(gSegPts, gSegs);
		m_pSensor->RemoveNonExistPoints(m_aScan, gSegPts);		
	
	// <Step> Occupancy Grid Mapping			
		m_pSensor->UpdateGridMap(m_pGridMap, pose, m_aScan, 0.15);		
		m_pGridMap->PruneOccupiedPts(pose, 15);		

	// <Step> Save current state
		posePrev = pose;
		pts0 = pts1;
		gPts0 = gPts1;
	}
}