#include <cfloat>
#include <iostream>
// slam wrapper
#include <slamWrapper.h>

#define _MEX_VERSION_

Robotics::SlamWrapper   *g_slamWrapper = NULL;

#ifdef _MEX_VERSION_
// matlab io
#include "mex.h"
void releaseMemory()
{
	if(g_slamWrapper) {
			delete g_slamWrapper;
			g_slamWrapper = NULL;
		}
}
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	if(nrhs<1) {
		mexErrMsgTxt("command required: loadSparseMatrix");
	}
	// read command
	char command[128];
	int rowLen, colLen;
	mxGetString(prhs[0],command,128);

	// process command	
	if (!strcmp(command,"Initialization")) {		
		// check arguments
		if(nrhs<4) {
			mexErrMsgTxt("paramters: typeSensor mapRes mapHalfWidth");
			return;
		}
		// load data and parameter		
		int typeSensor       = (int)((mxGetPr(prhs[1]))[0]);
		double mapRes        = (mxGetPr(prhs[2]))[0];
		double mapHalfWidth  = (mxGetPr(prhs[3]))[0];
		// initialize
		g_slamWrapper = new Robotics::SlamWrapper();
		g_slamWrapper->InitializeSensorModel((Robotics::TypeSensor)typeSensor);
		g_slamWrapper->InitializeMap(mapRes,mapHalfWidth);	
		
	}
	else if (!strcmp(command,"Slam")) {	
		// check arguments
		if(nrhs<2) {
			mexErrMsgTxt("paramters: scan");
			return;
		}
		if(!g_slamWrapper) {
			mexErrMsgTxt("Initialize the slam objct first!. Use command 'Initialization' ");
			return;
		}
		// load data and parameter		
		int scanLength       = mxGetN(prhs[1]);
		double* data         = mxGetPr(prhs[1]);
		// sanity check
		if(!g_slamWrapper->GetScanLen()==scanLength) {
			printf("%d %d\n",scanLength,g_slamWrapper->GetScanLen());
			releaseMemory();
			mexErrMsgTxt("length size error");
			return;
		}

		for(int i=0;i<scanLength;i++) {			
			g_slamWrapper->m_aScan[i] = (int)data[i];
		}		
		g_slamWrapper->Slam();
	}
	else if (!strcmp(command,"getMapAndPose")) {
		if(nrhs<2) {
			mexErrMsgTxt("paramters: map ");
			return;
		}
		// direct write on the input map	
		int rows       = mxGetM(prhs[1]);
		int cols        = mxGetN(prhs[1]);
		double* map        = mxGetPr(prhs[1]);
		// sanity check
		if(!g_slamWrapper->GetMap()->MapSizeCheck(rows,cols)) {
			releaseMemory();
			mexErrMsgTxt("Map size error");
			return;
		}
		// get bound
		int maxX,maxY,minX,minY;
		g_slamWrapper->GetMap()->GetBound(minX,minY,maxX,maxY);
		for (int j = minY; j <cols && j<maxY; j++)
		{
			for(int i=minX;i<rows && i<maxX;i++){
				map[(j*rows)+i] = g_slamWrapper->GetMap()->GetProb(i,j);								
			}			
		}		   
		
		// return the pose

		//// pose
		plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
		double* poseOut = mxGetPr(plhs[0]);
		poseOut[0] = g_slamWrapper->GetPose().p.x;		
		poseOut[1] = g_slamWrapper->GetPose().p.y;
		poseOut[2] = g_slamWrapper->GetPose().phi;
			
		plhs[1] = mxCreateDoubleMatrix(2, 1, mxREAL);
		double* poseOut2 = mxGetPr(plhs[1]);
		int pose_i,pose_j;
		g_slamWrapper->GetPose(pose_i,pose_j);
		poseOut2[0] = (double)pose_i;
		poseOut2[1] = (double)pose_j;
		////Nkt
		//plhs[2] = mxCreateDoubleMatrix(K, V, mxREAL);
		//double* NktOut = mxGetPr(plhs[2]);
		//for (int j = 0; j <V; j++)
		//{
		//	for(int i=0;i<K;i++){
		//		NktOut[(j*K)+i] = Ntk[j][i];				
		//		//mexPrintf("%f ",Nmk[i][j]);
		//	}
		//	//mexPrintf("\n");
		//}		   
		////mexPrintf("\n");
	}
	else if (!strcmp(command,"releaseMemory")) {	
		releaseMemory();
	}
	else {
		mexPrintf("Unknown command: %s\n",command);
	}
}

int main()
{	
	return 1;
}
#else
// simulation
#include <Simulator.h>
Robotics::Simulator		*m_pSimulator;
// Visualization
#include <CvPlotter.h>
Plotter::CvPlotter      *m_pCvPlotter;

void initialize()
{
	double mapRes   = 0.05; //meter
	double mapWidth = 150;  // meter
	// data reader
	m_pSimulator = new Robotics::Simulator();
	m_pSimulator->OpenLinsm("F:/data/stereo/real/20100528/laserData.txt", NULL, 0 );
	// plotter
	m_pCvPlotter = new Plotter::CvPlotter("Laser SLAM", 500, "pics");
	m_pCvPlotter->SetParameters(150, 0, 0);	

	g_slamWrapper = new Robotics::SlamWrapper();
	g_slamWrapper->InitializeSensorModel(m_pSimulator->GetTypeSensor());
	g_slamWrapper->InitializeMap(mapRes,mapWidth);	

}
int main()
{
	// initialization
	initialize();

	system("pause");
	while (m_pSimulator->NextLinsm()) {		

	// <Step> Data Reading		
		m_pSimulator->GetScan(g_slamWrapper->m_aScan);		
	// <Step> Slam
		g_slamWrapper->Slam();
	
	// <Step> Plot Result		
		// Plot Map
		m_pCvPlotter->PlotPts(g_slamWrapper->GetMap()->GetOccupied(), 255, 255, 255, true);

		Geom2D::Line bound;

		g_slamWrapper->GetMap()->GetBound(bound);
		double resolution = g_slamWrapper->GetMap()->GetResolution();

		for (int i = bound.first.x / resolution; i < bound.second.x / resolution; i++) {
			for (int j = bound.first.y / resolution; j < bound.second.y / resolution; j++) {
				Geom2D::Point pt;
				pt.x = i * resolution;
				pt.y = j * resolution;

				double prob = g_slamWrapper->GetMap()->GetProb(i * resolution, j * resolution);

				m_pCvPlotter->PlotPt(pt, prob * 255, prob * 255, prob * 255);
			}
		}				
		//// Plot current scan
		m_pCvPlotter->PlotPts(g_slamWrapper->GetScanPts(), 255, 0, 0, false, true);
		//// Plot Robot
		//m_pCvPlotter->PlotRobot(pose, 255, 0, 255);	

		m_pCvPlotter->Show(true, 1, false);			
	}		
	delete g_slamWrapper;
	return 1;
}
#endif
