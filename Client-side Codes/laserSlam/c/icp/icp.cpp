/* Iterated closest point (ICP) algorithm.
 * Tim Bailey 2004.
 */
#include "icp.hpp"
#include <cassert>
#include <time.h>
using namespace std;

namespace Geom2D {

ICP::ICP(const vector<Point>& reference, const double& gate) : 
ref(reference), nn(new SweepSearch(reference, gate))
{
}
ICP::~ICP()
{
}
void ICP::setNNGate(const double& gate)
{
	nn->setLimit(gate);
}
Pose ICP::align(const vector<Point>& obs, const Pose& init, double gate, int nits, bool interp,double convergence) 
{
	Pose pse = init,prevPse; 
	double gate_sqr = sqr(gate); 
	int size_obs = obs.size();
	const int numNN = 2;	
	vector<int> index(numNN); // used if interp == true		

	// set the gating
	setNNGate(gate);

	while (nits-- > 0) {
		Transform2D tr(pse);
		a.clear();
		b.clear();

		// For each point in obs, find its NN in ref
		for (int i = 0; i < size_obs; ++i) { 
			Point p = obs[i];
			tr.transform_to_global(p); // transform obs[i] to estimated ref coord-frame

			Point q;
			if (interp == false) { // simple ICP
				int idx = nn->query(p);
				if (idx == SweepSearch::NOT_FOUND)
					continue;

				q = ref[idx];
			} 
			else { // ICP with interpolation between 2 nearest points in ref
				(void) nn->query(p, index);
				assert(index.size() == 2);
				if (index[1] == SweepSearch::NOT_FOUND)
					continue;

				Line lne;
				lne.first  = ref[index[0]];
				lne.second = ref[index[1]];
				intersection_line_point(q, lne, p);
			}

			if (dist_sqr(p,q) < gate_sqr) { // check if NN is close enough 
				a.push_back(obs[i]);
				b.push_back(q);
			}
		}

		//assert(a.size() > 2); // TODO: replace this assert with a proper status flag
		pse = compute_relative_pose(a, b); // new iteration result
		// check convergence
		if (abs(pse.p.x-prevPse.p.x)<convergence &&
			abs(pse.p.y-prevPse.p.y)<convergence &&
			abs(pse.phi-prevPse.phi)<convergence) {
			break;
		}
		else {			
			prevPse = pse;		
		}
	}
	//printf("%d \n",nits);
	
	return pse;
}

} // namespace Geom2D
