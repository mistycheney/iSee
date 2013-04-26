
#ifndef __UTILITY_H__
#define __UTILITY_H__

#include "geometry2D.hpp"
#include <vector>
#include <float.h>
#include <math.h>

// #ifndef isnan(x)
// #define isnan(x) std::isnan(x)
// #endif

#ifndef isinf(x)
#define isinf(x) (std::isnan((x)/(x))&&((x)==(x)))
#endif

#define _USE_MATH_DEFINES
#include <cmath>

#include <time.h>
namespace Robotics {
#define PI          (M_PI)
#define SQRT2       (M_SQRT2)
#define sign(x)     (((x)>0)?(1):(((x)<0)?(-1):(0)))					// extracts sign
#define sqr(x)      ((x)*(x))											// calculates square
#define dtor(d)     ((d)*PI/180.0)										// transforms angle from degree to angle
#define rtod(r)     ((r)*180.0/PI)										// transforms angle from radian to degree
#define pi_to_pi(r) (((r)>PI)?((r)-2*PI):(((r)<-PI)?((r)+2*PI):(r)))	// normalizes angle to [-PI, PI]
#define Min(a,b)    (((a)<=(b))?(a):(b))   // extracts minimal element
#define Max(a,b)    (((a)>=(b))?(a):(b))
#define round(x)    (floor((x)+0.5))									// performs mathematic round
#define nelems(x)   ((int)(sizeof(x)/sizeof(*(x))))						// extracts number of array elements
}

inline static int numcomb(int n, int k) {
	double num = 1.0;
	
	for (int i = 0; i < k; i++) {
		num *= (double)(n-i)/(double)(k-i);
	}
	
	return (int)round(num);
}

namespace Utility {
	const double chi2cdfTable[41] = {
0,	0.248170365954151,	0.345279153981423,	0.416117579229635,	0.472910743134462,0.520499877813047,
0.561421973919000,	0.597216305753525,	0.628906630477303,	0.657218288852089,	0.682689492137086,
0.705733895695037,	0.726678321707702,	0.745786776396036,	0.763276429362143,	0.779328638080153,
0.794096789267932,	0.807712022888480,	0.820287505121000,	0.831921680965030,	0.842700792949715,
0.852700861377324,	0.861989262431340,	0.870626001163702,	0.878664749641518,	0.886153701993342,
0.893136285006620,	0.899651753537709,	0.905735693158789,	0.911420447420223,	0.916735483336446,
0.921707705853588,	0.926361729879696,	0.930720116778797,	0.934803580921870,	0.938631170860598,
0.942220428876403,	0.945587532008399,	0.948747417142630,	0.951713892323318,	0.954499736103642
									};	

	inline bool PointXLess(Geom2D::Point p1, Geom2D::Point p2) {
		return p1.x < p2.x;
	}
	
	inline bool PointYLess(Geom2D::Point p1, Geom2D::Point p2) {
		return p1.y < p2.y;
	}
	
	class Logs {
		public:			
			// Remark: log(A+B) = log(A) + log(1+exp(log(B)-log(A)))
			inline static double Add(double logA, double logB) {
				if (CloseToZero(logB) && LessLess(logB, logA)) {
					return logA;
				} else if (CloseToZero(logA) && LessLess(logA, logB)) {
					return logB;
				} else {
					return logA + log(1+exp(logB-logA));
				}
			}
			
			// Remark: log(A*B) = log(A) + log(B)
			inline static double Multiply(double logA, double logB) {
				return logA + logB;
			}
			
			// Remark: log(A/B) = log(A) - log(B)
			inline static double Divide(double logA, double logB) {
				return logA - logB;
			}
			
			inline static double CloseToZero(double logA) {
				return logA < log(DBL_MIN);
			}
			
			inline static bool LessLess(double logA, double logB) {
				return logA - logB < -1e2;
			}
	};
	class Random {		
		public:
			// Initialize the random number generator.
			inline Random() {
				srand((unsigned int)(time(NULL)));
			}

			// Sample from open interval (0, 1) uniformly.
			inline double Uniform() {
				return (double)rand()/(double)RAND_MAX;
			}
			
			// Sample from open interval (0, n) uniformly.
			inline double Uniform(double n) {
				return Uniform()*n;
			}
			
			// Sample from open interval (m, n) uniformly.
			inline double Uniform(double m, double n) {
				return Uniform(n-m)+m;
			}
			
			// Sample integer from {0, 1,..., n-1} uniformly.
			inline int Uniform(int n) {
				return rand()%n;
			}
			
			// Sample integer from {m, m+1,..., n-1} uniformly.
			inline int Uniform(int m, int n) {
				return Uniform(n-m)+m;
			}

			// Sample from a zero-mean Gaussian with given standard deviation.
			inline double Normal(double sigma) {
				return Normal(0.0, sigma);
			}

			// Sample from a Gaussian with given mean and standard deviation.
			inline double Normal(double mean, double sigma) {
				double x = 0.0;

				for (int i = 0; i < 12; i++) {
					x += Uniform();
				}

				x = mean + sigma*(x-6.0);

				return x;
			}

			// Get the cumulative probability from normal distribution
			inline double crfChi2(double sigma, double dist) {
				int ind = (int)(Min((dist*dist)/(sigma*sigma)*10, 40.));
				return chi2cdfTable[ind];

			}
			
			// Sample from combination C(n, m) where index is of size m
			inline void Combination(std::vector<int> &index, int n) {
				if (index.size() > n) index.resize(n);	// ensure n >= index.size() = m
				
				std::vector<int> array(n);
								
				for (int i = 0; i < n; i++) {
					array[i] = i;
				}
				
				for (int i = 0; i < index.size(); i++) {
					int j = Uniform(i, n);
					
					std::swap(array[i], array[j]);	
					
					index[i] = array[i];				
				}
			}	
					
			// Sample from combination C(n, m) where index is of size m
			inline void Permutation(std::vector<int> &index, int n) {
				index.resize(n);						// ensure n = index.size() = m

				Combination(index, n);
			}
	};	
	
	/**
	 * Simple Timer
	 *  Grants access to the current time stamp and calculates elapsed time duration
	 */
	class Time {
		public:
			inline Time() {
				Tic();
			}			
			
			inline void Tic() {
				m_clock = clock();
			}

			inline int Toc() {
				return (clock()-m_clock)*1000/CLOCKS_PER_SEC;
			}
			
			inline void Toc(unsigned int *elapsed) {
				*elapsed = Toc();
			}

			inline static time_t Sec() {
				return time(NULL);
			}			

			inline static clock_t Msec() {
				return clock()*1000/CLOCKS_PER_SEC;
			}

		private:
			clock_t m_clock;
	};

	/*
	*
	*
	*/

}

#endif
