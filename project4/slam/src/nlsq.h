#ifndef _nlsq_h
#define _nlsq_h
#include <ros/ros.h>
#include <string>
#include "math.h"
struct constraint {
	int p1, p2;
	double mean[3];
	double lambda[3][3];
};
typedef struct constraint constraint_t;

class NLSQ {
	public:

	NLSQ(int maxIterations, int numUnknowns, int measurementVectorDimension);
	~NLSQ();

	void optimize();

	void setErrorDimension(int n);
	void setExtraSpace(int n);
	void setMaxIterations(int n);
	void setNumUnknowns(int n);
	void setMesurementVectorDimension(int n);
	string info();

	private:
	int error_dimension;
	int extra_space;
	int max_iterations;
	int num_unknowns;
	int mesurement_vector_dimension;	

	void errv(double *p, double *e, int m, int n, void *data);
};

#endif
