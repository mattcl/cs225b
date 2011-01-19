#include "nlsq.h"
#include "levmar-2.5/levmar.h"

const int MAX_ITERATIONS = 100;
const int EXTRA_SPACE = 3;
const int ERROR_DIMENSION = 3;


NLSQ::NLSQ(){ 
	extra_space = EXTRA_SPACE;
	error_dimension = ERROR_DIMENSION;
	max_iterations = MAX_ITERATIONS;
	num_unknowns = 0;
	measurement_vector_dimension = 0;
}

NLSQ::~NLSQ() {

}

void NLSQ::optimize() {
	constraint_t *constraints = new constraint[10]; // for now
	int ret = dlevmar_dif(errv, initial_parameter_estimates, NULL, num_unknowns, measurement_vector_dimension, max_iterations, NULL, NULL, NULL, NULL, (void *)constraints);
	if(ret < 0)
		ROS_ERROR("error optimizing function");
}

void NLSQ::errv(double *p, double *e, int m, int n, void *data) {
	double *es = e;
	constraint_t *ms = (constraint_t *)data;
	for(int i = 0; i < (n - extra_space) / error_dimension; i++, ms++, e += error_dimension) {
		double *v1 = &p[3*ms->p1];
		double *v2 = &p[3*ms->p2];
		
		// compute delta z
		d = sqrt((v1[0] - v2[0])*(v1[0] - v2[0]) + (v1[1] - v2[1])*(v1[1] - v2[1]));
		alpha = atan2(v2[1] - v1[1], v2[0] - v1[0]);
		double h[3];
		h[0] = d * cos(alpha - v1[2]);
		h[1] = d * sin(alpha - v1[2]);
		h[2] = v1[2] - v2[2];
	
		double z[3];
		z[0] = h[0] - ms->mean[0];
		z[1] = h[1] - ms->mean[1];
		z[2] = h[2] - ms->mean[2];
		
		e[0] = z[0];
		e[1] = z[1];
		e[2] = z[2];
		
		// compute error
//		double **sic = ms->lambda;
//		double inter[3];
//		inter[0] = z[0] * sic[0][0] + z[1] * sic[1][0] + z[2] * sic[2][0];
//		inter[1] = z[0] * sic[0][1] + z[1] * sic[1][1] + z[2] * sic[2][1];
//		inter[2] = z[0] * sic[0][2] + z[1] * sic[1][2] + z[2] * sic[2][2];
		
		// set error
//		e[0] = inter[0] * z[0] + inter[1] * z[1] + inter[2] * z[2];
		
	}	

	// add a constraint such that the first pose is at 0, 0, 0
	ms = (constraint_t *)data;
	double *v = &p[0];
	e[0] = v[0];
	e[1] = v[1];
	e[2] = v[2];
}

void NLSQ::setErrorDimension(int n) {
	error_dimension = n;
}

void NLSQ::setExtraSpace(int n) {
	extra_space = n;
}

void NLSQ::setMaxIterations(int n) {
	max_iterations = n;
}

void NLSQ::setNumUnknowns(int n) {
	num_unknowns = n;
}

void NLSQ::setMeasurementVectorDimension(int n) {
	measurement_vector_dimension;
}
