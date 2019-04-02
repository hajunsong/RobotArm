#pragma once

#include <iostream>
#include <math.h>
#include <memory.h>

using namespace std;
typedef unsigned int uint;
const float TINY = 1.0e-20;

#define SIGN(a,b) ((b) >= 0.0f ? fabs(a) : -fabs(a))
#define MAX(x,y) ((x)>(y)?(x):(y))
#define MIN(x,y) ((x)>(y)?(y):(x))

class Numerical
{
public:
	Numerical();
	~Numerical();

	// LU solver
public:
    void ludcmp(float *a, int n, uint* indx, float d, float *fac);
    void lubksb(float *a, int n, uint* indx, float *b, float *x);

	// Singular Value Decomposition
public:
    float pythag(float a, float b);
    void svdcmp(float *a, int m, int n, float *U, float *w, float *v);

	// Integrator - Adams Bashforth 3rd order formulation
public:
    void absh3Initialize(float h, uint array_size);
    float absh3(float *Y, float *Yp, float t_current);
    void getY_next(float *Y);
	bool absh3_flag;
private:
    float step_size, t_next;
	uint n, intcount;
    float *Y_next, *AW, *AW1;
};

