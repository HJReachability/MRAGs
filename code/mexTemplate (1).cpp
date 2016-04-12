#include <math.h>
#include "matrix.h"
#include "mex.h"   //--This one is required
#include <float.h>
#include <stdio.h>
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

void finiteDiffHeat(double** u, int J, int N, double h, double k) {
int j, n;
double lambda;
lambda = k/h/h;

for (n=0; n<N-1; n++){
	for (j=1; j<J-1; j++) {
		u[j][n+1] = lambda*u[j+1][n] + (1-2*lambda)*u[j][n] + lambda*u[j-1][n];
	}
}
	
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //---Inside mexFunction---
    //Declarations
    double *uValues;
    double **u;
    const int *dims; 
	int J, N; // total number of grid points in x and t
    int j, n; // jth grid in x, nth grid in t
    double h, k; // grid spacing in x and t
	
    //Get the input
    uValues = mxGetPr(prhs[0]);
	h 		= mxGetScalar(prhs[1]);
	k 		= mxGetScalar(prhs[2]);
	
    dims    = mxGetDimensions(prhs[0]);
    J       = dims[0];
	N       = dims[1];
	
    printf("Size of u: %d by %d\n",J,N);
    printf("(k, h) = (%f, %f)\n",k,h);
	printf("lambda = %f\n", k/h/h);
	
    // memory allocation for u
	u = (double **) malloc ( J * sizeof(double*));

	for (j=0;j<J;j++){
		u[j] = (double *) malloc (N * sizeof(double));
    }

    // Initialize u
    for (j=0; j < J; j++) {
		for (n=0; n < N; n++) {
			u[j][n] = uValues[n*J+j];
        }
	}
    
	// Compute u
	finiteDiffHeat(u, J, N, h, k);
	
    // send the processed u to the output  
    for (j=0; j<J; j++)
		for (n=0; n<N; n++)
            uValues[(n*J)+j] = u[j][n];
    
    
    // delete u;
	for(j=0; j< J; j++){
		free(u[j]);
  	}
	free(u);
   
}
