// #ifndef __FIS_H__
// # define __FIS_H__

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <math.h>

#ifndef FIS_H
#define FIS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/animation-interface.h"
#include "fis.h"

#include <fstream>

/***********************************************************************
 Macros and definitions
 **********************************************************************/
/* Define portable printf and DOUBLE */
#if defined(MATLAB_MEX_FILE)
# define PRINTF mexPrintf
# define DOUBLE real_T
#elif defined(__SIMSTRUC__)
# define PRINTF ssPrintf
# define DOUBLE real_T
#else
# define PRINTF printf
# define DOUBLE double
#endif

#ifndef ABS
# define ABS(x)   ( (x) > (0) ? (x): (-(x)) )
#endif
#ifndef MAX
# define MAX(x,y) ( (x) > (y) ? (x) : (y) )
#endif
#ifndef MIN
# define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#endif
#define MF_PARA_N 4
#define STR_LEN 500
#define MF_POINT_N 101

/* debugging macros */
/*
#define PRINT(expr) printf(#expr " = %g\n", (DOUBLE)expr)
#define PRINTMAT(mat,m,n) printf(#mat " = \n"); fisPrintMatrix(mat,m,n)
#define FREEMAT(mat,m) printf("Free " #mat " ...\n"); fisFreeMatrix(mat,m)
#define FREEARRAY(array) printf("Free " #array " ...\n"); free(array)
*/

#if (defined(MATLAB_MEX_FILE) && !defined(__SIMSTRUC__))
# define FREE mxFree
#else
# define FREE free
#endif

#define FREEMAT(mat,m) fisFreeMatrix(mat,m)
#define FREEARRAY(array) FREE(array)

/***********************************************************************
 Data types
 **********************************************************************/

typedef struct fis_node {
    int handle;
    int load_param;
    char name[STR_LEN];
    char type[STR_LEN];
    char andMethod[STR_LEN];
    char orMethod[STR_LEN];
    char impMethod[STR_LEN];
    char aggMethod[STR_LEN];
    char defuzzMethod[STR_LEN];
    int userDefinedAnd;
    int userDefinedOr;
    int userDefinedImp;
    int userDefinedAgg;
    int userDefinedDefuzz;
    int in_n;
    int out_n;
    int rule_n;
    int **rule_list;
    double *rule_weight;
    int *and_or;    /* AND-OR indicator */
    double *firing_strength;
    double *rule_output;
    /* Sugeno: output for each rules */
    /* Mamdani: constrained output MF values of rules */
    struct io_node **input;
    struct io_node **output;
    double (*andFcn)(double, double);
    double (*orFcn)(double, double);
    double (*impFcn)(double, double);
    double (*aggFcn)(double, double);
    double (*defuzzFcn)(struct fis_node*, int, double*, int);
    double *BigOutMfMatrix; /* used for Mamdani system only */
    double *BigWeightMatrix;/* used for Mamdani system only */
    double *mfs_of_rule;    /* MF values in a rule */

    double *bias; /*bias, to be tuned when no rules are fired*/
    int isbias;

    struct fis_node *next;
} FIS;



typedef struct io_node {
    char name[STR_LEN];
    int mf_n;
    double bound[2];
    double value;
    struct mf_node **mf;
} IO;



typedef struct mf_node {
    char label[STR_LEN];    /* MF name */
    char type[STR_LEN];     /* MF type */
    int nparams;            /* length of params field */
    double *params;         /* MF parameters */
    int userDefined;        /* 1 if the MF is user-defined */
    double (*mfFcn)(double, double *); /* pointer to a mem. fcn */ 
    double value;           /* for Sugeno only */
    double *value_array;    /* for Mamdani only, array of MF values */
} MF;


/***********************************************************************
some redefinition universal functions 
 **********************************************************************/
/* display error message and exit */
void fisError(char *msg);

/* define a standard memory access function with error checking */
void *fisCalloc(int num_of_x, int size_of_x);

char **fisCreateMatrix(int row_n, int col_n, int element_size);
void fisFreeMatrix(void **matrix, int row_n);

/***********************************************************************
 Data structure: construction, printing, and destruction 
 **********************************************************************/

/* Build/Free FIS node and load parameter from fismatrix directly */
/* col_n is the number of columns of the fismatrix */
void fisBuildFisNode(FIS *fis, double **fismatrix, int col_n, int numofpoints);
void fisFreeFisNode(FIS *fis);

void fisPrintData(FIS *fis);

/***********************************************************************
 Evaluate the constructed FIS based on given input vector 
 **********************************************************************/

/* given input vector and FIS data structure, return output */
/* this is a wrap-up on fisEvaluate() */  
void getFisOutput(double *input, FIS *fis, double *output);

/* return a FIS matrix with all information */
double **returnFismatrix(const char *fis_file, int *row_n_p, int *col_n_p);

/* return data matrix */
double **returnDataMatrix(char *filename, int *row_n_p, int *col_n_p);

double **returnDataVector(double *a, double *b, double *c, double *d);
#endif /* __FIS__ */


