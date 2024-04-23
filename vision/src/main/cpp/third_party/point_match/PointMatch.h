/*
Copyright (C) 2004	Yefeng Zheng

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You are free to use this program for non-commercial purpose.              
If you plan to use this code in commercial applications, 
you need additional licensing: 
please contact zhengyf@cfar.umd.edu
*/

////////////////////////////////////////////////////////////////////////////////////////////////
// File NAME:		PointMatch.h
// File Function:	Header file
//
//				Developed by: Yefeng Zheng
//			   First created: Aug. 2004
//			University of Maryland, College Park
//					All Right Reserved
///////////////////////////////////////////////////////////////////////////////////////////////////

#if !defined __POINT_MATCH_H__
#define __POINT_MATCH_H__

#ifndef BOOL
#define BOOL bool
#endif

//Parameters that may affect the performance
extern	double	T_Init;		//Temparature of the Gibbs distribution. 
							//Used to transform the shape context distance to a probability measure.
extern  int		E_Ave;		//Average number of edges per point
extern	double	lambda_o;	//Regularization parameter (normalized) of the TPS deformation
extern  int		I_Max;		//Maximum number of iterations

//Flags of different options
extern	int		rotate_invariant_flag;	//Using rotation invariance for the first round
extern	int		relax_graph_match_flag;	//Using relaxation labeling method for graph matching
extern	int		affine_LMS_flag;		//Using LMS for the first iteration
extern	int		all_match_flag;			//Force to find as many matches as possible
extern	int		init_label_outlier_flag;//Using heuristic rule to label outlier
extern	int		neighborhood_flag;		//Different neighborhood definitions

#ifndef PI
#define	PI		3.1415926535898
#endif //PI

//#ifndef		eps			//A value used to avoid dividing zero
//#define		eps 2.2204e-016
//#endif	//eps

#ifndef		MAX_NEIGHBOR_SIZE
#define		MAX_NEIGHBOR_SIZE	30
#endif

struct MYPOINT{
	double	x;	//X coordinate
	double  y;	//Y coordinate
	double	nAngleToCenter;	//Angle from this point to the mass center of the shape
	int		nMatch;			//Matching result
	int		nTrueMatch;		//The true match, used to testing
	int		nNumNeighbor;	//Number of neighbors of this point 
							//It is just the number of edges connecting this point in the graph
	int		nNeighborList[MAX_NEIGHBOR_SIZE];	//Neighbor list
};

//Calculate squared Euclidean distance between two points
double	GetSquareDistance ( MYPOINT &pnt1, MYPOINT &pnt2 );
//Calculate bin edges for a shape context template
int		CalRBinEdge( int nbins_r, double r_inner, double r_outer, double *r_bins_edges );
//Calculate the angle from a point to the mass center of the shape
int		CalRefAngle( MYPOINT *Pnt, int nPnt );
//Calcualte the Euclidean distance between any pair of points
int		CalPointDist(MYPOINT *Pnt, int nPnt, double **r_array, double &mean_dist);
//Calculate the shape context for all points
int		CalShapeContext(MYPOINT *Pnt, int nPnt, int nbins_theta, int nbins_r, 
			   double *r_bin_edges, double **SC, double **r_array, double mean_dist, BOOL bRotateInvariant);
//Calculate the shape context distance matrix
int		HistCost( double **SC1, int nPnt1, double **SC2, int nPnt2, int nbins, double **costmat);

//Hungarian algorithm
double	hungarian( double **assigncost, int dim, int *colsol, int *rowsol);
//Bookstein method for the TPS deformation
int		bookstein( MYPOINT *X, MYPOINT *Y, int nPnt, double beta_k, double *cx, double *cy);

//Estimate the affine transformation from PntModel to PntDeform using the LMS method
//then transform PntModel using the estimated parameters.
int		AffineLMS( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform );
//Estimate the affine transformation from PntModel to PntDeform using the LS method
//then transform PntModel using the estimated parameters.
int		AffineLS( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform);

//Relaxation labeling for point matching
int		RelaxLabel( MYPOINT *RowPnt, int nRowPoint, MYPOINT *ColPnt, int nColPoint, double **costmat );
//Deterministic annealing for point matching
int		DeterministicAnnealing( MYPOINT *RowPnt, int nRowPoint, MYPOINT *ColPnt, int nColPoint, double **costmat );

//Label outliers using a heuristic rule
int		LabelOutlier( MYPOINT *Pnt, int nPnt, double **r_array, int nReservePnt );
//Set the matching cost to an outlier to infinity
int		SetOutlierCost( double **costmat, int nRow, int nCol, MYPOINT *Pnt );

//Point matching without considering the matching to the dummy point.
int		PointMatch( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform );
//Point matching under outlier by considering the matching to the dummy point.
int		PointMatchOutlier( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform );

//Estimate parameters of the TPS model, and transform the model shape toward the deformed shape
int		TPS_Transform( MYPOINT *PntModel, MYPOINT *PntModelNew, int nPntModel, MYPOINT *PntDeform, int nPntDeform, double lambda );

#endif // !defined(__POINT_MATCH_H__)
