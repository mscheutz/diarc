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
// File NAME:		Relax.h
// File Function:	Relaxation labeling for graph matching
//
//				Developed by: Yefeng Zheng
//			   First created: April 2004
//			University of Maryland, College Park
//					All Right Reserved
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef	__RELAX_H__
#define	__RELAX_H__

//Probability of matching to an outlier
#define	OUTLIER_PROB	0.2

//Two thresholds used to increase the speed of relaxation labeling
//When the probability of the best matching of a point is higher than this value
//the matching result of this point will be kept firm, and not updated further.
#define	ONE_FIRM		0.95
//If the probability of a match is less than this value,
//the probability will not be updated.
#define	ZERO_FIRM		0.001

//Temparature of the Gibbs distribution. 
//Used to transform the shape context distance to a probability measure.
extern double 	T_Init;

//Number of points corresponding to rows of costmat and p
extern int		nRowPnt;
//Number of points corresponding to column of costmat and p
extern int		nColPnt;

//Matchign probability matrix
extern double	**P;
//Temporary variable used to update P
extern double	**P2;

//Matching results of points corresponding to rows of p
extern int		*RowMatch;
//Matching results of points corresponding to columns of p
extern int		*ColMatch;

// If the probability of a match is higher than a threshold,
// keep it firm, and do not update it further.
int LabelFirmMatch( );
// Dump probability matrix for debug
int DumpProbMatrix( char *fn, double **p, int nRow, int nCol );
// Row normalization
double RowNorm( double **p, int nRow, int nCol );
// Function:	Column normalization
double ColNorm( double **p, int nRow, int nCol );
// Alterated row and column normalizations to convert the matching
// probablity matrix to a generalized doubly stochastic matrix.
int	Normalize( double **p, int nRowPnt, int nColPnt );
// Calculate the the shape context distance to a probability measure.
int Convert2Prob( double **costmat, double **p, int nRowPnt, double nColPnt, double T);
// Set the matching probability to a dummy point
int SetOutlierProb( double **p, int nRowPnt, int nColPnt, double OutlierProb );
// Initialize the matching probability matrix.
int	InitProb( double **costmat, double **p, int nRowPnt, int nColPnt );
// Calculate support function of the match between a point pair
double Support(MYPOINT RowPnt, MYPOINT ColPnt);
// One iteration of relaxation labeling
int OneIterationRL( MYPOINT *RowPnt, int nRowPnt, MYPOINT *ColPnt, int nColPnt);
// Relaxation labeling updates
int	RelaxLabel( MYPOINT *RowPnt, int nRowPoint, MYPOINT *ColPnt, int nColPoint, double **costmat );

#endif //__RELAX_H__