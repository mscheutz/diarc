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
// File NAME:		Relax.cpp
// File Function:	Relaxation labeling for graph matching
//
//				Developed by: Yefeng Zheng
//			   First created: April 2004
//			University of Maryland, College Park
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "PointMatch.h"
#include "Tools.h"
#include "Relax.h"

// The minimum value to prevent dividing zero errors
// It may be too large  1e-300 should be more suitable.
#define	MINIMUM_VALUE	1e-10

// Maximum iteration of relaxation labeling
#define R_Max			300

//Temparature of the Gibbs distribution. 
//Used to transform the shape context distance to a probability measure.
double 	T_Init = 0.1;

//Number of points corresponding to rows of costmat and p
int		nRowPnt;
//Number of points corresponding to column of costmat and p
int		nColPnt;

//Matchign probability matrix
double	**P;
//Temporary variable used to update P
double	**P2;

//Matching results of points corresponding to rows of p
int		*RowMatch = NULL;
//Matching results of points corresponding to columns of p
int		*ColMatch = NULL;

/*****************************************************************************
/*	Name:		LabelFirmMatch
/*	Function:	If the probability of a match is higher than a threshold,
/*				keep it firm, and do not update it further.
/*				It is used to reduce the computation demands.
/*	Parameter:	NULL
/*	Return:		1 -- All matches are kept firm. No updating is needed.
/*****************************************************************************/
int LabelFirmMatch( )
{
	int i, j, k;
	for( i=0; i<nRowPnt; i++ )
	{
		if( RowMatch[i] > 0 )	continue;	//It is already firm.

		for( j=0; j<nColPnt; j++ )
		{
			if( P[i][j] < ONE_FIRM )
				continue;
			for( k=0; k<nColPnt; k++ )
				P[i][k] = 0;
			for( k=0; k<nRowPnt; k++ )
				P[k][j] = 0;
			P[i][j] = 1;
			RowMatch[i] = j;
			ColMatch[j] = i;
			break;
		}
	}
	int nRowFirm = 0;
	for( i=0; i<nRowPnt; i++ )
		if( RowMatch[i] >= 0 )
			nRowFirm ++;
	if( nRowFirm == nRowPnt )	return 1;

	int nColFirm = 0;
	for( i=0; i<nColPnt; i++ )
		if( ColMatch[i] >= 0 )
			nColFirm ++;
	if( nColFirm == nColPnt )	return	1;
	return 0;
}

/*****************************************************************************
/*	Name:		DumpProbMatrix
/*	Function:	Dump probability matrix for debug
/*	Parameter:	fn     -- File name
/*				p	   -- Probability matrix
/*				nRow   -- Number of rows
/*				nCol   -- Number of columns
/*	Return:		0 -- Succeed
/*****************************************************************************/
int DumpProbMatrix( char *fn, double **p, int nRow, int nCol )
{
	FILE *fp = fopen( fn, "wb" );
	for( int i=0; i<nRow; i++ )
		fwrite( p[i], sizeof(double), nCol, fp );
	fclose( fp );
	return 0;
}

/*****************************************************************************
/*	Name:		RowNorm
/*	Function:	Row normalization
/*	Parameter:	p			-- Matching probability matrix
/*				nRow		-- Number of rows
/*				nCol		-- Number of columns
/*	Return:		nMaxUpdate	-- Maximum update over all elements in the probability matrix
/*****************************************************************************/
double RowNorm( double **p, int nRow, int nCol )
{
	int i, j;
	double nMaxUpdate = 0;
	for( i=0; i<nRow; i++ )
	{
		if( RowMatch[i] >= 0 )	continue;	//This match is firm. No updating is made.

		double nSum = 0;
		for( j=0; j<nCol; j++ )
			nSum += p[i][j];
		if( nSum < 0 )
			printf( "Probability matrix is wrong!\n" );
		for( j=0; j<nCol; j++ )
		{
			double temp = p[i][j] / (nSum+MINIMUM_VALUE);
			nMaxUpdate = std::max( nMaxUpdate, fabs( temp - p[i][j] ) );
			p[i][j] = temp;
		}
	}
	return nMaxUpdate;
}

/*****************************************************************************
/*	Name:		ColNorm
/*	Function:	Column normalization.
/*	Parameter:	p		-- Matching probability matrix
/*				nRow	-- Number of rows
/*				nCol    -- Number of columns
/*	Return:		nMaxUpdate	-- Maximum update over all elements in the probability matrix
/*****************************************************************************/
double ColNorm( double **p, int nRow, int nCol )
{
	int i, j;
	double nMaxUpdate = 0;
	for( i=0; i<nCol; i++ )
	{
		if( ColMatch[i] >= 0 )	continue;	//This match is firm. No updating is made.

		double nSum = 0;
		for( j=0; j<nRow; j++ )
			nSum += p[j][i];

		if( nSum < 0 )
			printf( "Probability matrix is wrong!\n" );
		for( j=0; j<nRow; j++ )
		{
			double temp = p[j][i] / (nSum+MINIMUM_VALUE);
			nMaxUpdate = std::max( nMaxUpdate, fabs( temp - p[j][i] ) );
			p[j][i] = temp;
		}
	}
	return nMaxUpdate;
}

/*****************************************************************************
/*	Name:		Normalize
/*	Function:	Alterated row and column normalizations to convert the matching
/*				probablity matrix to a generalized doubly stochastic matrix.
/*	Parameter:	p		-- Matching probability matrix. Its actually dimension
/*						   is at least (nRowPnt+1)*(nColPnt+1)
/*				nRowPnt	-- Number of points corresponding to rows of p
/*				nColPnt -- Number of points correponding to columns of p
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	Normalize( double **p, int nRowPnt, int nColPnt )
{
	int		nIter=0;
	double	nMaxUpdate = 1;
	while( nMaxUpdate > 1e-2 && nIter <= 20 )
	{
		if( nRowPnt < nColPnt )
		{
			nMaxUpdate = RowNorm( p, nRowPnt, nColPnt );
			nMaxUpdate += ColNorm( p, nRowPnt+1, nColPnt );
		}
		else if( nRowPnt > nColPnt )
		{
			nMaxUpdate = RowNorm( p, nRowPnt, nColPnt+1 );
			nMaxUpdate += ColNorm( p, nRowPnt, nColPnt );
		}
		else
		{
			nMaxUpdate = RowNorm( p, nRowPnt, nColPnt );
			nMaxUpdate += ColNorm( p, nRowPnt, nColPnt );
		}
		nIter++;
	}
	return 0;
}

/*****************************************************************************
/*	Name:		Convert2Prob
/*	Function:	Calculate the the shape context distance to a probability measure.
/*	Parameter:	costmat -- Matrix for the shape context distances.
/*				p       -- Resulting matching probability matrix
/*				nRowPnt	-- Number of rows
/*				nColPnt -- Number of columns
/*				T       -- Temperature of the Gibbs distribution
/*	Return:		0 -- Succeed
/*****************************************************************************/
int Convert2Prob( double **costmat, double **p, int nRowPnt, double nColPnt, double T)
{
	int i, j;
	for( i=0; i<nRowPnt; i++ )
	{
		for( j=0; j<nColPnt; j++ )
			p[i][j]= exp(-costmat[i][j]/T);
	}	
	return 0;
}

/*****************************************************************************
/*	Name:		SetOutlierProb
/*	Function:	Set the matching probability to a dummy point
/*	Parameter:	p			-- Matching probability matrix
/*				nRowPnt		-- Number of rows of p
/*				nColPnt		-- Number of columns of p
/*				OutlierProb	-- Matching probability to a dummy point
/*	Return:		0 -- Succeed
/*****************************************************************************/
int SetOutlierProb( double **p, int nRowPnt, int nColPnt, double OutlierProb )
{
	for( int i=0; i<nRowPnt; i++ )
		p[i][nColPnt] = OutlierProb;
	for( int i=0; i<nColPnt; i++ )
		p[nRowPnt][i] = OutlierProb;
	return 0;
}

/*****************************************************************************
/*	Name:		InitProb
/*	Function:	Initialize the matching probability matrix.
/*	Parameter:	costmat	-- Cost matrix
/*				p		-- Matching probability matrix
/*				nRowPnt	-- Number of rows
/*				nColPnt	-- Number of columns
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	InitProb( double **costmat, double **p, int nRowPnt, int nColPnt )
{
	//Convert a shape context distance to a probability measure
	Convert2Prob( costmat, p, nRowPnt, nColPnt, T_Init );

	//Set the matching probability to a dummy point
	SetOutlierProb( p, nRowPnt, nColPnt, OUTLIER_PROB );

	//Alterated row and column normalization
	Normalize( p, nRowPnt, nColPnt );

	return 0;
}

/*****************************************************************************
/*	Name:		Support
/*	Function:	Calculate support function of the match between a point pair
/*	Parameter:	RowPnt	-- A row point
/*				ColPnt	-- A column point
/*	Return:		Support function of the match
/*****************************************************************************/
double Support(MYPOINT RowPnt, MYPOINT ColPnt)
{
	int		j, k;
	double	S = 0;
	for( j=0; j<RowPnt.nNumNeighbor; j++ )
	{
		int nIndex1 = RowPnt.nNeighborList[j];
		for( k=0; k<ColPnt.nNumNeighbor; k++ )
		{
			int nIndex2 = ColPnt.nNeighborList[k];
			S += P[nIndex1][nIndex2];
		}
	}
	return S;
}

/*****************************************************************************
/*	Name:		OneIterationRL
/*	Function:	One iteration of relaxation labeling
/*	Parameter:	RowPnt	-- Point set corresponding to rows of p
/*				nRowPnt	-- Number of points of RowPnt
/*				ColPnt	-- Point set corresponding to columns of p
/*				nColPnt	-- Number of points of ColPnt
/*	Return:		0 -- Succeed
/*****************************************************************************/
int OneIterationRL( MYPOINT *RowPnt, int nRowPnt, MYPOINT *ColPnt, int nColPnt)
{
	int		i, j;
	double	S;
	//One iteration of updating
	for( i=0; i<nRowPnt; i++ )
	{
		for( j=0; j<nColPnt; j++ )
		{
			if( RowMatch[i] >= 0 || ColMatch[j] >= 0 )
				P2[i][j] = P[i][j];
			else if ( P[i][j] < ZERO_FIRM )
				P2[i][j] = P[i][j];
			else
			{
				S = Support( RowPnt[i], ColPnt[j]);
				P2[i][j] = P[i][j]*S;
			}
		}
	}
	
	//Update the probability matrix
	for( i=0; i<nRowPnt; i++ )
		memcpy( P[i], P2[i], sizeof(double)*nColPnt );

	//Set matching probability to dummy points
	if( nRowPnt != nColPnt )
		SetOutlierProb( P, nRowPnt, nColPnt, OUTLIER_PROB );

	//Alterated row and column normalization
	Normalize( P, nRowPnt, nColPnt );
	return 0;
}

/*****************************************************************************
/*	Name:		RelaxLabel
/*	Function:	Relaxation labeling updates
/*	Parameter:	RowPnt	  -- Point set corresponding to the rows of costmat
/*				nRowPoint -- Number of points of RowPnt
/*				ColPnt	  -- Point set corresponding to the columns of cosmat
/*				nColPoint -- Number of points of ColPnt
/*				costmat   -- Cost matrix. It is changed after calling.
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	RelaxLabel( MYPOINT *RowPnt, int nRowPoint, MYPOINT *ColPnt, int nColPoint, double **costmat )
{
	int i, j;

	nRowPnt = nRowPoint;
	nColPnt = nColPoint;

	//Intialize the matching results
	RowMatch = new int[nRowPnt];
	ColMatch = new int[nColPnt];
	for( i=0; i<nRowPnt; i++ )
		RowMatch[i] = -1;
	for( i=0; i<nColPnt; i++ )
		ColMatch[i] = -1;

	P = (double**)malloc( sizeof(double)*(nRowPnt+1) );
	P2 = (double**)malloc( sizeof(double)*(nRowPnt+1) );
	for( i=0; i<nRowPnt+1; i++ )
	{
		P[i] = (double*)malloc( sizeof(double)*(nColPnt+1) );
		P2[i] = (double*)malloc( sizeof(double)*(nColPnt+1) );
		memset( P[i], 0, sizeof(double)*(nColPnt+1) );
		memset( P2[i], 0, sizeof(double)*(nColPnt+1) );
	}

	//Using the shape context distance to initialize the matching probability.
	InitProb( costmat, P, nRowPnt, nColPnt );

	//Dump the matching probability matrix for debug
//	DumpProbMatrix( "c:\\matrix_init", p, nRowPnt, nColPnt );

	//Relaxation labeling updates
	int nIter = 0;
	for( nIter=0; nIter<R_Max; nIter++)
	{
		OneIterationRL( RowPnt, nRowPnt, ColPnt, nColPnt );
		//Test if all matches are firm
		if( LabelFirmMatch( ) == 1 )
			break; //No updating is needed.
	}
	printf( "\t# Iteration of relaxation labeling: %d\n", nIter );

	//Dump the matching probability matrix for debug
//	DumpProbMatrix( "c:\\matrix", p, nRowPnt, nColPnt );

	//Update the matching results
	for( i=0; i<nRowPnt; i++ )
		RowPnt[i].nMatch = RowMatch[i];
	for( i=0; i<nColPnt; i++ )
		ColPnt[i].nMatch = ColMatch[i];

	//Update the costmatrix
	for( i=0; i<nRowPnt; i++ )
		for( j=0; j<nColPnt; j++ )
			costmat[i][j] = -P[i][j];

	//Free memory
	for( i=0; i<nRowPnt+1; i++ )
	{
		free( P[i] );
		free( P2[i] );
	}
	free( P );
	free( P2 );
	delete RowMatch;
	delete ColMatch;
	RowMatch = NULL;
	ColMatch = NULL;

	return 0;
}
