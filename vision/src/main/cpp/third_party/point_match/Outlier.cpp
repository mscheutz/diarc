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
// File NAME:		Outlier
// File Function:	Using a heuristic rule to label outliers for the first iteration.
//					Used for the experiments on the outlier set of the Chui-Rangarajan data set.
//
//				Developed by: Yefeng Zheng
//			   First created: April 2004
//			University of Maryland, College Park
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "PointMatch.h"

/*****************************************************************************
/*	Name:		LabelOutlier
/*	Function:	Label outliers using a heuristic rule. Used for the first iteration.
/*	Parameter:	Pnt			-- Point set
/*				nPnt		-- Number of points
/*				r_array		-- Distance matrix between any point pairs
/*				nReservePnt -- Number of reserved points
/*	Return:		0 -- Succeed
/*****************************************************************************/
int LabelOutlier( MYPOINT *Pnt, int nPnt, double **r_array, int nReservePnt )
{
	int		i, j, k, m;
	int		nNN = 6;	//Number of neighbors
	double	**MinDist = (double**)malloc( sizeof(double*)*nPnt );
	//Get the distance from a point to its neighors
	for( i=0; i<nPnt; i++ )
	{
		MinDist[i] = (double*)malloc( sizeof(double)*nNN );
		for( j=0; j<nNN; j++ )
			MinDist[i][j] = 1e+10;
		for( j=0; j<nPnt; j++ )
		{
			if( j == i )	continue;
			if( r_array[i][j] >= MinDist[i][nNN-1] )	continue;	//It is too large
			for( k=nNN-2; k>=0; k-- )
				if( r_array[i][j] > MinDist[i][k] )
					break;
			k++;
			for( m=nNN-2; m>=k; m-- )
				MinDist[i][m+1] = MinDist[i][m];
			MinDist[i][k] = r_array[i][j];
		}
	}
	//Summarize the distance
	double *SumDist = new double[nPnt];
	for( i=0; i<nPnt; i++ )
	{
		SumDist[i] = 0;
		for( j=0; j<nNN; j++ )
			SumDist[i] += MinDist[i][j];
	}
	//Sort the data, get the nReservePnt^th value
	int *PntIndex = new int[nPnt];
	for( i=0; i<nPnt; i++ )
		PntIndex[i] = i;
	int Min;
	for( i=0; i<nPnt; i++ )
	{
		Min = i;
		for( j=i+1; j<nPnt; j++ )
			if( SumDist[j] < SumDist[Min] )
				Min = j;
		if( Min != i )
		{
			double tmp = SumDist[i];
			SumDist[i] = SumDist[Min];
			SumDist[Min] = tmp;
			int tmpIndex = PntIndex[i];
			PntIndex[i] = PntIndex[Min];
			PntIndex[Min] = tmpIndex;
		}
	}
	//Set outliers
	for( i=0; i<nPnt; i++ )
		Pnt[i].nMatch = -1;
	for( i=0; i<nReservePnt; i++ )
		Pnt[PntIndex[i]].nMatch = 0;
	
	//Free memory
	delete SumDist;
	delete PntIndex;
	for( i=0; i<nPnt; i++ )
		free( MinDist[i] );
	free( MinDist );

	return 0;
}

/*****************************************************************************
/*	Name:		SetOutlierCost
/*	Function:	Set the matching cost between an inside point and a outlier point
/*				to infinity. Used for the first iteration only.
/*	Parameter:	costmat -- Cost matrix
/*				nRow    -- Number of rows of the cost matrix.
/*				nCol	-- Number of columns of the cost matrix.
/*				Pnt		-- Point set. The nMatch field acts as an outlier indicator.
/*	Return:		0 -- Succeed
/*****************************************************************************/
int SetOutlierCost( double **costmat, int nRow, int nCol, MYPOINT *Pnt )
{
	int i, j;
	if( nRow < nCol )
	{
		for( i=0; i<nCol; i++ )
		{
			if( Pnt[i].nMatch >= 0 )	continue;
			for( j=0; j<nRow; j++ )
				costmat[j][i] = 1e+10;
		}
	}
	else 
	{
		for( i=0; i<nRow; i++ )
		{
			if( Pnt[i].nMatch >= 0 )	continue;
			for( j=0; j<nCol; j++ )
				costmat[i][j] = 1e+10;
		}
	}

	return 0;
}
