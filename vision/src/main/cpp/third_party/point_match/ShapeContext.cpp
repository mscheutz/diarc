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
// File NAME:		ShapeContext.cpp
// File Function:	Compute the shape context
//
//				Developed by: Yefeng Zheng
//			   First created: Aug. 2003
//			University of Maryland, College Park
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "PointMatch.h"
#include "Tools.h"
#include <limits>

extern double **r_array;
extern double **theta_array;

/*****************************************************************************
/*	Name:		CalPointDist
/*	Function:	Calculate the distance array
/*	Parameter:	Pnt       -- Points set
/*				nPnt      -- Number of points
/*				r_array   -- Distance array
/*				mean_dist -- Mean distance
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	CalPointDist(MYPOINT *Pnt, int nPnt, double **r_array, double &mean_dist)
{
	//Calculate the raw Euclidean distance matrix
	int i, j;
	for( i=0; i<nPnt; i++ )
		for( j=0; j<nPnt; j++ )
			r_array[i][j] = sqrt( GetSquareDistance( Pnt[i], Pnt[j] ) );

	//Calculate the mean distance
	mean_dist = 0;
	int		nValid = 0;
	for( i=0; i<nPnt; i++ )
	{
		if( Pnt[i].nMatch == -1 )
			continue;
		for( j=0; j<nPnt; j++ )
		{
			if( Pnt[j].nMatch == -1 )
				continue;
			mean_dist += r_array[i][j];
			nValid++;
		}
	}
	mean_dist /= nValid;
	return 0;
}

/*****************************************************************************
/*	Name:		CalShapeContext
/*	Function:	Compute the shape context
/*	Parameter:	Pnt				-- Points set
/*				nPnt			-- Number of points
/*				nbins_theta		-- Number of bins in angle
/*				nbins_r         -- Number of bins in radius
/*				r_bins_edges    -- Edges of the bins in radius
/*				SC              -- Shape context for each point
/*				r_array			-- Distance array
/*				mean_dist       -- Mean distance
/*				bRotateInvariant-- Rotation invariance or not
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	CalShapeContext(MYPOINT *Pnt, int nPnt, int nbins_theta, int nbins_r, 
			   double *r_bins_edges, double **SC, double **r_array, double mean_dist, BOOL bRotateInvariant)
{
	//Calculate the quantized angle matrix
	int i, j, k;
	for( i=0; i<nPnt; i++ )
	{
		for( j=0; j<nPnt; j++ )
		{
			if( i==j )
				theta_array[i][j] = 0;
			else
			{
				theta_array[i][j] = atan2( Pnt[j].y - Pnt[i].y, Pnt[j].x - Pnt[i].x );
				if( bRotateInvariant )
					theta_array[i][j] -= Pnt[i].nAngleToCenter;
				//put the theta in [0, 2*pi)
				theta_array[i][j] = fmod(fmod(theta_array[i][j]+1e-5,2*PI)+2*PI,2*PI);
				//Qualization
				theta_array[i][j] = floor( theta_array[i][j]*nbins_theta/(2*PI) );
			}
		}
	}

	//Normalization and qualization of radius matrix
	for( i=0; i<nPnt; i++ )
	{
		for( j=0; j<nPnt; j++ )
		{
			r_array[i][j] /= mean_dist;
			for( k=0; k<nbins_r; k++ )
				if( r_array[i][j] <= r_bins_edges[k] )
					break;
			r_array[i][j] = nbins_r-1-k;
		}
	}

	//Counting points inside each bin
	for( i=0; i<nPnt; i++ )
	{
		memset( SC[i], 0, sizeof(double)*nbins_r*nbins_theta );
		for( j=0; j<nPnt; j++ )
		{
			if( i == j )	//Do not count the point itself. This is a bug in the original shape context.
				continue;

			if( Pnt[j].nMatch == -1 )
				continue;
			if( r_array[i][j] < 0 )	// Out of range
				continue;

			int	index = r_array[i][j]*nbins_theta + theta_array[i][j];
			SC[i][index]++;
		}
	}
	return 0;
}

/*****************************************************************************
/*	Name:		CalRBinEdge
/*	Function:	Calculate radius bin edges
/*	Parameter:	nbins_r -- Number of bins in radius
/*				r_inner -- Radius of the inner bin
/*				r_outer -- Radius of the outer bin
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	CalRBinEdge( int nbins_r, double r_inner, double r_outer, double *r_bins_edges )
{
	double	nDist = ( log10(r_outer) - log10(r_inner) ) / (nbins_r-1);
	for( int i=0; i<nbins_r; i++ )
		r_bins_edges[i] = pow(10, log10(r_inner)+nDist*i);
	return 0;
}

/*************************************************************************************
/*	Name:		GetSquareDistance
/*	Function:	Get square of the Euclidean distance of two points
/*	Parameter:	pnt1 -- Point 1
/*				pnt2 -- Point 2
/*	Return:		Squared distance of two points
/**************************************************************************************/
double GetSquareDistance ( MYPOINT &pnt1, MYPOINT &pnt2 )
{
    double dx = pnt1.x - pnt2.x ;
    double dy = pnt1.y - pnt2.y ;
    return dx*dx + dy*dy;
}

/*************************************************************************************
/*	Name:		CalRefAngle
/*	Function:	Calcuate the angle between a point to its mass center.
/*				This angle is used as a reference to obtain rotation invariance SC.
/*	Parameter:	Pnt         -- Points
/*				nPnt        -- Number of points
/*	Return:		0 -- Succeed
/**************************************************************************************/
int	CalRefAngle( MYPOINT *Pnt, int nPnt)
{
	//Calculate the mass center
	MYPOINT CenterPnt;
	CenterPnt.x = 0;
	CenterPnt.y = 0;
	int		nValidPnt = 0;
	for( int i=0; i<nPnt; i++ )
	{
		if( Pnt[i].nMatch == -1 )
			continue;
		CenterPnt.x += Pnt[i].x;
		CenterPnt.y += Pnt[i].y;
		nValidPnt ++;
	}
	CenterPnt.x /= nValidPnt;
	CenterPnt.y /= nValidPnt;

	//Calculate the reference angles
	for(int i=0; i<nPnt; i++ )
		Pnt[i].nAngleToCenter = atan2( Pnt[i].y - CenterPnt.y, Pnt[i].x-CenterPnt.x );
	return 0;
}

/*****************************************************************************
/*	Name:		HistCost
/*	Function:	Calculate the shape context distance
/*	Parameter:	SC1     -- SC of points on shape 1
/*				nPnt1   -- Number of points on shape 1
/*				SC2		-- SC of points on shape 2
/*				nPnt2   -- Number of points on shape 2
/*				nbins   -- Number of bins
/*				costmat -- Distance matrix
/*	Return:		0 -- Succeed
/*****************************************************************************/
int HistCost( double **SC1, int nPnt1, double **SC2, int nPnt2, int nbins, double **costmat)
{
	int i, j, k;
	//Normalization
	double	nsum;
	for( i=0; i<nPnt1; i++ )
	{
	   nsum = std::numeric_limits<double>::epsilon();
	   for( j=0; j<nbins; j++ )
		   nsum += SC1[i][j];
	   for( j=0; j<nbins; j++ )
		   SC1[i][j] /= nsum;
	}
	for( i=0; i<nPnt2; i++ )
	{
	   nsum = std::numeric_limits<double>::epsilon();
	   for( j=0; j<nbins; j++ )
		   nsum += SC2[i][j];
	   for( j=0; j<nbins; j++ )
		   SC2[i][j] /= nsum;
	}

	//Calculate distance
	for( i=0; i<nPnt1; i++ )
	{
	   for( j=0; j<nPnt2; j++ )
	   {
			nsum = 0;
			for( k=0; k<nbins; k++ )
			{
				nsum += (SC1[i][k] - SC2[j][k]) * (SC1[i][k] - SC2[j][k]) / 
						(SC1[i][k] + SC2[j][k] + std::numeric_limits<double>::epsilon() );
			}
			costmat[i][j] = nsum/2;
	   }
	}
	return 0;
}

