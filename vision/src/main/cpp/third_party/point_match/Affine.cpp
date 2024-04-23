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
// File NAME:		Affine.cpp
// File Function:	Affine estimation and transformation
//
//				Developed by: Yefeng Zheng
//			   First created: Aug. 2003
//			University of Maryland, College Park
///////////////////////////////////////////////////////////////////////////////////////////////////

#include "stdlib.h"
#include <math.h>
#include "PointMatch.h"
#include "Tools.h"

extern double mean_dist_model;

/*************************************************************************************
/*	Name:		IsSmallTriangle
/*	Function:	Test if three points form a small triangle. Only large triangles are
/*				used to estimate the affine transformation.
/*	Parameter:	Pnt	      -- Point set of the shape
/*				ValidPnt  -- Indexes of three candidate points
/*				mean_dist -- Mean distance
/*	Return:		0 -- Succeed
/**************************************************************************************/
BOOL IsSmallTriangle( MYPOINT *Pnt, int *ValidPnt, double mean_dist )
{
	double Dist1 = GetDistance( Pnt[ValidPnt[0]].x, Pnt[ValidPnt[0]].y, Pnt[ValidPnt[1]].x, Pnt[ValidPnt[1]].y );
	if( Dist1 < 0.8*mean_dist )
		return TRUE;
	double	Dist2 = GetDistance( Pnt[ValidPnt[0]].x, Pnt[ValidPnt[0]].y, Pnt[ValidPnt[2]].x, Pnt[ValidPnt[2]].y );
	if( Dist2 < 0.8*mean_dist )
		return TRUE;
	double Dist3 = GetDistance( Pnt[ValidPnt[1]].x, Pnt[ValidPnt[1]].y, Pnt[ValidPnt[2]].x, Pnt[ValidPnt[2]].y );
	if( Dist3 < 0.8*mean_dist )
		return TRUE;
	//Test if three points are on the same line
	double theta1 = atan2( Pnt[ValidPnt[1]].y - Pnt[ValidPnt[0]].y, 
						   Pnt[ValidPnt[1]].x - Pnt[ValidPnt[0]].x);

	double theta2 = atan2( Pnt[ValidPnt[2]].y - Pnt[ValidPnt[1]].y, 
						   Pnt[ValidPnt[2]].x - Pnt[ValidPnt[1]].x);

	double theta3 = atan2( Pnt[ValidPnt[0]].y - Pnt[ValidPnt[2]].y, 
						   Pnt[ValidPnt[0]].x - Pnt[ValidPnt[2]].x);

	double angle1 = theta2 - theta1 - PI;
	//Convert it to 0 to 2*PI
	while( angle1 > 2*PI )	angle1 -= 2*PI;
	while( angle1 < 0 )		angle1 += 2*PI;
	//Convert it to 0 to PI
	if( angle1 > PI )	angle1 = 2*PI-angle1;
	//Test if the angle is too small
	if( angle1 < 2*PI/180 || angle1 > 178*PI/180 )
		return TRUE;

	double angle2 = theta2 - theta3 - PI;
	while( angle2 > 2*PI ) angle2 -= 2*PI;
	while( angle2 < 0 )	angle2 += 2*PI;
	if( angle2 > PI )	angle2 = 2*PI-angle2;
	if( angle2 < 2*PI/180 || angle2 > 178*PI/180 )
		return TRUE;

	double angle3 = theta3 - theta1 - PI;
	while( angle3 > 2*PI )	angle3 -= 2*PI;
	while( angle3 < 0 )	angle3 += 2*PI;
	if( angle3 > PI )	angle3 = 2*PI-angle3;
	if( angle3 < 2*PI/180 || angle3 > 178*PI/180 )
		return TRUE;
	return FALSE;
}

/*****************************************************************************
/*	Name:		Permute
/*	Function:	Permute the points randomly
/*	Parameter:	nPos   -- Indexes of all points
/*				nPoint -- Number of points
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	Permute( int *nPos, int nPoint )
{
	int i;
	for( int nIter=0; nIter<10; nIter++ )
	{
		for(i=0; i<nPoint; i++ )
		{
			int nSwitch = rand()*nPoint/RAND_MAX;
			if( nSwitch >= nPoint )
				nSwitch = nPoint-1;
			int temp = nPos[i];
			nPos[i] = nPos[nSwitch];
			nPos[nSwitch] = temp;
		}
	}
	return 0;
}

/*****************************************************************************
/*	Name:		AffineEst
/*	Function:	Estimate parameters of the affine transformation from the model
/*				point set to the deformed point set.
/*	Parameter:	PntModel	-- Points on the model shape
/*				nPntModel	-- Number of points on the model shape
/*				PntDeform   -- Points on the deformed shape
/*				nPntDeform  -- Number of points on the deformed shape
/*				Trans       -- Translation
/*				A           -- Affine transformation matrix
/*	Return:		0 -- Succeed
/*****************************************************************************/
int AffineEst( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform, MYPOINT &Trans, double A[2][2])
{
	int	i, j;
	double	P[3][3];
	double  Q[3][2];
	for( i=0; i<3; i++ )
		for( j=0; j<3; j++ )
			P[i][j] = 0;
	for( i=0; i<3; i++ )
		for( j=0; j<2; j++ )
			Q[i][j] = 0;

	for( i=0; i<nPntModel; i++ )
	{
		if( PntModel[i].nMatch == -1 )
			continue;

		P[0][0] ++;
		P[0][1] += PntModel[i].x;
		P[0][2] += PntModel[i].y;
		P[1][1] += PntModel[i].x * PntModel[i].x;
		P[1][2] += PntModel[i].x * PntModel[i].y;
		P[2][2] += PntModel[i].y * PntModel[i].y;

		int nMatch = PntModel[i].nMatch;
		Q[0][0] += PntDeform[nMatch].x;
		Q[0][1] += PntDeform[nMatch].y;
		Q[1][0] += PntDeform[nMatch].x * PntModel[i].x;
		Q[1][1] += PntDeform[nMatch].y * PntModel[i].x;
		Q[2][0] += PntDeform[nMatch].x * PntModel[i].y;
		Q[2][1] += PntDeform[nMatch].y * PntModel[i].y;
	}

	P[1][0] = P[0][1];
	P[2][0] = P[0][2];
	P[2][1] = P[1][2];

	Inv( (double*)P, 3 );

	double	a[3][2];
	MatrixMultiply( (double*)P, (double*)Q, (double*)a, 3, 3, 2 );

	Trans.x = a[0][0];
	Trans.y = a[0][1];
	A[0][0] = a[1][0];
	A[0][1] = a[2][0];
	A[1][0] = a[1][1];
	A[1][1] = a[2][1];
	return 0;
}

/*****************************************************************************
/*	Name:		AffineEst2
/*	Function:	Estimate parameters of the affine transformation from the model
/*				point set to the deformed point set.
/*				The difference with affine_est() is that the match correpondence
/*				of point PntModel[i] is point PntDeform[i].
/*	Parameter:	PntModel   -- Points on the model shape
/*				PntDeform  -- Points on the deformed shape
/*				nPnt	   -- Number of points
/*				Trans      -- Translation of the affine transformation
/*				A          -- Affine transformation matrix
/*	Return:		0 -- Succeed
/*****************************************************************************/
int AffineEst2( MYPOINT *PntModel, MYPOINT *PntDeform, int nPnt, MYPOINT &Trans, double A[2][2])
{
	int	i, j;
	double	P[3][3];
	double  Q[3][2];
	for( i=0; i<3; i++ )
		for( j=0; j<3; j++ )
			P[i][j] = 0;
	for( i=0; i<3; i++ )
		for( j=0; j<2; j++ )
			Q[i][j] = 0;

	for( i=0; i<nPnt; i++ )
	{
		P[0][0] ++;
		P[0][1] += PntModel[i].x;
		P[0][2] += PntModel[i].y;
		P[1][1] += PntModel[i].x * PntModel[i].x;
		P[1][2] += PntModel[i].x * PntModel[i].y;
		P[2][2] += PntModel[i].y * PntModel[i].y;

		Q[0][0] += PntDeform[i].x;
		Q[0][1] += PntDeform[i].y;
		Q[1][0] += PntDeform[i].x * PntModel[i].x;
		Q[1][1] += PntDeform[i].y * PntModel[i].x;
		Q[2][0] += PntDeform[i].x * PntModel[i].y;
		Q[2][1] += PntDeform[i].y * PntModel[i].y;
	}

	P[1][0] = P[0][1];
	P[2][0] = P[0][2];
	P[2][1] = P[1][2];

	if( Inv( (double*)P, 3 ) != 0 )
		return -1;

	double	a[3][2];
	MatrixMultiply( (double*)P, (double*)Q, (double*)a, 3, 3, 2 );

	Trans.x = a[0][0];
	Trans.y = a[0][1];
	A[0][0] = a[1][0];
	A[0][1] = a[2][0];
	A[1][0] = a[1][1];
	A[1][1] = a[2][1];
	return 0;
}

/*****************************************************************************
/*	Name:		AffineTransform
/*	Function:	Perform affine transformation
/*	Parameter:	x  -- X coordinate of a point
/*				y  -- Y coordinate of a point
/*				u  -- X coordinate after transformation
/*				v  -- Y coordinate after transformation
/*				Trans    -- Translation of the affine transformation
/*				A        -- Affine transformation matrix
/*	Return:		0 -- Succeed
/*****************************************************************************/
int AffineTransform( double x, double y, double &u, double &v, MYPOINT Trans, double A[2][2] )
{
	u = A[0][0] * x + A[0][1] * y + Trans.x;
	v = A[1][0] * x + A[1][1] * y + Trans.y;
	return 0;
}

/*****************************************************************************
/*	Name:		AffineTransform
/*	Function:	Perform affine transformation to all points in a shape
/*	Parameter:	Pnt    -- A set of points
/*				nPnt   -- Number of points
/*				Trans  -- Translation of the affine transformation
/*				A      -- Affine transformation matrix
/*	Return:		0 -- Succeed
/*****************************************************************************/
int AffineTransform( MYPOINT *Pnt, int nPnt, MYPOINT Trans, double A[2][2] )
{
	for( int i=0; i<nPnt; i++ )
	{
		double u, v;
		AffineTransform( Pnt[i].x, Pnt[i].y, u, v, Trans, A );
		Pnt[i].x = u;
		Pnt[i].y = v;
	}
	return 0;
}

/*****************************************************************************
/*	Name:		GetAffineResidual
/*	Function:	Calculate the residual after affine transformation.
/*				Model point set is transformed. The residual error is calculated
/*				as the Euclidean distance from a transformed model point to 
/*				the corresponding deformed point.
/*	Parameter:	PntModel   -- Points on the model shape
/*				nPntModel  -- Number of points on the model shape
/*				PntDeform  -- Points on the deformed shape
/*				Trans      -- Translation of the affine transformation
/*				A          -- Affine transformation matrix
/*	Return:		0 -- Succeed
/*****************************************************************************/
int GetAffineResidual( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, MYPOINT Trans, double A[2][2], double *nErr )
{
	int i;
	int		nValidPnt = 0;
	double	u, v;
	for( i=0; i<nPntModel; i++ )
	{
		int nMatch = PntModel[i].nMatch;
		if( nMatch < 0 )	continue;
		AffineTransform( PntModel[i].x, PntModel[i].y, u, v, Trans, A);
		nErr[nValidPnt] = GetDistance( PntDeform[nMatch].x, PntDeform[nMatch].y, u, v );
		nValidPnt++;
	}
	return 0;
}

/*****************************************************************************
/*	Name:		GetMedianAffineResidual
/*	Function:	Calculate the median of the residual errors after affine transformation
/*	Parameter:	PntModel    -- Points on the model shape
/*				nPntModel	-- Number of points on the model shape
/*				PntDeform   -- Points on the deformed shape
/*				Trans       -- Translation of the affine transformation
/*				A           -- Affine transformation matrix
/*	Return:		0 -- Succeed
/*****************************************************************************/
double GetMedianAffineResidual( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, MYPOINT Trans, double A[2][2])
{
	double	*Err = new double[nPntModel];
	int		nValidPnt = 0;
	for( int i=0; i<nPntModel; i++ )
		if( PntModel[i].nMatch >= 0 )
			nValidPnt++;

	GetAffineResidual( PntModel, nPntModel, PntDeform, Trans, A, Err );
	double nMedian = GetMedian( Err, nValidPnt );
	delete Err;
	return nMedian;
}

/*****************************************************************************
/*	Name:		AffineLS
/*	Function:	Estimate the affine transformation using LS (Least Squares) method,
/*				then transformation the model point set using the estimated 
/*				affine transformation parameters.
/*	Parameter:	PntModel    -- Points on the model shape
/*				nPntModel   -- Number of points on the model shape
/*				PntDeform   -- Points on the deformed shape
/*				nPntDeform  -- Number of points on the deformed shape
/*	Return:		0 -- Succeed
/*****************************************************************************/
int AffineLS( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform)
{
	//Affine parameters
	MYPOINT Trans;
	double  A[2][2];

	//Estimate affine parameters
	AffineEst( PntModel, nPntModel, PntDeform, nPntDeform, Trans, A );

	//Perform affine transformation
	AffineTransform( PntModel, nPntModel, Trans, A );

	return 0;
}

/*****************************************************************************
/*	Name:		AffineEstLMS
/*	Function:	Estimate parameters of the affine transformation from the model
/*				point set to the deformed point set using the LMS method.
/*	Parameter:	PntModel    -- Points on the model shape
/*				nPntModel   -- Number of points on the model shape
/*				PntDeform   -- Points on the deformed shape
/*				nPntDeform  -- Number of points on the deformed shape
/*				Trans       -- Translation of the affine transformation
/*				A           -- Affine transformation matrix
/*	Return:		0 -- Succeed
/*****************************************************************************/
int AffineEstLMS( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform, 
				  MYPOINT &Trans, double A[2][2] )
{
	int i, j;
	//Temporary affine parameters for selected subset of matches
	MYPOINT Trans2;
	double	A2[2][2];
	MYPOINT Pnt1[3], Pnt2[3];
	double	nMinMedianErr = 1e+10;

	//Get the valid points
	int		*ValidPnt = new int[std::min(nPntModel, nPntDeform)];
	int		nValidPnt = 0;
	for( i=0; i<nPntModel; i++ )
	{
		if( PntModel[i].nMatch < 0 )
			continue;
		ValidPnt[nValidPnt] = i;
		nValidPnt++;
	}
	if( nValidPnt < 0.3*std::min( nPntModel, nPntDeform) ) //Too few matching pairs. Use them all to estimate the affine transformation
		AffineEst( PntModel, nPntModel, PntDeform, nPntDeform, Trans, A );
	else
	{
		int	nPermuteNumber = 0;
		//Permute the valid points
		Permute( ValidPnt, nValidPnt );

		int nStart=0;
		int nSample = 0;
		//Increasing the number of subset does not significantly improve the performance.
		//I tried using 100,000 subsets, but no significant difference.
		while( nSample < 1000 && nPermuteNumber < 1000 )
		{
			//Select points ValidPnt[nStart], ValidPnt[nStart+1], ValidPnt[nStart+2] to estimate the affine transformation.
			//Permute the valid points if we reach the end of the queue.
			if( nStart == nValidPnt-3 )
			{
				Permute( ValidPnt, nValidPnt );
				nStart = 0;
				nPermuteNumber++;
			}

			//Only large triangles, which are reliable, are use to estimate the affine transformation.
			BOOL IsSmall = IsSmallTriangle( PntModel, &ValidPnt[nStart], mean_dist_model );
			if( IsSmall == FALSE )
			{
				//Estimate the affine transformation
				for( j=0; j<3; j++ )
				{
					Pnt1[j] = PntModel[ValidPnt[nStart+j]];
					int nMatch = Pnt1[j].nMatch;
					Pnt2[j] = PntDeform[nMatch];
				}
				if( AffineEst2( Pnt1, Pnt2, 3, Trans2, A2 ) == 0 )
				{
					//Calculate the median residual
					double nMedianErr = GetMedianAffineResidual( PntModel, nPntModel, PntDeform, Trans2, A2);
					if( nMedianErr < nMinMedianErr )
					{
						nMinMedianErr = nMedianErr;
						Trans = Trans2;
						for( i=0; i<2; i++ )
							for( j=0; j<2; j++ )
								A[i][j] = A2[i][j];
					}
					nSample++;
				}
			}
			nStart++;
		}
		if( nPermuteNumber >= 300 )
			AffineEst( PntModel, nPntModel, PntDeform, nPntDeform, Trans, A );
	}
	//Free memory
	delete ValidPnt;

	return 0;
}

/*****************************************************************************
/*	Name:		AffineLMS
/*	Function:	Estimate the affine transformation using LMS (Least Median Squares) method,
/*				then transform the model point set using the estimated 
/*				affine transformation parameters.
/*	Parameter:	PntModel    -- Points on the model shape
/*				nPntModel   -- Number of points on the model shape
/*				PntDeform   -- Points on the deformed shape
/*				nPntDeform  -- Number of points on the deformed shape
/*	Return:		0 -- Succeed
/*****************************************************************************/
int AffineLMS( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform)
{
	//Set a seed for the random number generator. We use this to remove randomness.
	srand(0);

	//Affine parameters
	MYPOINT Trans;
	double  A[2][2];

	//Estimate affine parameters
	AffineEstLMS( PntModel, nPntModel, PntDeform, nPntDeform, Trans, A );

	//Perform affine transformation
	AffineTransform( PntModel, nPntModel, Trans, A);

	return 0;
}
