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
// File NAME:		Bookstein.cpp
// File Function:	TPS deformation using the Bookstein algorithm. Please refer to the 
//					following paper for details
//					F.L. Bookstein. Principal Warps: Thin-Plate Splines and the Decomposition
//					of Deformation. IEEE PAMI, 11(6), 1989: 567-585.
//
//				Developed by: Yefeng Zheng
//			   First created: Aug. 2003
//			University of Maryland, College Park
///////////////////////////////////////////////////////////////////////////////////////////////////

#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "Tools.h"
#include "PointMatch.h"
#include <math.h>
#include <limits>

extern double *TPS;
extern double *InvTPS;

/*****************************************************************************
/*	Name:		Cal_cxy
/*	Function:	Calculate cx and cy for TPS deformation
/*	Parameter:	InvTPS   -- Inversion of the matrix of the TPS deformation
/*				dim      -- Dimension of the matrix
/*				Y        -- Model points
/*				cx       -- cx of TPS deformation
/*				cy       -- cy of TPS deformation
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	Cal_cxy( double *InvTPS, int dim, MYPOINT *Y, double *cx, double *cy )
{
	int i, j;
	for( i=0; i<dim; i++ )
	{
		cx[i] = 0;
		cy[i] = 0;
		for( j=0; j<dim-3; j++ )
		{
			cx[i] += InvTPS[i*dim+j]*Y[j].x;
			cy[i] += InvTPS[i*dim+j]*Y[j].y;
		}
	}
	return 0;
}

/*****************************************************************************
/*	Name:		Cal_E
/*	Function:	Calculate bending energy of TPS deformation model
/*	Parameter:	TPS		-- Matrix of the TPS deformation
/*				dim		-- Dimension of the matrix
/*				cx		-- cx of TPS deformation
/*				cy		-- cy of TPS deformation
/*				TPS_cost-- Bending energy of TPS deformation model
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	Cal_E( double *TPS, int dim, double *cx, double *cy, double &TPS_cost )
{
	TPS_cost = 0;
	int i, j;
	for( i=0; i<dim-3; i++ )
	{
		for( j=0; j<dim-3; j++ )
		{
			TPS_cost += cx[i] * TPS[i*dim+j] * cx[j];
			TPS_cost += cy[i] * TPS[i*dim+j] * cy[j];
		}
	}
	TPS_cost /= 2;
	return 0;
}

/*****************************************************************************
/*	Name:		bookstein
/*	Function:	Calculate the TPS deformation using the Bookstein algorithm.
/*				Bending Y to X.
/*	Parameter:	X        -- Point set 1
/*				Y        -- Point set 2
/*				nPnt     -- Number of poionts
/*				cx       -- cx of TPS deformation
/*				cy       -- cy of TPS deformation
/*	Return:		0 -- Succeed
/*****************************************************************************/
int bookstein( MYPOINT *X, MYPOINT *Y, int nPnt, double beta_k, double *cx, double *cy)
{
	int i, j;
	// compute distances between points
	int dim = nPnt + 3;
	memset( TPS, 0, sizeof(double)*dim*dim);
	for( i=0; i<nPnt; i++ )
	{
		for( j=0; j<nPnt; j++ )
		{
			TPS[i*dim+j] = GetSquareDistance( X[i], X[j] );
			if( i == j )
				TPS[i*dim+j]  *= log( TPS[i*dim+j] + 1 );
			else
				TPS[i*dim+j]  *= log(TPS[i*dim+j]);
		}
	}
	
	// adding submatrix P and P'
	for( i=0; i<nPnt; i++ )
	{
		TPS[i*dim+dim-3] = 1;
		TPS[i*dim+dim-2] = X[i].x;
		TPS[i*dim+dim-1] = X[i].y;

		TPS[(dim-3)*dim + i] = 1;
		TPS[(dim-2)*dim + i] = X[i].x;
		TPS[(dim-1)*dim + i] = X[i].y;
	}

	//Regularization
	for( i=0; i<dim-3; i++ )
		TPS[i*dim+i] += beta_k;

	//Inversion
	memcpy( InvTPS, TPS, sizeof(double)*dim*dim );
	if( Inv( InvTPS, dim ) != 0 )
	{
		printf( "bookstein failed!\n" );
		return -1;
	}

	// Calculate cx and cy of the TPS deformation
	Cal_cxy( InvTPS, dim, Y, cx, cy );

	return 0;
}

/*****************************************************************************
/*	Name:		TPS_Transform
/*	Function:	Perform TPS deformation
/*	Parameter:	Pnt			-- Point set
/*				PntNew		-- Point set after the TPS deformation
/*				nPnt		-- Number of poionts
/*				PntAnchor	-- Anchor point set for the TPS deformation
/*				nPntAnchor	-- Number of anchor points
/*				cx			-- cx of TPS deformation parameters
/*				cy			-- cy of TPS deformation parameters
/*	Return:		0 -- Succeed
/*****************************************************************************/
int TPS_Transform( MYPOINT *Pnt, MYPOINT *PntNew, int nPnt, MYPOINT *PntAnchor, int nPntAnchor, double *cx, double *cy )
{
	int i, j;
	// warp each coordinate
	for( i=0; i<nPnt; i++ )
	{
		PntNew[i].x = cx[nPntAnchor] + cx[nPntAnchor+1] * Pnt[i].x + cx[nPntAnchor+2]*Pnt[i].y;
		PntNew[i].y = cy[nPntAnchor] + cy[nPntAnchor+1] * Pnt[i].x + cy[nPntAnchor+2]*Pnt[i].y;

		for( j=0; j<nPntAnchor; j++ )
		{
			double	d2 = GetSquareDistance( PntAnchor[j], Pnt[i] );
			d2 *= log(d2+std::numeric_limits<double>::epsilon());
			PntNew[i].x += cx[j]*d2;
			PntNew[i].y += cy[j]*d2;
		}
	}

	return 0;
}

/*****************************************************************************
/*	Name:		TPS_Transform
/*	Function:	Estimate parameters of TPS model, then perform TPS deformation.
/*				The model shape is bended toward the deformed shape.
/*	Parameter:	PntModel	-- Point set of the model shape
/*				PntModelNew	-- Point set of the model shape after bending
/*				nPntModel	-- Number of points in the model shape
/*				PntDeform	-- Point set of the deformed shape
/*				nPntDeform	-- Number of points in the deformed shape
/*				lambda		-- Regularization parameter
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	TPS_Transform( MYPOINT *PntModel, MYPOINT *PntModelNew, int nPntModel, 
				   MYPOINT *PntDeform, int nPntDeform, double lambda )
{
	int		nMatch	= 0;
	for( int i=0; i<nPntModel; i++ )
		if( PntModel[i].nMatch >= 0 )
			nMatch++;

	double	*cx = new double[nMatch+4];		//cx of the TPS deformation
	double	*cy = new double[nMatch+4];		//cy of the TPS deformation

	MYPOINT	*PntModelTemp	= new MYPOINT[nPntModel];
	MYPOINT *PntDeformTemp	= new MYPOINT[nPntDeform];

	//Remove outliers. PntDeformTemp is the result after outlier removal
	RemoveOutlier( PntDeform, PntDeformTemp, nPntDeform );

	//Sort the model shape points using the matching result
	//Using the original points PntModel!!!
	//PntDeform is used to provide matching correpondence
	//PntModelTemp is the result after point sorting
	SortPoints( PntModel, PntModelTemp, PntDeform, nPntDeform );

	//Estimate the TPS deformation parameters
	bookstein( PntModelTemp, PntDeformTemp, nMatch, lambda, cx, cy);
	//Perform the TPS deformation
	TPS_Transform( PntModel, PntModelNew, nPntModel, PntModelTemp, nMatch, cx, cy );	

	//Release memory
	delete PntModelTemp;
	delete PntDeformTemp;
	delete cx;
	delete cy;

	return 0;
}
