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
// File NAME:		PointMatch.cpp
// File Function:	Providing point matching functions
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

//Parameters which may effect the performance
extern	double	T_Init;		//Temparature of the Gibbs distribution. 
							//Used to transform the shape context distance to a probability measure.
int		E_Ave		= 5;	//Average number of edges per point
double	lambda_o	= 1;	//Regularization parameter (normalized) of the TPS deformation
int		I_Max		= 10;	//Maximum number of iteration

//Flags of different options
int		rotate_invariant_flag	= 0;	//Default: Do not use rotation invariance for the first round
int		relax_graph_match_flag	= 1;	//Default: Using the relaxation labeling method for graph matching
int		affine_LMS_flag			= 1;	//Default: Using LMS for the first iteration
int		all_match_flag			= 0;	//Default: Do not force to find as many matches as possible
int		init_label_outlier_flag	= 0;	//Default: Do not use heuristic rule to label outliers
int		neighborhood_flag		= 0;	//Default: Simple neighborhood definition. 
										//         0 -- Simple neighborhood
										//         1 -- Neighborhood definition robust under non-Uniform scale changes

//Parameter of the shape context
int		nbins_theta	= 12;		//Number of bins in angle
								//Decreasing nbins_theta to eight for the first iteration does not improve the performance.
int		nbins_r		= 5;		//Number of bins in distance
double	r_inner		= 1.0/8;	//The minimum distance
double	r_outer		= 2.0;		//The maximum distance
double	mean_dist_deform, mean_dist_model;  //Mean distances

//Some global variables
double	r_bins_edges[10];		//Lattice of radius
double	**SCModel	= NULL;		//Shape contexts for the model shape
double  **SCDeform	= NULL;		//Shape contexts for the deformed shape	
double	**r_array	= NULL;		//Distance array
double  **theta_array=NULL;		//Theta angle array
double  **costmat	= NULL;		//Matrix of the shape context distances between points from two shapes
double  *TPS		= NULL;		//Matrix of the TPS transform
double  *InvTPS		= NULL;		//Inversion of the matrix of the TPS transform

/*****************************************************************************
/*	Name:		AllocateMemory
/*	Function:	Allocate memory for global variables
/*	Parameter:	nMaxPnt	-- Maximum point number
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	AllocateMemory( int nMaxPnt )
{
	SCModel = (double**)malloc( sizeof(double*)*nMaxPnt );
	SCDeform = (double**)malloc( sizeof(double*)*nMaxPnt );

	r_array = (double**)malloc( sizeof(double*)*nMaxPnt );
	theta_array = (double**)malloc( sizeof(double*)*nMaxPnt );

	costmat = (double**)malloc( sizeof(double*)*nMaxPnt );
	if( SCModel == NULL || SCDeform == NULL || 
		r_array == NULL || theta_array == NULL ||
		costmat == NULL )
	{
		printf( "Memory used up!\n" );
		return -1;
	}

	int nFeature = nbins_theta*nbins_r;
	for( int i=0; i<nMaxPnt; i++ )
	{
		SCModel[i] = (double*)malloc( sizeof(double)*nFeature );
		SCDeform[i] = (double*)malloc( sizeof(double)*nFeature );
		r_array[i] = (double*)malloc( sizeof(double)*nMaxPnt );
		theta_array[i] = (double*)malloc( sizeof(double)*nMaxPnt );
	}
	for(int i=0; i<nMaxPnt; i++ )
	{
		costmat[i] = (double*)malloc( sizeof(double)*nMaxPnt );
		for( int j=0; j<nMaxPnt; j++ )
			costmat[i][j] = 0;
	}
	TPS = (double*)malloc( sizeof(double)*(nMaxPnt+3)*(nMaxPnt+3) );
	InvTPS = (double*)malloc( sizeof(double)*(nMaxPnt+3)*(nMaxPnt+3) );
	return 0;
}

/*****************************************************************************
/*	Name:		FreeMemory
/*	Function:	Free memory used by global variables
/*	Parameter:	nMaxPnt	-- Maximum point number
/*	Return:		0 -- Succeed
/*****************************************************************************/
int FreeMemory( int nMaxPnt )
{
	int	i;
	if( SCModel != NULL )
	{
		for( int i=0; i<nMaxPnt; i++ )
			free( SCModel[i] );
		free( SCModel );
	}

	if( SCDeform != NULL )
	{
		for( int i=0; i<nMaxPnt; i++ )
			free( SCDeform[i] );
		free( SCDeform );
	}

	if( r_array != NULL )
	{
		for( int i=0; i<nMaxPnt; i++ )
			free( r_array[i] );
		free( r_array );
	}

	if( theta_array != NULL )
	{
		for( int i=0; i<nMaxPnt; i++ )
			free( theta_array[i] );
		free( theta_array );
	}

	if( costmat != NULL )
	{
		for( i=0; i<nMaxPnt; i++ )
			free( costmat[i] );
		free( costmat );
	}

	if( TPS != NULL )
		free( TPS );
	if( InvTPS != NULL )
		free( InvTPS );
	return 0;
}

/*************************************************************************************
/*	Name:		PointMatch
/*	Function:	Matching two sets of points (no outliers are considered)
/*				"No outlier" just means that in each iteration, all points are used
/*				to calculate the shape context.
/*				If the difference between nPntModel and nPntDeform is not too large,
/*				you can still use this function.
/*				After matching, the model shape is transformed to the deformed shape.
/*	Parameter:	PntModel   -- Points from the model shape.
/*				nPntModel  -- Number of points from the model shape
/*				PntDeform  -- Points from the deformed shape
/*				nPntDeform -- Number of points from the deformed shape
/*	Return:		0 -- Succeed
/**************************************************************************************/
int PointMatch( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform )
{
	int		i, j;
	int		nGood;
	int		nCurIter = 1;
	int		nMaxPnt = std::max( nPntModel, nPntDeform);
	MYPOINT	*PntModel2	= new MYPOINT[nPntModel];	//Working point set for shape matching

	//Allocate memory
	AllocateMemory( nMaxPnt );
	CalRBinEdge( nbins_r, r_inner, r_outer, r_bins_edges);

	//Initialization working point set
	for( i=0; i<nPntModel; i++ )
	{
		PntModel[i].nMatch	= 0;
		PntModel2[i]		= PntModel[i];
	}

	while( nCurIter <= I_Max )
	{
		if( nCurIter == 1 && rotate_invariant_flag )
			nbins_theta = 4;
		else
			nbins_theta = 12;

		if( all_match_flag == 1 && relax_graph_match_flag == 1 && nCurIter == I_Max)
		{//In the last iteration, using Euclidean distance
			for( i=0; i<nPntModel; i++ )
				for( j=0; j<nPntDeform; j++ )
					costmat[i][j] = GetDistance( PntModel2[i].x, PntModel2[i].y, PntDeform[j].x, PntDeform[j].y );
		}
		else
		{
			//All points are used to calculate shape context. No outliers.
			for( i=0; i<nPntModel; i++ )
				PntModel2[i].nMatch = 0; 
			for( i=0; i<nPntDeform; i++ )
				PntDeform[i].nMatch = 0;

			//Calculate the reference angles
			CalRefAngle( PntModel2, nPntModel );
			//Calculate the distance array
			CalPointDist( PntModel2, nPntModel, r_array, mean_dist_model );
			if( relax_graph_match_flag )		//Set edges of the model graph
			{
				if( neighborhood_flag == 0 )
					SetEdgeSimple( PntModel2, nPntModel, r_array, E_Ave );
				else
					SetEdgeNonUniformScale( PntModel2, nPntModel, r_array, E_Ave );
			}

			//Compute shape context for all points on the model shape
			CalShapeContext(PntModel2, nPntModel, nbins_theta, nbins_r, r_bins_edges, SCModel, r_array, mean_dist_model, nCurIter==1 && rotate_invariant_flag);

			//Calculate the reference angles
			CalRefAngle( PntDeform, nPntDeform );
			//Calculate the distance array
			CalPointDist( PntDeform, nPntDeform, r_array, mean_dist_deform );
			if( relax_graph_match_flag )		//Set edges of the deformed graph
			{
				if( neighborhood_flag == 0 )
					SetEdgeSimple( PntDeform, nPntDeform, r_array, E_Ave);
				else
					SetEdgeNonUniformScale( PntDeform, nPntDeform, r_array, E_Ave);
			}

			//Compute shape context for all points on the deformed shape
			if( nCurIter == 1 )
				CalShapeContext(PntDeform, nPntDeform, nbins_theta, nbins_r, r_bins_edges, SCDeform, r_array, mean_dist_deform, nCurIter==1 && rotate_invariant_flag);
			else
				CalShapeContext(PntDeform, nPntDeform, nbins_theta, nbins_r, r_bins_edges, SCDeform, r_array, mean_dist_model, nCurIter==1 && rotate_invariant_flag);

			//Calculate the shape context distance between any point pair of two shapes
			HistCost( SCModel, nPntModel, SCDeform, nPntDeform, nbins_theta*nbins_r, costmat );
		}

/*		int GetSharedEdgeNumber( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform, int &nEdgeModel, int &nEdgeDeform, int &nEdgeShared);
		int nEdgeModel;
		int nEdgeDeform;
		int nEdgeShared;
		GetSharedEdgeNumber( PntModel2, nPntModel, PntDeform, nPntDeform, nEdgeModel, nEdgeDeform, nEdgeShared);
*/
		int	bRelax;
		if( relax_graph_match_flag == 0 )
			bRelax = 0;
		else if( all_match_flag == 1 && nCurIter == I_Max )    //In the last iteration, we using the Hungarian algorithm
															   //to find as many matches as possible.
			bRelax = 0;
		else
			bRelax = 1;

		if( bRelax )
		{
			RelaxLabel( PntModel2, nPntModel, PntDeform, nPntDeform, costmat );
			int nMatchPair = 0;
			for( i=0; i<nPntModel; i++ )
				if(PntModel2[i].nMatch >= 0)	nMatchPair++;
			if( nMatchPair < 30 )
				SetMinMatch( costmat, PntModel2, nPntModel, PntDeform, nPntDeform, 30, 1 );
		}
		else	//Point matching with the Hungarian algorithm
			HungarianMatch( costmat, PntModel2, nPntModel, PntDeform, nPntDeform, nMaxPnt );

		//Calculate the number of matched points
		BOOL	bChange = FALSE;	
		nGood = 0;
		for( i=0; i<nPntModel; i++ )
		{
			if(PntModel2[i].nMatch >= 0)
				nGood ++;
			if( PntModel[i].nMatch != PntModel2[i].nMatch )
			{
				bChange = TRUE;
				PntModel[i].nMatch = PntModel2[i].nMatch;
			}
		}
		printf( "\tIteration %d\tMatch:%d\n", nCurIter, nGood );

		//Transform the model shape using the affine transformation for the first iteration.
		if( nCurIter == 1 && affine_LMS_flag )
			AffineLMS( PntModel2, nPntModel, PntDeform, nPntDeform );
		else		
		{//Transform the model shape Using the TPS deformation model
			// compute regularization parameter
			double	lambda	= (mean_dist_model*mean_dist_model)*lambda_o;
			TPS_Transform( PntModel, PntModel2, nPntModel, PntDeform, nPntDeform, lambda );
		}

		//Test if the matching results of the previous and current iterations are the same.
		if( bChange == FALSE && nGood == std::min(nPntModel, nPntDeform) )
			break;
		//Increse the iteration number
		nCurIter ++;
	}
	//Copy the transformed shape to the model shape
	for( i=0; i<nPntModel; i++ )
	{
		PntModel[i].x = PntModel2[i].x;
		PntModel[i].y = PntModel2[i].y;
	}
	//Free memory
	FreeMemory( nMaxPnt );
	delete	PntModel2;

	return 0;
}

/*************************************************************************************
/*	Name:		PointMatchOutlier
/*	Function:	Matching two sets of points (considering outliers).
/*				This function is tuned to the Chui-Rangarajan outlier data set.
/*				Generally, you do not need to use this function if the number of points
/*				of two shapes are not significantly different. Use PointMatch( ) instead.
/*				After matching, the model shape is transformed to the deformed shape.
/*	Parameter:	PntModel     -- Point set of the model shape
/*				nPntModel    -- Number of points in the model shape
/*				PntDeform    -- Point set of the deformed shape
/*				nPntDeform   -- Number of points in the deformed shape
/*	Return:		0 -- Succeed
/**************************************************************************************/
int PointMatchOutlier( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform )
{
	int		i, j;
	int		nGood;
	int		nCurIter	= 1;
	int		nMaxPnt		= std::max( nPntModel, nPntDeform );
	MYPOINT	*PntModel2	= new MYPOINT[nPntModel];	//Working point set for shape matching

	//Allocate memory
	AllocateMemory( nMaxPnt );
	CalRBinEdge( nbins_r, r_inner, r_outer, r_bins_edges);

	//Initialization that none point is outlier
	for( i=0; i<nPntModel; i++ )
		PntModel[i].nMatch = 0;
	for( i=0; i<nPntDeform; i++ )
		PntDeform[i].nMatch = 0;

	//Initialization working point set
	for( i=0; i<nPntModel; i++ )
	{
		PntModel[i].nMatch	= 0;
		PntModel2[i]		= PntModel[i];
	}

	while( nCurIter <= I_Max )
	{
		//Initialize the cost matrix
		for( i=0; i<nMaxPnt; i++ )
			for( j=0; j<nMaxPnt; j++ )
				costmat[i][j] = 1e+5;

		//Calculate the reference angles
		CalRefAngle( PntModel2, nPntModel );
		//Calculate the distance array
		CalPointDist( PntModel2, nPntModel, r_array, mean_dist_model );
		if( relax_graph_match_flag )		//Set edges of the model graph
		{
			if( neighborhood_flag == 0 )
				SetEdgeSimple( PntModel2, nPntModel, r_array, E_Ave );
			else
				SetEdgeNonUniformScale( PntModel2, nPntModel, r_array, E_Ave );
		}

		//Set outliers using a heuristic rule for the first iteration.
		//It works on the outlier set of the Chui-Rangarajan data set
		if( init_label_outlier_flag && nCurIter <= 1 && nPntModel > nPntDeform )
			LabelOutlier( PntModel2, nPntModel, r_array, nPntDeform );

		//Compute shape context all points of model shape
		CalShapeContext(PntModel2, nPntModel, nbins_theta, nbins_r, r_bins_edges, SCModel, r_array, mean_dist_model, nCurIter==1 && rotate_invariant_flag);

		//Calculate the reference angles
		CalRefAngle( PntDeform, nPntDeform );
		//Calculate the distance array
		CalPointDist( PntDeform, nPntDeform, r_array, mean_dist_deform );
		if( relax_graph_match_flag )		//Set edges of the model graph
		{
			if( neighborhood_flag == 0 )
				SetEdgeSimple( PntDeform, nPntDeform, r_array, E_Ave);
			else
				SetEdgeNonUniformScale( PntDeform, nPntDeform, r_array, E_Ave);
		}

/*		int GetSharedEdgeNumber( MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform, int &nEdgeModel, int &nEdgeDeform, int &nEdgeShared);
		int nEdgeModel;
		int nEdgeDeform;
		int nEdgeShared;
		GetSharedEdgeNumber( PntModel2, nPntModel, PntDeform, nPntDeform, nEdgeModel, nEdgeDeform, nEdgeShared);
*/
		//Set outliers using a heuristic rule for the first iteration.
		//It works on the outlier set of the Chui-Rangarajan data set.
		if( init_label_outlier_flag && nCurIter <= 1 && nPntDeform > nPntModel )
			LabelOutlier( PntDeform, nPntDeform, r_array, nPntModel );

		//Compute shape context for the deformed shape
		CalShapeContext(PntDeform, nPntDeform, nbins_theta, nbins_r, r_bins_edges, SCDeform, r_array, mean_dist_model, nCurIter==1 && rotate_invariant_flag);

		//Calculate the shape context distance for any point pair between two shapes
		HistCost( SCModel, nPntModel, SCDeform, nPntDeform, nbins_r*nbins_theta, costmat );

		//Set the matching cost between an inside point and an outliers point to infinity. 
		//Used for the first iteration only.
		if( nCurIter <= 1 )
		{
			if( nPntModel > nPntDeform )
				SetOutlierCost( costmat, nPntModel, nPntDeform, PntModel );
			else
				SetOutlierCost( costmat, nPntModel, nPntDeform, PntDeform );
		}

		if( nCurIter < I_Max-1 && relax_graph_match_flag == 1)
		{
			RelaxLabel( PntModel2, nPntModel, PntDeform, nPntDeform, costmat );
			int nMatchPair = 0;
			for( i=0; i<nPntModel; i++ )
				if(PntModel2[i].nMatch >= 0)	nMatchPair++;
			if( nMatchPair < 30 )
				SetMinMatch( costmat, PntModel2, nPntModel, PntDeform, nPntDeform, 30, 1 );
		}
		else//Point matching with the Hungarian algorithm
			HungarianMatch( costmat, PntModel2, nPntModel, PntDeform, nPntDeform, nMaxPnt );

		nGood = 0;
		BOOL	bChange = FALSE;
		for( i=0; i<nPntModel; i++ )
		{
			//This is a bug in the original shape context method.
//			if( PntModel2[i].nMatch >= 0 && PntModel2[i].nMatch < nPntModel)
			//The correct one should be	
			if(PntModel2[i].nMatch >= 0)
				nGood ++;
			if( PntModel[i].nMatch != PntModel2[i].nMatch )
			{
				bChange = TRUE;
				PntModel[i].nMatch = PntModel2[i].nMatch;
			}
		}
		printf( "\tIteration %d\tMatch:%d\n", nCurIter, nGood );

		//Transform the model shape using the affine transformation
		if( nCurIter == 1 && affine_LMS_flag )
			AffineLMS( PntModel2, nPntModel, PntDeform, nPntDeform );
		else		
		{//Transform the model shape using the TPS deformation model
			// compute regularization parameter
			double	lambda=(mean_dist_model*mean_dist_model)*lambda_o;
			TPS_Transform( PntModel, PntModel2, nPntModel, PntDeform, nPntDeform, lambda );
		}
		//Test if the matching results of the previous and current iterations are the same.
		if( bChange == FALSE && nGood == std::min(nPntModel, nPntDeform) )
			break;

		// Set outlier flags
		if( nGood != std::min(nPntModel, nPntDeform ) )
			HungarianMatch( costmat, PntModel2, nPntModel, PntDeform, nPntDeform );
		nCurIter ++;
	}
	//Copy the transformed shape to the model shape
	for( i=0; i<nPntModel; i++ )
	{
		PntModel[i].x = PntModel2[i].x;
		PntModel[i].y = PntModel2[i].y;
	}

	//Free memory
	FreeMemory( nMaxPnt );
	delete	PntModel2;

	return 0;
}

