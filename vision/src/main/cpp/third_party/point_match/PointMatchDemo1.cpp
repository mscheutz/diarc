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
// File NAME:		PointMatchDemo1.cpp
// File Function:	This is a demo on the synthesized Chui-Rangarajan data set.
//					Details of our approach are available from our technical report:
//					Y. Zheng and D. Doermann, "Robust Point Matching for Non-Rigid Shapes: 
//					A Relaxation Labeling Based Approach,"
//					Tech. Rep. LAMP-TR-117, University of Maryland, College Park, USA, 2004.
//
//					For details of the data set, please refer to the following paper:
//					H. Chui and A. Rangarajan, "A New Point Matching Algorithm for Non-Rigid Registration," 
//					CV&IU (Computer Vision & Image Understanding), 89(2-3), 2003: 114-141.
//
//				Developed by: Yefeng Zheng
//			   First created: Sept. 2004
//			University of Maryland, College Park
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "PointMatch.h"
#include "Tools.h"

double	nMeanErr[10], nSigmaErr[10];
double	nMeanMisMatch[10], nSigmaMisMatch[10];

/*************************************************************************************
/*	Name:		CalMatchError
/*	Function:	Calculate matching errors
/*	Parameter:	PntModel	-- Points of the model shape after transformation
/*				PntTruth    -- Ground truth
/*				nPnt        -- Number of points
/*				nErr        -- Average residual error
/*				nMisMatch   -- Number of mis-matched points
/*	Return:		0 -- Succeed
/**************************************************************************************/
int CalMatchError( MYPOINT *PntModel, MYPOINT *PntTruth, int nPnt,  double &nErr, int &nMisMatch )
{
	int nValidPnt = 0;
	nErr = 0;
	for( int i=0; i<nPnt; i++ )
	{
		int nTrueMatch = PntModel[i].nTrueMatch;
		if( nTrueMatch < 0 )	continue;
		nValidPnt ++;

		double dist0 = (PntTruth[nTrueMatch].x - PntModel[i].x) * (PntTruth[nTrueMatch].x - PntModel[i].x) + 
				(PntTruth[nTrueMatch].y - PntModel[i].y) * (PntTruth[nTrueMatch].y - PntModel[i].y);
		dist0 = sqrt( dist0 );
		nErr += dist0;
	}
	nErr /= nValidPnt;

	nMisMatch = 0;
	for( int i=0; i<nPnt; i++ )
	{
		int nTrueMatch = PntModel[i].nTrueMatch;
		if( nTrueMatch < 0 )	continue;		
		if( PntModel[i].nMatch != PntModel[i].nTrueMatch)
			nMisMatch++;
	}
	return 0;
}

/*************************************************************************************
/*	Name:		SingleTest
/*	Function:	Matching two sets of points
/*	Parameter:	fnSyn     -- Name of the file storing two sets of points
/*				fnLog     -- Log file name
/*				nErr      -- Average residual error
/*				nMisMatch -- Number of the mis-matched points
/*	Return:		0 -- Succeed
/**************************************************************************************/
int SingleTest( char *fnSyn, char *fnLog, double &nErr, int &nMisMatch )
{
	printf( "Processing %s ...\n", GetFileName( fnSyn ).c_str() );

	MYPOINT	*PntModel	= NULL;	//Original points from model shape
	MYPOINT	*PntDeform	= NULL;	//Original points from deformed shape
	MYPOINT	*PntTruth	= NULL;	//Groundtruth
	int	nPntModel, nPntDeform, nPntTruth;
	if( ReadSynFile( fnSyn, PntModel, nPntModel, PntDeform, nPntDeform, PntTruth, nPntTruth ) != 0 )
	{
		printf( "Read link file: %s failed!\n", fnSyn );
		return -1;
	}

	//Matching two sets of shapes
	//Attention: After mathcing, PntModel is transformed toward PntDeform
	if( nPntModel == nPntDeform )
		PointMatch(PntModel, nPntModel, PntDeform, nPntDeform);
	else
		PointMatchOutlier(PntModel, nPntModel, PntDeform, nPntDeform );

	//Calculate matching errors
	nErr = 0;
	nMisMatch = 0;
	if( nPntTruth > 0 )
	{
		CalMatchError( PntModel, PntTruth, nPntModel, nErr, nMisMatch );

		printf( "\tMean Err:%f\t", nErr );
		printf( "Mismatch:%d\n", nMisMatch );
		
		FILE *fpLog = fopen( fnLog, "at" );
		fprintf( fpLog, "%s\t%d\t%f\n", GetFileNameWithoutExt(fnSyn).c_str(), nMisMatch, nErr );
		fclose( fpLog );
	}
	
	//Free memory
	delete	PntModel;
	delete	PntDeform;
	delete	PntTruth;

	return 0;
}

/*****************************************************************************
/*	Name:		BatchTest
/*	Function:	Batch testing on the synthesized data set
/*	Parameter:	pattern		-- Pattern of the testing shape. 
/*							   It may be "fish" or "chinese."
/*				type		-- Type of the robustness test. 
/*							   It may be def, noise, outlier, or rot.
/*				nTotalLevel -- Total levels of deformations
/*				nTotalTrial -- Total trials at each level
/*				fnLog		-- Log file name
/*	Return:		0 -- Succeed
/*****************************************************************************/
int BatchTest( char *pattern, char *type, int nTotalLevel, int nTotalTrial, char *fnLog )
{
	char	fnSynFile[1024];//Current processing image
	double	nErr;				//Average residual error
	int		nMisMatch;			//Number of mis-matched points
	FILE*	fpLog;

	//unsigned	Time = GetTickCount();
	remove( fnLog );

	for( int nLevel=0; nLevel<nTotalLevel; nLevel++ )
	{
		nMeanErr[nLevel] = 0;
		nSigmaErr[nLevel] = 0;
		nMeanMisMatch[nLevel] = 0;
		nSigmaMisMatch[nLevel] = 0;

		for( int nTrial=0; nTrial<nTotalTrial; nTrial++ )
		{
			sprintf( fnSynFile, "save_%s_%s_%d_%d.syn", pattern, type, nLevel+1, nTrial+1 );
			SingleTest( fnSynFile, fnLog, nErr, nMisMatch );
			nMeanErr[nLevel] += nErr;
			nSigmaErr[nLevel] += nErr * nErr;
			nMeanMisMatch[nLevel] += nMisMatch;
			nSigmaMisMatch[nLevel] += nMisMatch*nMisMatch;
		}
		nMeanErr[nLevel] /= nTotalTrial;
		nSigmaErr[nLevel] = sqrt( nSigmaErr[nLevel]/nTotalTrial - nMeanErr[nLevel]*nMeanErr[nLevel] );
		nMeanMisMatch[nLevel] /= nTotalTrial;
		nSigmaMisMatch[nLevel] = sqrt( nSigmaMisMatch[nLevel]/nTotalTrial - nMeanMisMatch[nLevel]*nMeanMisMatch[nLevel] );

		fpLog = fopen( fnLog, "at" );
		fprintf( fpLog, "------------------------------------------\n" );
		fprintf( fpLog, "Level %d\t Mean Err: %f\tSigma Err: %f\n\tMismatch: %f\tSigma Mismatch: %f\n", 
				nLevel+1, nMeanErr[nLevel], nSigmaErr[nLevel], nMeanMisMatch[nLevel], nSigmaMisMatch[nLevel] );
		fprintf( fpLog, "------------------------------------------\n\n" );
		fclose( fpLog );
	}

	fpLog = fopen( fnLog, "at" );
	fprintf( fpLog, "\n------------------------------------------\n" );
	for( int nLevel=0; nLevel<nTotalLevel; nLevel++ )
		fprintf( fpLog, "Level %d\t Mean Err: %f\tSigma Err: %f\n\tMismatch: %f\tSigma Mismatch: %f\n", 
				nLevel+1, nMeanErr[nLevel], nSigmaErr[nLevel], nMeanMisMatch[nLevel], nSigmaMisMatch[nLevel] );
	fprintf( fpLog, "------------------------------------------\n" );
	fclose( fpLog );

	//Time = GetTickCount() - Time;
	//printf( "Total: %f seconds\tAver: %f s/comparison\n", 0.001*Time, 0.001*Time/(nTotalLevel*nTotalTrial) );
	return 0;
}

/*****************************************************************************
/*	Name:		Usage
/*	Function:	Print help 
/*	Parameter:	NULL
/*	Return:		NULL
/*****************************************************************************/
int Usage()
{
	printf( "PointMatchDemo1 pattern type\n" );
	printf( "Testing results are saved in a log file: pattern_type.log.\n" );
	printf( "    pattern -- fish or chinese. Two shapes in the Chui-Rangarajan data set.\n" );
	printf( "    type    -- def, noise, outlier, rot, or occlusion.\n" );
	printf( "               def       -- Testing robustness under deformation.\n" );
	printf( "               noise     -- Testing robustness under noise.\n" );
	printf( "               outlier   -- Testing robustness under outliers.\n" );
	printf( "               rot       -- Testing robustness under rotation.\n" );
	printf( "               occlusion -- Testing robustness under occlusion.\n" );
	printf( " Examples:\n" );
	printf( " PointMatchDemo1 fish def\n" );
	printf( "     Testing on the fish_def data set.\n" );
	printf( "     The results are saved in the file of fish_def.log.\n" );
	return 0;
}

/*****************************************************************************
/*	Name:		_tmain
/*	Function:	Main function
/*	Parameter:	
/*	Return:		0 -- Succeed
/*****************************************************************************/
int main(int argc, char **argv)
{
//Initialize parameters
	relax_graph_match_flag	= 1;
	rotate_invariant_flag	= 0;
	affine_LMS_flag			= 1;
	all_match_flag			= 1;

	if( argc != 3 )
	{
		Usage();
		return -1;
	}

	int		nTotalLevel;
	int		nTotalTrial = 100;
	char	pattern[20];
	char	type[20];
	strcpy( pattern, argv[argc-2] );
	strcpy( type, argv[argc-1] );
	if( strcmp( type, "def" ) == 0 )
	{
		nTotalLevel	= 5;
		E_Ave		= 7;
		T_Init		= 0.05;
	}
	else if	(strcmp( type, "noise" ) == 0 )
	{
		nTotalLevel = 6;
		E_Ave		= 9;
		lambda_o	= 3;	//Increase the weight of smoothing regularization for the noise data set
	}
	else if( strcmp( type, "outlier" ) == 0 )
	{
		nTotalLevel				= 5;
		E_Ave					= 5;
		init_label_outlier_flag	= 1;
	}
	else if( strcmp( type, "rot" ) == 0 )
	{
		nTotalLevel				= 6;
		E_Ave					= 5;
		rotate_invariant_flag	= 1;	//Rotation invariant
	}
	else if( strcmp( type, "occlusion" ) == 0 )
	{
		nTotalLevel				= 6;
		E_Ave					= 5;
		T_Init					= 0.0005;
		init_label_outlier_flag	= 0;
	}
	else
	{
		printf( "Type %s is not available!\n", type );
		Usage();
		return -1;
	}

	//Begin batch testing
	char fnLog[1024];
	sprintf( fnLog, "%s_%s.log", pattern, type );
	BatchTest( pattern, type, nTotalLevel, nTotalTrial, fnLog );

	return 0;
}
