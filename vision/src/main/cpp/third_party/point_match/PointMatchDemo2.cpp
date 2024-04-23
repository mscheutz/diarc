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
// File Function:	This is a demo of our relaxation labeling based approach.
//					Details of our approach are available from our technical report:
//					Y. Zheng and D. Doermann, "Robust Point Matching for Non-Rigid Shapes: 
//					A Relaxation Labeling Based Approach,"
//					Tech. Rep. LAMP-TR-117, University of Maryland, College Park, USA, 2004.
//
//				Developed by: Yefeng Zheng
//			   First created: Aug. 2004
//			University of Maryland, College Park
////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "PointMatch.h"
#include "Tools.h"
#include <math.h>

/*************************************************************************************
/*	Name:		SingleTest
/*	Function:	Matching two sets of points. The model point set is warped toward the 
/*				deformed point set during matching.
/*	Parameter:	fnPntModel  -- File of the model point set
/*				fnPntDeform	-- File of the deformed point set
/*				fnMatch		-- Name of the file to save the point mathing results
/*	Return:		0 -- Succeed
/**************************************************************************************/
int SingleTest( char *fnPntModel, char *fnPntDeform, char *fnMatch )
{
	MYPOINT	*PntModel	= NULL;	//Model point set
	MYPOINT	*PntDeform	= NULL;	//Deformed point set
	int	nPntModel, nPntDeform;
	if( ReadPointFile( fnPntModel, PntModel, nPntModel ) != 0 )
	{
		printf( "Read point file %s failed\n", fnPntModel );
		return -1;
	}
	if( ReadPointFile( fnPntDeform, PntDeform, nPntDeform ) != 0 )
	{
		printf( "Read point file: %s failed!\n", fnPntDeform );
		return -1;
	}

	//PointMatch() function will change PntModel, keep a copy of PntModel
	MYPOINT *PntModel2 = new MYPOINT[nPntModel];
	for( int i=0; i<nPntModel; i++ )
		PntModel2[i] = PntModel[i];

	//Matching two sets of points
	//PntModel is warped toward PntDeform after point matching
	int nMinPnt = std::min( nPntModel, nPntDeform );
	int nMaxPnt = std::max( nPntModel, nPntDeform );
	if( nMinPnt > nMaxPnt/2 )	//The outlier rate is not too high
		PointMatch( PntModel, nPntModel, PntDeform, nPntDeform );
	else
		PointMatchOutlier( PntModel, nPntModel, PntDeform, nPntDeform );

	//Since the point positions may be changed after matching, we copy matching results only.
	for( int i=0; i<nPntModel; i++ )
		PntModel2[i].nMatch = PntModel[i].nMatch;

	//Dump matching result
	DumpMatch( fnMatch, PntModel2, nPntModel, PntDeform, nPntDeform );

	//Free memory
	delete PntModel;
	delete PntDeform;
	delete PntModel2;

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
	printf( "PointMatchDemo2 [-g] [-a] [-r] [-f] [-t T_Init] [-e E_Ave] [-i I_Max] fnPntModel fnPntDeform\n" );
	printf( "Output of this program is a file with suffix *.match.\n" );
	printf( "  -g        -- Do not use relaxation labeling based graph matching.\n" );
	printf( "  -a        -- Do not use the affine transformation for the first iteration.\n" );
	printf( "  -r        -- Rotation invariant matching.\n" );
	printf( "               Avoid using it unless necessary. Imposing rotation invariance\n" );
	printf( "               may deteriorate the performance.\n" );
	printf( "  -f        -- Force to find as many matches as possible.\n" );
	printf( "  -t T_Init -- Parameter related to converting the shape context distance\n" );
	printf( "               to a probability measure.\n" );
	printf( "               Default: 0.1\n" );
	printf( "  -e E_Ave  -- Average number of neighbors of a point.\n" );
	printf( "               Default: 5\n" );
	printf( "  -i I_Max  -- Maximum number of iterations.\n" );
	printf( "               Default: 10\n" );
	printf( "  fnPntModel  -- File containing the model point set.\n" );
	printf( "  fnPntDeform -- File containing the deformed point set.\n" );
	printf( "NOTE: After each iteration of point matching, the model point set is\n" );
	printf( "      warped toward the deformed point set. Therefore, the matching procedure\n" );
	printf( "      is not symmetric. The point matching results of the following commands may be different.\n" );
	printf( "      'PointMatchDemo2 fnPoint1 fnPoint2'\n" );
	printf( "      'PointMatchDemo2 fnPoint2 fnPoint1'\n" );
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
	relax_graph_match_flag = 1;
	affine_LMS_flag = 1;
	rotate_invariant_flag = 0;
	all_match_flag = 0;

//Parse command line
	int i = 0;
	for(i=1;i<argc;i++)
	{
		if(argv[i][0] != '-') break;
		++i;
		switch(tolower(argv[i-1][1]))
		{
		case 'g':
			relax_graph_match_flag = 0;
			i--;
			break;
		case 'a':
			affine_LMS_flag = 0;
			i--;
			break;
		case 'r':
			rotate_invariant_flag = 1;
			i--;
			break;
		case 'f':
			all_match_flag = 1;
			i--;
			break;
		case 't':
			T_Init	= atof( argv[i] );
			break;
		case 'e':
			E_Ave = atoi( argv[i] );
			if( E_Ave >= MAX_NEIGHBOR_SIZE )
			{
				printf("The average number if edges for a point is too large\n" );
				printf("because the maximum number of edges for a point is %d.\n", MAX_NEIGHBOR_SIZE );
				return -1;
			}
			break;
		case 'i':
			I_Max = atoi( argv[i] );
			break;
		case '?':
		case 'h':
			Usage();
			return 0;
		default:
			printf( "Input parameters error!\n" );
			Usage();
			return -1;
		}
	}
	if( i!= argc-2 )
	{
		Usage();
		return -1;
	}

	char	fnMatch[1024];
	char	fnPntModel[1024];
	char	fnPntDeform[1024];
	strcpy( fnPntModel, argv[argc-2] );
	strcpy( fnPntDeform, argv[argc-1] );

	//Point matching
	sprintf( fnMatch, "%s_%s.match", GetFileNameWithoutExt( fnPntModel ).c_str(), GetFileNameWithoutExt( fnPntDeform ).c_str() );
	SingleTest( fnPntModel, fnPntDeform, fnMatch );
	return 0;
}
