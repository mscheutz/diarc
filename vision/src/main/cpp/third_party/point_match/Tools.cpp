/*Copyright (C) 2004	Yefeng Zheng

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

/////////////////////////////////////////////////////////////////////////
// File NAME:		Tools.cpp
// File Function:	Commnon functions by other modules
//
//				Developed by: Yefeng Zheng
//				First created: May, 2004
//			University of Maryland, College Park
/////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "Tools.h"
#include "PointMatch.h"

/*************************************************************************************
/*	Name:		GetDistance
/*	Function:	Get distance of two points
/*	Parameter:	x1 -- X ordinate of point 1
/*				y1 -- Y ordinate of point 1
/*				x2 -- X ordinate of point 2
/*				y2 -- Y ordinate of point 2
/*	Return:		Distance of two points
/**************************************************************************************/
double GetDistance ( double x1, double y1, double x2, double y2 )
{
    double dx = x1 - x2;
    double dy = y1 - y2 ;
    return sqrt( dx*dx + dy*dy ) ;
}

////////////////////////////////////////////////////////////////////////
// Functions related to setting edges in a graph
////////////////////////////////////////////////////////////////////////

typedef	struct 
{
	int		Node1;
	int		Node2;
	double	Dist;
}EDGE; //Data structure of an edge

/*****************************************************************************
/*	Name:		AddEdge
/*	Function:	Add an edge to the list. Only the first nMaxPnt shortest edges
/*				are preserved.
/*	Parameter:	Edge       -- List of edges
/*				nEdge      -- Number of edges
/*				nMaxPnt    -- Maximum number of edges preserved
/*				Node1      -- One end point of an edge
/*				Node2      -- The other end point of an edge
/*				Dist	   -- Length of the edge
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	AddEdge( EDGE *Edge, int &nEdge, int nMaxPnt, int Node1, int Node2, double Dist )
{
	int i, j;
	//Find the insertion position
	for( i=nMaxPnt-1; i>=0; i-- )
		if( Edge[i].Dist < Dist )
			break;
	//It is larger than any value in the array
	if( i == nMaxPnt-1 )
		return 0;

	//Insert the node
	for( j=nEdge-1; j>i; j-- )
		Edge[j+1] = Edge[j];
	Edge[i+1].Node1 = Node1;
	Edge[i+1].Node2 = Node2;
	Edge[i+1].Dist = Dist;
	nEdge++;

	//Remove tails
	if( nEdge >= nMaxPnt+1 )
	{
		for( j=nMaxPnt-1; j<nEdge-1; j++ )
		{
			if( Edge[j].Dist != Edge[j+1].Dist )
			{
				nEdge = j+1;
				break;
			}
		}
	}
	return 0;
}

/*****************************************************************************
/*	Name:		SetEdgeSimple_Outlier
/*	Function:	The neighborhood of a normal point should not include outliers.
/*				For an outlier point, if exist a point A that both are in each other's
/*				E_Ave nearest neighbors, we set A as its neighbor.
/*				The neighborhood definition is no longer symmetric.
/*	Parameter:	Pnt       -- Points set
/*				nPnt      -- Number of points
/*				r_arrary  -- Distance array between any point pair
/*				E_Ave	  -- Average number of edges per node
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	SetEdgeSimple_Outlier( MYPOINT *Pnt, int nPnt, double **r_array, double E_Ave )
{
	Pnt[0].nMatch = -1;
	Pnt[1].nMatch = -1;

	int		i, j;
	// Set neighborhood for outliers
	SetEdgeNonUniformScale( Pnt, nPnt, r_array, E_Ave );
	for( i=0; i<nPnt; i++ )
	{
		if( Pnt[i].nMatch >= 0 )
			Pnt[i].nNumNeighbor = 0;
	}

	// Set neighborhood for normal points
	int		nMaxPnt = nPnt*E_Ave/2;
	int		MAX_EDGE_NUM = nPnt*(E_Ave+2);
	EDGE	*Edge = new EDGE[MAX_EDGE_NUM];
	int		nEdge = 0;
	//Initialization
	for( i=0; i<MAX_EDGE_NUM; i++ )
	{
		Edge[i].Node1 = -1;
		Edge[i].Node2 = -1;
		Edge[i].Dist = 1e+10;
	}

	//Keep the shortest edges
	for( i=0; i<nPnt; i++ )
	{
		if( Pnt[i].nMatch < 0 )	
			continue;
		for( j=i+1; j<nPnt; j++ )
		{
			if( Pnt[j].nMatch < 0 )
				continue;
			AddEdge( Edge, nEdge, nMaxPnt, i, j, r_array[i][j] );
			//The maximum overflow is 20%
			nEdge = (int)std::min( (double)nEdge, nMaxPnt*1.2 );
		}
	}

	//Add edges to nodes
	for( i=0; i<nEdge; i++ )
	{
		int		Node1 = Edge[i].Node1;
		int		Node2 = Edge[i].Node2;

		if( Pnt[Node1].nNumNeighbor < MAX_NEIGHBOR_SIZE )
		{
			Pnt[Node1].nNeighborList[Pnt[Node1].nNumNeighbor] = Node2;
			Pnt[Node1].nNumNeighbor++;
		}
		if( Pnt[Node2].nNumNeighbor < MAX_NEIGHBOR_SIZE )
		{
			Pnt[Node2].nNeighborList[Pnt[Node2].nNumNeighbor] = Node1;
			Pnt[Node2].nNumNeighbor++;
		}
	}
	delete Edge;

// Dump graph
//	DumpGraph( "C:\\Graph.txt", Pnt, nPnt );

	return 0;
}

/*****************************************************************************
/*	Name:		SetEdgeSimple
/*	Function:	Preserving the nPnt*E_Ave shortest edges in the graph.
/*	Parameter:	Pnt       -- Points set
/*				nPnt      -- Number of points
/*				r_arrary  -- Distance array between any point pair
/*				E_Ave	  -- Average number of edges per node
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	SetEdgeSimple( MYPOINT *Pnt, int nPnt, double **r_array, double E_Ave )
{
	int		i, j;
	int		nMaxPnt = nPnt*E_Ave/2;
	int		MAX_EDGE_NUM = nPnt*(E_Ave+2);
	EDGE	*Edge = new EDGE[MAX_EDGE_NUM];
	int		nEdge = 0;
	//Initialization
	for( i=0; i<MAX_EDGE_NUM; i++ )
	{
		Edge[i].Node1 = -1;
		Edge[i].Node2 = -1;
		Edge[i].Dist = 1e+10;
	}

	for( i=0; i<nPnt; i++ )
		Pnt[i].nNumNeighbor = 0;

	//Keep the shortest edges
	for( i=0; i<nPnt; i++ )
	{
		for( j=i+1; j<nPnt; j++ )
		{
			AddEdge( Edge, nEdge, nMaxPnt, i, j, r_array[i][j] );
			//The maximum overflow is 20%
			nEdge = (int)std::min( (double)nEdge, nMaxPnt*1.2 );
		}
	}

	//Add edges to nodes
	for( i=0; i<nEdge; i++ )
	{
		int		Node1 = Edge[i].Node1;
		int		Node2 = Edge[i].Node2;

		if( Pnt[Node1].nNumNeighbor < MAX_NEIGHBOR_SIZE )
		{
			Pnt[Node1].nNeighborList[Pnt[Node1].nNumNeighbor] = Node2;
			Pnt[Node1].nNumNeighbor++;
		}
		if( Pnt[Node2].nNumNeighbor < MAX_NEIGHBOR_SIZE )
		{
			Pnt[Node2].nNeighborList[Pnt[Node2].nNumNeighbor] = Node1;
			Pnt[Node2].nNumNeighbor++;
		}
	}
	delete Edge;

// Dump graph
//	DumpGraph( "C:\\Graph.txt", Pnt, nPnt );

	return 0;
}

/*****************************************************************************
/*	Name:		SetEdgeNonUniformScale
/*	Function:	This neighborhood definition deals with the case that different
/*				parts may undergo different degrees of deformation. Suppose A 
/*				and B are two points in a shape, if A is among the top E_ave 
/*				nearest points of B, and B is among the top E_ave nearest points
/*				of A, then A and B are neighbors.
/*	Parameter:	Pnt       -- Points set
/*				nPnt      -- Number of points
/*				r_arrary  -- Distance array between any point pair
/*				E_Ave	  -- Average number of edges per node
/*	Return:		0 -- Succeed
/*****************************************************************************/
int	SetEdgeNonUniformScale( MYPOINT *Pnt, int nPnt, double **r_array, double E_Ave )
{
	int i, j, k;
	// Initialize the return parameters
	for( i=0; i<nPnt; i++ )
		Pnt[i].nNumNeighbor = 0;

	// Allocate memory to hold the list of the top E_Ave nearst points for each point
	EDGE	**pEdge;;
	int		*nEdge;
	nEdge = new int[nPnt];
	memset( nEdge, 0, sizeof(int)*nPnt );
	pEdge = new EDGE*[nPnt];
	for( i=0; i<nPnt; i++ )
	{
		// There may be ties in the list of top E_Ave nearest points.
		// We allocate more memory than E_Ave for this case.
		pEdge[i] = new EDGE[2*(int)E_Ave];
		for( j=0; j<2*E_Ave; j++ )
		{
			pEdge[i][j].Node1 = -1;
			pEdge[i][j].Node2 = -1;
			pEdge[i][j].Dist = 1e+10;
		}
	}

	// Calculate the top E_Ave nearest point for each point
	for( i=0; i<nPnt; i++ )
	{
		for( j=0; j<nPnt; j++ )
		{
			if( i == j )	continue;	// Do not count itself
			AddEdge( pEdge[i], nEdge[i], E_Ave, i, j, r_array[i][j] );
			// Allow some overflow
			nEdge[i] = std::min( (double)nEdge[i], E_Ave*2 );
		}
	}

	// Add edges to nodes
	int		nTotalEdge = 0;
	for( i=0; i<nPnt; i++ )
	{
		int Node1 = i;
		for( j=0; j<nEdge[i]; j++ )
		{
			int Node2 = pEdge[i][j].Node2;
			// Do not count the same edge twice
			if( Node1 > Node2 )		continue;
			
			for( k=0; k<nEdge[Node2]; k++ )
			{
				if( pEdge[Node2][k].Node2 == Node1 )
				{// Node1 and Node2 are both in each other's top E_Ave list of nearest points
					if( Pnt[Node1].nNumNeighbor < MAX_NEIGHBOR_SIZE )
					{
						Pnt[Node1].nNeighborList[Pnt[Node1].nNumNeighbor] = Node2;
						Pnt[Node1].nNumNeighbor++;
					}
					if( Pnt[Node2].nNumNeighbor < MAX_NEIGHBOR_SIZE )
					{
						Pnt[Node2].nNeighborList[Pnt[Node2].nNumNeighbor] = Node1;
						Pnt[Node2].nNumNeighbor++;
					}
					nTotalEdge++;
					break;
				}
			}
		}
	}

	for( i=0; i<nPnt; i++ )
		delete pEdge[i];
	delete nEdge;
	delete pEdge;

//Dump graph
//	DumpGraph( "C:\\Graph.txt", Pnt, nPnt );

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
// Hungarian matching
/////////////////////////////////////////////////////////////////////////////////

/*****************************************************************************
/*	Name:		HungarianMatch
/*	Function:	Point matching using the Hungarian algorithm. Arbitary number
/*				of outliers are allowed.
/*	Parameter:	costmat   -- Cost matrix of comparisons
/*				RowPnt    -- The set of points corresponding to rows of costmat
/*				nRowPnt   -- Number of points of RowPnt
/*				ColPnt	  -- The set of points corresponding to columns of costmat
/*				nColPnt   -- Number of points of ColPnt
/*				nTotalPnt -- The dimension of costmat is nTotalPnt*nTotalPnt.
/*							 nTotalPnt may be larger than max(nRowPnt, nColPnt)
/*							 if matching to the dummy point is permitted.
/*	Return:		0 -- Succeed
/*****************************************************************************/
double HungarianMatch( double **costmat, MYPOINT *RowPnt, int nRowPnt, MYPOINT *ColPnt, int nColPnt, int nTotalPnt )
{
	int		*colsol = new int[nTotalPnt];		//Matching results of the columns
	int		*rowsol = new int[nTotalPnt];		//Matching results of the rows

	double cost = hungarian(costmat, nTotalPnt, colsol, rowsol);
	for( int i=0; i<nRowPnt; i++ )
	{
		if( rowsol[i] >= nColPnt )
			RowPnt[i].nMatch = -1;
		else
			RowPnt[i].nMatch = rowsol[i];
	}

	for( int i=0; i<nColPnt; i++ )
	{
		if( colsol[i] >= nRowPnt )
			ColPnt[i].nMatch = -1;
		else
			ColPnt[i].nMatch = colsol[i];
	}

	delete colsol;
	delete rowsol;

	return cost;
}

/*****************************************************************************
/*	Name:		HungarianMatch
/*	Function:	Point matching using the Hungarian algorithm. 
/*				When nRowPnt==nColPnt, none points will be matched to outlier.
/*				When nRowPnt!=nColPnt, minimum number of outliers are introduced.
/*	Parameter:	costmat	-- Cost matrix, which is a square matrix with the 
/8						   dimension of max( nRowPnt, nColPnt ).
/*				RowPnt  -- The set of points corresponding to rows of costmat
/*				nRowPnt -- Number of points of RowPnt
/*				ColPnt	-- The set of points corresponding to columns of costmat
/*				nColPnt -- Number of points of ColPnt
/*	Return:		0 -- Succeed
/*****************************************************************************/
double HungarianMatch( double **costmat, MYPOINT *RowPnt, int nRowPnt, MYPOINT *ColPnt, int nColPnt)
{
	int	i, j;
	double cost;
	if( nRowPnt != nColPnt )
	{
		if( nRowPnt < nColPnt )
		{
			for( i=nRowPnt; i<nColPnt; i++ )
				for( j=0; j<nColPnt; j++ )
					costmat[i][j] = 0;	//The actual cost of matching to a dummy point does not matter.
		}
		else
		{
			for( i=0; i<nRowPnt; i++ )
				for( j=nColPnt; j<nRowPnt; j++ )
					costmat[i][j] = 0;
		}
	}
	cost = HungarianMatch( costmat, RowPnt, nRowPnt, ColPnt, nColPnt, std::max(nRowPnt, nColPnt) );
	return cost;
}

/*****************************************************************************
/*	Name:		SetMinMatch
/*	Function:	Set minimum number of matched pairs.
/*	Parameter:	costmat		-- Cost matrix
/*				PntRow		-- Points for row
/*				nPntRow		-- Number of rows of the cost matrix. 
/*				PntCol		-- Points for column.
/*				nPntCol		-- Number of columns of the cost matrix
/*				nMinMatch	-- Minimum number of matched pairs.
/*				nMinConf	-- Minimum matching confidence.
/*	Return:		0 -- Succeed
/*****************************************************************************/
int SetMinMatch( double **costmat, MYPOINT *PntRow, int nPntRow, MYPOINT *PntCol, int nPntCol, int nMinMatch, double nMinConf )
{
	// First get optional matches using the Hungarian algorithm
	HungarianMatch( costmat, PntRow, nPntRow, PntCol, nPntCol, std::max(nPntRow, nPntCol) );

	int		i, j;
	int		nPnt1, nPnt2;
	//Preserve matching pairs and matching confidence
	typedef	struct
	{
		int		nPnt1, nPnt2;
		double	nConf;
	}MATCH_PAIR;

	MATCH_PAIR	*MatchPair = new MATCH_PAIR[std::min(nPntRow, nPntCol)];
	int			nMatchPair = 0;
	for( i=0; i<nPntRow; i++ )
	{
		if( PntRow[i].nMatch < 0 )
			continue;
		MatchPair[nMatchPair].nPnt1 = i;
		MatchPair[nMatchPair].nPnt2 = PntRow[i].nMatch;
		MatchPair[nMatchPair].nConf = -costmat[i][PntRow[i].nMatch];
		nMatchPair++;
	}
	for( i=0; i<nPntRow; i++ )
		PntRow[i].nMatch = -1;
	for( i=0; i<nPntCol; i++ )
		PntCol[i].nMatch = -1;

	//Sort matching confidence
	int Max;
	for( i=0; i<nMatchPair; i++ )
	{
		Max = i;
		for( j=i+1; j<nMatchPair; j++ )
			if( MatchPair[Max].nConf < MatchPair[j].nConf )
				Max = j;
		if( Max != i )
		{
			MATCH_PAIR temp = MatchPair[i];
			MatchPair[i] = MatchPair[Max];
			MatchPair[Max] = temp;
		}
	}
	//Preserve at least nMinMacth matched pairs
	for( i=0; i<nMinMatch; i++ )
	{
		nPnt1 = MatchPair[i].nPnt1;
		nPnt2 = MatchPair[i].nPnt2;
		PntRow[nPnt1].nMatch = nPnt2;
		PntCol[nPnt2].nMatch = nPnt1;
	}
	for( i=nMinMatch; i<nMatchPair; i++ )
	{
		if( MatchPair[i].nConf < nMinConf )
			break;
		nPnt1 = MatchPair[i].nPnt1;
		nPnt2 = MatchPair[i].nPnt2;
		PntRow[nPnt1].nMatch = nPnt2;
		PntCol[nPnt2].nMatch = nPnt1;
	}

	delete MatchPair;

	return 0;
}


///////////////////////////////////////////////////////////////////////////////
// Functions related to input and output files
///////////////////////////////////////////////////////////////////////////////

/*****************************************************************************
/*	Name:		ReadSynFile
/*	Function:	Read the Chui-Rangarajan data
/*	Parameter:	fnSyn        -- SYN file name
/*				PntModel     -- Points of the model shape
/*				nPntModel    -- Number of points in the model shape
/*				PntDeform    -- Points of the deformed shape
/*				nPntDeform   -- Number of points in the deformed shape
/*				PntTruth     -- Ground truth
/*				nPntTruth    -- Number of points in the ground truth
/*	Return:		0 -- Succeed
/*****************************************************************************/
int ReadSynFile( char *fnSyn, MYPOINT *&PntModel, int &nPntModel, MYPOINT *&PntDeform, int &nPntDeform, MYPOINT *&PntTruth, int &nPntTruth )
{
	nPntModel = nPntDeform = nPntTruth = 0;
	FILE *fpSyn = fopen( fnSyn, "rt" );
	if( fpSyn == NULL )
		return -1;

	char buf[1024];
	//Get the number of points of the model shape
	fgets( buf, 1024, fpSyn );
	char *pos = strchr( buf, '=');
	sscanf( pos+1, "%d", &nPntModel );
	if( nPntModel <= 0 || nPntModel > 1000 )
	{
		printf( "The point number of the model shape is out the range of [1, 1000]\n" );
		return -1;
	}
	//Allocate memory
	PntModel = new MYPOINT[nPntModel];

	//Read points of the model shape
	int nID;
	for( int i=0; i<nPntModel; i++ )
	{
		if( fgets( buf, 1024, fpSyn ) == NULL )
			return -1;
		sscanf( buf, "%d%lf%lf", &nID, &PntModel[i].x, &PntModel[i].y );
		PntModel[i].nNumNeighbor = 0;
		PntModel[i].nTrueMatch = -1;
	}

	//Get the number of points of the deformed shape
	fgets( buf, 1024, fpSyn );
	pos = strchr( buf, '=');
	sscanf( pos+1, "%d", &nPntDeform );
	if( nPntDeform <= 0 || nPntDeform > 1000 )
	{
		printf( "The point number of the deformed shape is out the range of [1, 1000]\n" );
		return -1;
	}
	// Allocate memory
	PntDeform = new MYPOINT[nPntDeform];

	//Read points of the deformed shape
	for( int i=0; i<nPntDeform; i++ )
	{
		if( fgets( buf, 1024, fpSyn ) == NULL )
			return -1;
		sscanf( buf, "%d%lf%lf", &nID, &PntDeform[i].x, &PntDeform[i].y );
		PntDeform[i].nNumNeighbor = 0;
		PntDeform[i].nTrueMatch = -1;
	}

	//Get the number of points of the ground truth
	fgets( buf, 1024, fpSyn );
	pos = strstr( buf, "Truth");
	if( pos != NULL )
	{
		pos = strchr( buf, '=' );
		sscanf( pos+1, "%d", &nPntTruth );
		if( nPntTruth <= 0 || nPntTruth > 1000 )
		{
			printf( "The point number of the deformed shape is out the range of [1, 1000]\n" );
			return -1;
		}

		PntTruth = new MYPOINT[nPntTruth];

		//Read ground truth
		for( int i=0; i<nPntTruth; i++ )
		{
			if( fgets( buf, 1024, fpSyn ) == NULL )
				return -1;
			sscanf( buf, "%d%lf%lf", &nID, &PntTruth[i].x, &PntTruth[i].y );
			PntTruth[i].nNumNeighbor = 0;
		}
		fgets( buf, 1024, fpSyn );
	}

	//Get the corespondence between two point sets
	int nMatch;
	pos = strstr( buf, "Match");
	if( pos != NULL )
	{
		pos = strchr( buf, '=' );
		sscanf( pos+1, "%d", &nMatch );

		//Read point matching ground truth
		int nPoint1, nPoint2;
		for( int i=0; i<nMatch; i++ )
		{
			if( fgets( buf, 1024, fpSyn ) == NULL )
				return -1;
			sscanf( buf, "%d%d", &nPoint1, &nPoint2 );
			if( nPoint1<0 || nPoint1 >= nPntModel || nPoint2<0 || nPoint2>=nPntDeform )
				return -1;
			PntModel[nPoint1].nTrueMatch = nPoint2;
			PntDeform[nPoint2].nTrueMatch = nPoint1;
		}
	}
	fclose( fpSyn );
	return 0;
}

/*****************************************************************************
/*	Name:		ReadPointFile
/*	Function:	Input a set of points
/*	Parameter:	fnPoint -- POINT file name
/*				Pnt		-- Points
/*				nPnt	-- Number of points
/*	Return:		0 -- Succeed
/*****************************************************************************/
int ReadPointFile( char *fnPoint, MYPOINT *&Pnt, int &nPnt )
{
	nPnt = 0;
	FILE *fpPoint = fopen( fnPoint, "rt" );
	if( fpPoint == NULL )
		return -1;

	char buf[1024];
	//Get the number of points of the shape
	fgets( buf, 1024, fpPoint );
	char *pos = strchr( buf, '=');
	sscanf( pos+1, "%d", &nPnt );
	if( nPnt <= 0 || nPnt > 1000 )
	{
		printf( "The point number of the deformed shape is out the range of [1, 1000]\n" );
		return -1;
	}
	Pnt = new MYPOINT[nPnt];

	//Read points of the shape
	int nID;
	for( int i=0; i<nPnt; i++ )
	{
		if( fgets( buf, 1024, fpPoint ) == NULL )
			return -1;
		sscanf( buf, "%d%lf%lf", &nID, &Pnt[i].x, &Pnt[i].y );
		Pnt[i].nNumNeighbor = 0;
		Pnt[i].nTrueMatch = -1;
	}

	fclose( fpPoint );
	return 0;
}

/*****************************************************************************
/*	Name:		DumpMatch
/*	Function:	Dump the matching results
/*	Parameter:	fnMatch		-- Output file name
/*				PntModel	-- Points of the model shape
/*				nPntModel	-- Number of points of the model shape
/*				PntDeform	-- Points of the deformed shape
/*				nPntDeform	-- Number of points of the deformed shape
/*	Return:		0 -- Succeed
/*****************************************************************************/
int DumpMatch( char *fnMatch, MYPOINT *PntModel, int nPntModel, MYPOINT *PntDeform, int nPntDeform )
{
	FILE *fpMatch = fopen( fnMatch, "wt" );
	if( fpMatch == NULL )	return -1;

	int i;
	fprintf( fpMatch, "Model=%d\n", nPntModel );
	for( i=0; i<nPntModel; i++ )
		fprintf( fpMatch, "%3d\t%10f %10f\n", i, PntModel[i].x, PntModel[i].y );

	fprintf( fpMatch, "Deform=%d\n", nPntDeform );
	for( i=0; i<nPntDeform; i++ )
		fprintf( fpMatch, "%3d\t%10f %10f\n", i, PntDeform[i].x, PntDeform[i].y );

	int nMatch = 0;
	for( i=0; i<nPntModel; i++ )
	{
		if( PntModel[i].nMatch >= 0 )
			nMatch++;
	}
	fprintf( fpMatch, "Match=%d\n", nMatch );
	for( i=0; i<nPntModel; i++ )
	{
		if( PntModel[i].nMatch < 0 )	continue;
		fprintf( fpMatch, "%3d\t", i );
		fprintf( fpMatch, "%3d ", PntModel[i].nMatch );
		fprintf( fpMatch, "\n" );
	}
	fclose( fpMatch );
	return 0;
}

/*****************************************************************************
/*	Name:		DumpGraph
/*	Function:	Dump the graph representation
/*	Parameter:	fnGraph  -- Output file name
/*				Pnt      -- Point set
/*				nPnt     -- Number of points
/*	Return:		0 -- Succeed
/*****************************************************************************/
int DumpGraph( char *fnGraph, MYPOINT *Pnt, int nPnt )
{
	FILE *fpGraph = fopen( fnGraph, "wt" );
	if( fpGraph == NULL )	return -1;

	int i, j;
	fprintf( fpGraph, "Nodes=%d\n", nPnt );
	for( i=0; i<nPnt; i++ )
		fprintf( fpGraph, "%3d\t%10f %10f\n", i, Pnt[i].x, Pnt[i].y );

	int nEdge = 0;
	for( i=0; i<nPnt; i++ )
		nEdge += Pnt[i].nNumNeighbor;
	fprintf( fpGraph, "Edges=%d\n", nEdge );
	for( i=0; i<nPnt; i++ )
	{
		for( j=0; j<Pnt[i].nNumNeighbor; j++ )
			fprintf( fpGraph, "%3d\t%3d\n", i, Pnt[i].nNeighborList[j] );
	}
	fclose( fpGraph );
	return 0;
}

/******************************************************************************
/*	Name:		GetFileExt
/*	Function:	Get file extension
/*	Parameter:	FileName -- File name
/*	Return:		File extension
/*****************************************************************************/
std::string GetFileExtension ( std::string FileName )
{
    unsigned pos ;
    if ( (pos=FileName.rfind('.')) != std::string::npos )
        return FileName.substr( pos );
    else
        return "" ;
}

/******************************************************************************
/*	Name:		GetFileName
/*	Function:	Get file name without path from a file name string
/*	Parameter:	strFileName -- File name string
/*	Return:		File name
/*
/*****************************************************************************/
std::string GetFileName ( std::string strFileName )
{
    unsigned pos, l ;
    l = strFileName.length() ;
    if ( (pos=strFileName.rfind('/')) >= std::string::npos )
        return strFileName.substr( pos ) ;
    else
        return strFileName;
}

/******************************************************************************
/*	Name:		ChangeFileExt
/*	Function:	Change file extension
/*	Parameter:	dest -- Destination file name
/*				src  -- Source file name
/*				ext  -- File extension wanted 
/*	Return:		0  -- Correct
/*				-1 -- Error
/*****************************************************************************/
int ChangeFileExt ( char *dest, char *src, char *ext )
{
    strcpy ( dest, src ) ;
    for ( int i=strlen(dest)-1 ; i>=0 ; i-- )
    {
        if ( dest[i] == '.' )
        {
            dest[i] = '\0' ;
            break ;
        }
        else if ( dest[i] == '\\' )
            break ;
    }
    strcat ( dest, "." ) ;
    strcat ( dest, ext ) ;
    return 0 ;
}

/******************************************************************************
/*	Name:		GetFileNameWithoutExt
/*	Function:	Get file name from a string, removing path and file extension.
/*	Parameter:	strFileName -- File name string
/*	Return:		File name
/*****************************************************************************/
std::string GetFileNameWithoutExt ( std::string strFileName )
{
    unsigned pos, l ;
    std::string csFileName;
    l = strFileName.length() ;
    if ( (pos=strFileName.rfind('/')) >= std::string::npos )
        csFileName = strFileName.substr ( pos ) ;
    else
        csFileName = strFileName;

    pos = csFileName.rfind( '.' );
    if( pos >= std::string::npos )
        return csFileName.substr( 0, pos );
    else
        return csFileName;
}

///////////////////////////////////////////////////////////////////////////////
// Functions related to matrix operation
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
/*	Name:		MatrixMultiply
/*	Function:	Multiply two matrices
/*	Parameter:	a -- Matrix m*n
/*				b -- Matrix n*k
/*				c -- Matrix a*b, it is m*k
/*				m -- Dimension 
/*				n -- Dimension
/*				k -- Dimension
/*	Return:		0 -- Succeed	-1 -- Error
/*****************************************************************************/
double	MatrixMultiply( double *a, double *b, double *c, int m, int n, int k)
{
	int i, j, l;
	for( i=0; i<m; i++ )
	{
		for( j=0; j<k; j++ )
		{
			c[i*k+j] = 0;
			for( l=0; l<n; l++ )
				c[i*k+j] += a[i*n+l]*b[l*k+j];
		}
	}
	return 0;
}

/******************************************************************************
/*	Name:		Inv
/*	Function:	Inverse a matrix
/*	Parameter:	a -- Matrix, changed after inversion
/*				n -- Dimension of the matrix
/*	Return:		0 -- Succeed
/*				-1 -- Error
/*****************************************************************************/
int Inv(double a[], int n)
{//Invert Matrix 
	int *is,*js,i,j,k,l,u,v;
    double d,p;
    is=(int*)malloc(n*sizeof(int));
    js=(int*)malloc(n*sizeof(int));
    for (k=0; k<=n-1; k++)
      { d=0.0;
        for (i=k; i<=n-1; i++)
        for (j=k; j<=n-1; j++)
          { l=i*n+j; p=fabs(a[l]);
            if (p>d) { d=p; is[k]=i; js[k]=j;}
          }
        if (d+1.0==1.0)
          { 
			free(is); free(js); printf("err**not inv\n");
            return -1;
          }
        if (is[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=is[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (js[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+js[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        l=k*n+k;
        a[l]=1.0/a[l];
        for (j=0; j<=n-1; j++)
          if (j!=k)
            { u=k*n+j; a[u]=a[u]*a[l];}
        for (i=0; i<=n-1; i++)
          if (i!=k)
            for (j=0; j<=n-1; j++)
              if (j!=k)
                { u=i*n+j;
                  a[u]=a[u]-a[i*n+k]*a[k*n+j];
                }
        for (i=0; i<=n-1; i++)
          if (i!=k)
            { u=i*n+k; a[u]=-a[u]*a[l];}
      }
    for (k=n-1; k>=0; k--)
      { if (js[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=js[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (is[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+is[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
      }
    free(is); free(js);
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////
// Miscellaneous functions
/////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************************
/*	Name:		SortPoints
/*	Function:	Sort points after matching
/*	Parameter:	Pnt         -- Points
/*				Pnt2        -- Points after sorting
/*				PntMatch    -- Provide match correpondece
/*				nPnt        -- Number of points
/*	Return:		0 -- Succeed
/**************************************************************************************/
int SortPoints( MYPOINT *Pnt, MYPOINT *Pnt2, MYPOINT *PntMatch, int nPnt)
{
	int	nNewPnt = 0;
	int i; 
	for(i=0; i<nPnt; i++ )
	{
		if( PntMatch[i].nMatch == -1 )
			continue;
		Pnt2[nNewPnt] = Pnt[PntMatch[i].nMatch];
		nNewPnt++;
	}
	return 0;
}

/*************************************************************************************
/*	Name:		RemoveOutlier
/*	Function:	Remove outlier
/*	Parameter:	Pnt         -- Points
/*				Pnt2        -- Points after removing outlier
/*				nPnt        -- Number of points
/*	Return:		0 -- Succeed
/**************************************************************************************/
int	RemoveOutlier( MYPOINT *Pnt, MYPOINT *Pnt2, int nPnt )
{
	int i, nValid=0;
	for( i=0; i<nPnt; i++ )
	{
		if( Pnt[i].nMatch == -1 )
			continue;
		Pnt2[nValid] = Pnt[i];
		nValid++;
	}
	return 0;
}


/*************************************************************************************
/*	Name:		GetMedian
/*	Function:	This Quickselect routine is based on the algorithm described in
 *				"Numerical recipes in C", Second Edition,
 *				Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 *				This code was written by Nicolas Devillard - 1998. Public domain.
 /*				Warning: the routine will change the values in the array!!!
/*	Parameter:	arr   -- An array
/*				n     -- Number of elements in the array
/*	Return:		Median value
/**************************************************************************************/
double GetMedian(double *arr, int n)
{
#define ELEM_SWAP(a,b) { register double t=(a);(a)=(b);(b)=t; }
    int low, high ;
    int median;
    int middle, ll, hh;

    low = 0 ; high = n-1 ; median = (low + high) / 2;
    for (;;) {
        if (high <= low) /* One element only */
            return arr[median] ;

        if (high == low + 1) {  /* Two elements only */
            if (arr[low] > arr[high])
                ELEM_SWAP(arr[low], arr[high]) ;
            return arr[median] ;
        }

    /* Find median of low, middle and high items; swap into position low */
    middle = (low + high) / 2;
    if (arr[middle] > arr[high])    ELEM_SWAP(arr[middle], arr[high]) ;
    if (arr[low] > arr[high])       ELEM_SWAP(arr[low], arr[high]) ;
    if (arr[middle] > arr[low])     ELEM_SWAP(arr[middle], arr[low]) ;

    /* Swap low item (now in position middle) into position (low+1) */
    ELEM_SWAP(arr[middle], arr[low+1]) ;

    /* Nibble from each end towards middle, swapping items when stuck */
    ll = low + 1;
    hh = high;
    for (;;) {
        do ll++; while (arr[low] > arr[ll]) ;
        do hh--; while (arr[hh]  > arr[low]) ;

        if (hh < ll)
        break;

        ELEM_SWAP(arr[ll], arr[hh]) ;
    }

    /* Swap middle item (in position low) back into correct position */
    ELEM_SWAP(arr[low], arr[hh]) ;

    /* Re-set active partition */
    if (hh <= median)
        low = ll;
        if (hh >= median)
        high = hh - 1;
    }
}
