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

/////////////////////////////////////////////////////////////////////////
// File NAME:		Tools.h
// File Function:	Commnon functions for other modules
//
//				Developed by: Yefeng Zheng
//				First created: May, 2004
//			University of Maryland, College Park
/////////////////////////////////////////////////////////////////////////

#ifndef __TOOLS_H__
#define __TOOLS_H__

#include <string>
#include "PointMatch.h"

#ifndef BOOL
#define BOOL bool
#endif

#ifndef TRUE
#define TRUE true
#endif

#ifndef FALSE
#define FALSE false
#endif

#ifndef	PI
#define PI  3.1415926535
#endif

//Calculate the Euclidean distance between two points
double	GetDistance ( double x1, double y1, double x2, double y2 );
//Set edges in a graph, a simple definition of neighborhood
int		SetEdgeSimple( MYPOINT *Pnt, int nPnt, double **r_array, double Aver_Edge_Num );
//Set edges in a graph, another definition of neighborhood
int		SetEdgeNonUniformScale( MYPOINT *Pnt, int nPnt, double **r_array, double Aver_Edge_Num );
//Set edges in a graph. Normal points and outliers get different treatment.
int		SetEdgeSimple_Outlier( MYPOINT *Pnt, int nPnt, double **r_array, double E_Ave );
//Get file extension
std::string	GetFileExtension ( std::string FileName ) ;
//Get file name from a string
std::string	GetFileName ( std::string strFileName );
//Get file name (without extension) from a string
std::string	GetFileNameWithoutExt ( std::string strFileName );
//Change file extension
int		ChangeFileExt ( char *dest, char *src, char *ext );

//Get the median vaule of an array for data type 'double' with a quick algorithm
//Warning: it will change the array.
double	GetMedian(double *arr, int n);
//Inverse a matrix
int		Inv(double a[], int n);
//Matrix multiplication
double	MatrixMultiply( double *a, double *b, double *c, int m, int n, int k);
//Sort points for TPS deformation
int		SortPoints( MYPOINT *Pnt, MYPOINT *Pnt2, MYPOINT *PntMatch, int nPnt);
//Removing outliers after matching. Used before calculating the TPS deformation model.
int		RemoveOutlier( MYPOINT *Pnt, MYPOINT *Pnt2, int nPnt );
//Read *.syn file
int		ReadSynFile( char *fnSyn, MYPOINT *&PntModel, int &nPntModel, MYPOINT *&PntDeform, int &nPntDeform, MYPOINT *&PntTruth, int &nPntTruth );
//Read *.point file
int		ReadPointFile( char *fnPoint, MYPOINT *&Pnt, int &nPnt );
//Dump matching result
int		DumpMatch( char *fnMatch, MYPOINT *Pnt1, int nPnt1, MYPOINT *Pnt2, int nPnt2 );
//Dump graph for debugging
int		DumpGraph( char *fnGraph, MYPOINT *Pnt, int nPnt );
//Matching using the Hungarian algorithm, considering arbitary number of outliers
double	HungarianMatch( double **costmat, MYPOINT *Pnt1, int nPnt1, MYPOINT *Pnt2, int nPnt2, int nTotalPnt );
//Matching using the Hungarian algorithm, considering minimumm number of outliers
double	HungarianMatch( double **costmat, MYPOINT *Pnt1, int nPnt1, MYPOINT *Pnt2, int nPnt2 );
//Set minimum number of matching pair after relaxation labeling
int		SetMinMatch( double **costmat, MYPOINT *PntRow, int nPntRow, MYPOINT *PntCol, int nPntCol, int nMinMatch, double nMinConf );

#endif // __TOOLS_H__
