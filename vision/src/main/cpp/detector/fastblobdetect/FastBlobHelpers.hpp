/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef BLOBHELPERS_HPP
#define BLOBHELPERS_HPP

#include "Blob.hpp"
#include <functional>
#include <vector>
#include <opencv2/opencv.hpp>

struct BlobD_compareIsAbove :
   public std::binary_function<const BlobD*, const BlobD*, bool>
{
   bool operator() (const BlobD* top, const BlobD* bottom) const
   {
      return (top->top < bottom->top);
   };
};

struct BlobD_compareIsLarger :
   public std::binary_function<const BlobD*, const BlobD*, bool>
{
   bool operator() (const BlobD* small, const BlobD* big) const
   {
      return (small->area < big->area);
   };
};

bool inRange( int lower, int upper, int val );

void initializeBlobRanges(int R_LOW_W, int R_HIGH_W, int R_LOW_D, int R_HIGH_D, int G_LOW_W, int G_HIGH_W, int G_LOW_D, int G_HIGH_D, int B_LOW_W, int B_HIGH_W, int B_LOW_D, int B_HIGH_D, int *R_ptr, int *G_ptr, int *B_ptr);

void initializeBlobRanges(int R_LOW, int R_HIGH, int G_LOW, int G_HIGH, int B_LOW, int B_HIGH, int *R_ptr, int *G_ptr, int *B_ptr);

void initializeAllBlobRanges(int R_LOW[], int R_HIGH[], int G_LOW[], int G_HIGH[], int B_LOW[], int B_HIGH[], int* R_ptr, int *G_ptr, int *B_ptr,int numcols);


/*
int filterBlobsInBlobs(int numBlobs);

int mergeAllBlobs(int numBlobs);

CvRect mergeAllBlobsVector(vector<BlobD*> theBlobs);

int mergeBlobs(int numBlobs);

int filterBlobsByShape(int numBlobs);

void drawBlobs(IplImage* img, int numBlobs);
*/
#endif

