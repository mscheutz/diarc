/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include "BlobDetection.hpp"
#include <functional>
#include <vector>

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


bool inRange( int lower, int upper, int val )
{
   bool result = false;
	if (lower > upper)
	{
		int temp = lower;
		lower = upper;
		upper = temp;
	}
   if ( (val >= lower) && (val <= upper) )
      result = true;

   return result;
}


void initializeAllBlobRanges(int R_LOW[], int R_HIGH[], int G_LOW[], int G_HIGH[], int B_LOW[], int B_HIGH[], int* R_ptr, int *G_ptr, int *B_ptr,int numcols) {

  // clear first:
  for (int c = 0; c < 32; c++) {
    for ( int q = 0; q < 256; q++ ) {
      R_ptr[q] = 0;
      G_ptr[q] = 0;
      B_ptr[q] = 0;
    }
  }
  

  // now set the relevant ones:
  for (int c = 0; c < numcols; c++) {
    for ( int q = 0; q < 256; q++ ) {
      if ( inRange(R_LOW[c], R_HIGH[c], q) )
         R_ptr[q] |= (1 << c);
      if ( inRange(G_LOW[c], G_HIGH[c], q) )
         G_ptr[q] |= (1 << c);
      if ( inRange(B_LOW[c], B_HIGH[c], q) )
         B_ptr[q] |= (1 << c);
    }
  }
  
  // print the colors
  //for ( int q = 0; q < 256; q++ )
  //printf("%d: %d %d %d\n",q,R_ptr[q],G_ptr[q],B_ptr[q]);

}


void initializeBlobRanges(int R_LOW_W, int R_HIGH_W, int R_LOW_D, int R_HIGH_D, int G_LOW_W, int G_HIGH_W, int G_LOW_D, int G_HIGH_D, int B_LOW_W, int B_HIGH_W, int B_LOW_D, int B_HIGH_D, int *R_ptr, int *G_ptr, int *B_ptr)
{
   for ( int q = 0; q < 256; q++ )
   {
      if ( (!inRange(R_LOW_W, R_HIGH_W, q)) && (!inRange(R_LOW_D, R_HIGH_D, q)) )
         R_ptr[q] = 0;
      else if ( (!inRange(R_LOW_W, R_HIGH_W, q)) && (inRange(R_LOW_D, R_HIGH_D, q)) )
         R_ptr[q] = 1;
      else if ( (inRange(R_LOW_W, R_HIGH_W, q)) && (!inRange(R_LOW_D, R_HIGH_D, q)) )
         R_ptr[q] = 2;
      else if ( (inRange(R_LOW_W, R_HIGH_W, q)) && (inRange(R_LOW_D, R_HIGH_D, q)) )          
         R_ptr[q] = 3;
   }

   for ( int q = 0; q < 256; q++ )
   {
      if ( !inRange(G_LOW_W, G_HIGH_W, q) && !inRange(G_LOW_D, G_HIGH_D, q) )
         G_ptr[q] = 0;

      else if ( !inRange(G_LOW_W, G_HIGH_W, q) && inRange(G_LOW_D, G_HIGH_D, q) )
         G_ptr[q] = 1;

      else if ( inRange(G_LOW_W, G_HIGH_W, q) && !inRange(G_LOW_D, G_HIGH_D, q) )
         G_ptr[q] = 2;

      else if ( inRange(G_LOW_W, G_HIGH_W, q) && inRange(G_LOW_D, G_HIGH_D, q) )
         G_ptr[q] = 3;
   }

   for ( int q = 0; q < 256; q++ )
   {
      if ( !inRange(B_LOW_W, B_HIGH_W, q) && !inRange(B_LOW_D, B_HIGH_D, q) )
         B_ptr[q] = 0;

      else if ( !inRange(B_LOW_W, B_HIGH_W, q) && inRange(B_LOW_D, B_HIGH_D, q) )
         B_ptr[q] = 1;

      else if ( inRange(B_LOW_W, B_HIGH_W, q) && !inRange(B_LOW_D, B_HIGH_D, q) )
         B_ptr[q] = 2;

      else if ( inRange(B_LOW_W, B_HIGH_W, q) && inRange(B_LOW_D, B_HIGH_D, q) )
         B_ptr[q] = 3;
   }
}

void initializeBlobRanges(int R_LOW, int R_HIGH, int G_LOW, int G_HIGH, int B_LOW, int B_HIGH, int *R_ptr, int *G_ptr, int *B_ptr)
{
   for ( int q = 0; q < 256; q++ )
   {
      if ( !inRange(R_LOW, R_HIGH, q) )
         R_ptr[q] = 0;
      else
         R_ptr[q] = 1;
   }

   for ( int q = 0; q < 256; q++ )
   {
      if ( !inRange(G_LOW, G_HIGH, q) )
         G_ptr[q] = 0;

      else
         G_ptr[q] = 1;
   }

   for ( int q = 0; q < 256; q++ )
   {
      if ( !inRange(B_LOW, B_HIGH, q) )
         B_ptr[q] = 0;

      else
         B_ptr[q] = 1;
   }
}

/*
int filterBlobsInBlobs(int numBlobs) {

   for ( int i = 0; i < numBlobs; i++ ) {
      for ( int j = 0; j < numBlobs; j++ ) {
         if ( (i!=j) && ((blobs+j)->xcg > (blobs+i)->left) && ((blobs+j)->xcg < (blobs+i)->right)
            && ((blobs+j)->ycg > (blobs+i)->top) && ((blobs+j)->ycg < (blobs+i)->bottom)
            && ((blobs+i)->area >= (blobs+j)->area) )
         {
            for (int k=j; k< numBlobs-1; k++)
               *(blobs+k)=*(blobs+k+1);
            numBlobs--;
            if (i > j) //if i is getting shifted
               i--;
            j--;
            break;
         }
      }
   }
   return numBlobs;
}

int mergeAllBlobs(int numBlobs)
{
	for (int i=1; i < numBlobs; i++)
	{
		if ((blobs+i)->left < (blobs)->left)
			blobs->left = (blobs+i)->left;
		if ((blobs+i)->right > (blobs)->right)
			blobs->right = (blobs+i)->right;
		if ((blobs+i)->top < (blobs)->top)
			blobs->top = (blobs+i)->top;
		if ((blobs+i)->bottom > (blobs)->bottom)
			blobs->bottom = (blobs+i)->bottom;
	}
	blobs->xcg = (blobs->right + blobs->left) / 2;
	blobs->ycg = (blobs->right + blobs->left)/2;
	blobs->area = (blobs->bottom - blobs->top) * (blobs->right - blobs->left);
	return 1;
}

CvRect mergeAllBlobsVector(vector < BlobD* > theBlobs)
{
	if (theBlobs.size()==0)
		return cvRect(0,0,1,1);

	CvRect r = cvRect(theBlobs[0]->left, theBlobs[0]->top, theBlobs[0]->right-theBlobs[0]->left, theBlobs[0]->bottom-theBlobs[0]->top);
	for (int i=1; i < theBlobs.size(); i++)
	{
		if (theBlobs[i]->left < r.x)
			r.x=theBlobs[i]->left;
		if (theBlobs[i]->right > (r.x+r.width))
         r.width =theBlobs[i]->right - r.x;
		if (theBlobs[i]->top < r.y)
         r.y=theBlobs[i]->top;
		if (theBlobs[i]->bottom > (r.y+r.height))
         r.height =theBlobs[i]->bottom - r.y;
	}
	return r;
}

int mergeBlobs(int numBlobs)
{
   for ( int i = 0; i < numBlobs; i++ ) {
      for ( int j = 0; j < numBlobs; j++ ) {
         if ( ((blobs+i)->xcg < (blobs+j)->right) && ((blobs+i)->xcg > (blobs+j)->left)
            && ( (blobs+i)->bottom + 0.2*((blobs+i)->right - (blobs+i)->left)  > (blobs+j)->top ) && ((blobs+i)->top < (blobs+j)->top) )
         {
            if ((blobs+i)->left > (blobs+j)->left)
               (blobs+i)->left = (blobs+j)->left;
            if ((blobs+i)->right < (blobs+j)->right)
               (blobs+i)->right = (blobs+j)->right;
            (blobs+i)->bottom = (blobs+j)->bottom;
            (blobs+i)->xcg=(int)( (blobs+i)->left+(blobs+i)->right )/2;
            (blobs+i)->ycg=(int)( (blobs+i)->top+(blobs+i)->bottom )/2;
            (blobs+i)->area += (blobs+j)->area;
            for (int k=j; k< numBlobs-1; k++)
               *(blobs+k)=*(blobs+k+1);
            numBlobs--;
            i--;
            break;
         }
      }
   }

   return numBlobs;
}

int filterBlobsByShape(int numBlobs)
{
   for ( int j = 0; j < numBlobs; j++ ) {
      if ( ((blobs+j)->right - (blobs+j)->left) > 1.2*((blobs+j)->bottom - (blobs+j)->top)
         || ((blobs+j)->bottom - (blobs+j)->top) > 2*((blobs+j)->right - (blobs+j)->left)
         || ((blobs+j)->right - (blobs+j)->left) <= 0
         || ((blobs+j)->bottom - (blobs+j)->top) <= 0 )
      {
         for (int k=j; k< numBlobs-1; k++)
            *(blobs+k)=*(blobs+k+1);
         numBlobs--;
         j--;
      }
   }
}

void drawBlobs(IplImage* img, int numBlobs)
{
   for (int i=0; i < numBlobs; i++) {
      cvRectangle(img, cvPoint( (blobs+i)->left, (blobs+i)->top ), cvPoint( (blobs+i)->right, (blobs+i)->bottom ), CV_RGB(255, 255, 0), 1);
   }
}
*/
