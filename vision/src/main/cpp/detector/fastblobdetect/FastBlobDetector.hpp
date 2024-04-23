/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef FASTBLOBDETECTOR_HPP
#define FASTBLOBDETECTOR_HPP

// Author: Matthias Scheutz
// Last modified: 07/01/2004
//              : 12/09/2010 (Evan Krause) reorganized into cohesive class, but didn't change any calculation details

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <log4cxx/logger.h>


//comments are fun
// uncomment the next line if OPENCV is used
#define OPENCV
#ifdef OPENCV
#include <opencv2/opencv.hpp>
#endif


//======== #DEFINES ===============
// this can be a most  32, otherwise the int arithmetic will not work
#define PARALLELCOLS   32
//======== END #DEFINES ===============


//========= TYPEDEFS ===============

typedef struct BlobInfo_s {
    int UL_x;
    int UL_y;
    int LR_x;
    int LR_y;
    int color;
    int colorID;
    std::string colorlabel;
    int C_x;
    int C_y;
    int R;
    int G;
    int B;
    float area;
} BlobInfo;

// the blob data structure

typedef struct BlobD_s {
    int xcg;
    int ycg;
    int area;
    int top;
    int left;
    int right;
    int bottom;
    double alpha;
    double beta;
    double shape;
    int rav; // the average of the blob color
    int bav;
    int gav;
    int rsq;
    int bsq;
    int gsq;
    float rstdev; // the stdev
    float bstdev;
    float gstdev;
    int colorID;
    int nummerged; // to keep track of the number of merged blobs
    struct LConnector_s *merge; // used for merging
    int indbl; // the index in the blob list of this blob
} BlobD;

typedef struct LConnector_s {
    struct BlobD_s *blob;
    struct LConnector_s *next;
} LConnector;

typedef struct ImageBundle_s {
    struct LConnector_s * bundles[PARALLELCOLS];
    int color;
} ImageBundle;

//========= END TYPEDEFS ===============

class FastBlobDetector {
public:

    typedef boost::shared_ptr<FastBlobDetector> Ptr;

    FastBlobDetector();
    ~FastBlobDetector();

    void initBlobDetection(int imgWidth, int imgHeight);

#ifdef OPENCV
    std::vector<BlobInfo> getBlobs(cv::Mat original, int startx, int starty, int width, int height);
    std::vector <BlobInfo> getMBlobs(cv::Mat newim, int startx, int starty, int width, int height);
#else
    std::vector <BlobInfo> getBlobs(int *original, int startx, int starty, int width, int height);
    std::vector <BlobInfo> getMBlobs(int *newim, int startx, int starty, int width, int height);
#endif
    void setColors(const int numcols, const std::string names[], const int* redLow, const int* redHigh, const int* greenLow, const int* greenHigh, const int* blueLow, const int* blueHigh);
    void setMinsMaxs(const int size, const int* passedMin, const int* passedMax);
    void addColor(const std::string name, const int redLow, const int redHigh,
        const int greenLow, const int greenHigh,
        const int blueLow, const int blueHigh,
        const int minSize, const int maxSize);
    const int getNumColors() const;
    void getColors(const int size, int redLow[], int redHigh[], int greenLow[], int greenHigh[], int blueLow[], int blueHigh[]) const;
    std::string getColorLabel(const int index) const;
    void getSizeMinsMaxs(const int size, int mins[], int maxs[]) const;

private:
    mutable boost::mutex blob_mutex;

    void *nofail_malloc(size_t size);
    bool inRange(int lower, int upper, int val);

    int numcolors;
    int pixcount; // = 0;

    int WIDTH; // = 320;
    int HEIGHT; // = 240;

    // uncomment the next line to prevent marking of blobs in image
#define MARKBLOB  //MZ:  FIXME:  potentially all of this MARKBLOB, MARKIT, MARKITM COULD BE CLEANED UP. 
    int MARKIT; // = 0;
    int MARKITM; // = 1;

#define MARKCOL 0
    int BMARKCOLB; // = 0;
    int BMARKCOLR; // = 0;
    int BMARKCOLG; // = 0;


#define MAXCOLS        32
    // the next two numbers will depend on the number of colors checked in parallel
    // the more colors are check the higher the space should be
    // if the system runs of out blobs, it will return -1
#define MAXBLOBS       10000
#define MAXLCONNECTORS 10000
    // NOTE: this has to be > 1, for color blobs

    // NOTE: this has to be > 1, for motion blobs
    int MINMBLOBSIZE; // = 40;
#define BACKGROUND     0x00ffffff;
#define CRESOLUTION    256

    // NOTE: this needs to be a power of 2 (>=1 and <= 256)
#define LEVELS         256

    // the threshold around a color dimension to detect motion
#define MBTHRESH        30
#define MRTHRESH        30
#define MGTHRESH        30

    // 32 different colors can be matched at the same time
    // these need to be predefined (and can be changed before matching)
    std::string colorlabels[32];
    int R_Colors[LEVELS];
    int B_Colors[LEVELS];
    int G_Colors[LEVELS];
    int Min_Sizes[LEVELS];
    int Max_Sizes[LEVELS];

    // these are the neighborhood thresholds for motion detection
    int MTableR[65536];
    int MTableG[65536];
    int MTableB[65536];

    // e.g., at 64 different color levels each level would cover several RGB values,
    //i.e., 0-3, 4-7, 8-12, .., 252-255
    // need to be assigned colors to each RGB dimension before running the algorithm
    int LevelMapping[CRESOLUTION];

    BlobD *blobs;
    BlobD *startblobs;
    LConnector *bundles;
    ImageBundle *image;

    // buffer to hold the old image for motion detection
    // MS: this should be done via MALLOC!!!
    char oldim[320 * 240 * 3];

	log4cxx::LoggerPtr logger;
};

#endif	//FASTBLOBDETECTOR_HPP
