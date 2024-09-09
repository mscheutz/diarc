/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <boost/thread/locks.hpp>

#include "FastBlobDetector.hpp"

FastBlobDetector::FastBlobDetector()
: numcolors(0),
pixcount(0),
WIDTH(320),
HEIGHT(240),
MARKIT(0),
MARKITM(1),
BMARKCOLB(0),
BMARKCOLR(0),
BMARKCOLG(0),
MINMBLOBSIZE(40),
logger(log4cxx::Logger::getLogger("diarc.detector.fastblobdetect.FastBlobDetector")) {
}

FastBlobDetector::~FastBlobDetector() {
  //cleanup malloced stuff in initBlobDetection!!!
  free(blobs);
  free(bundles);
  free(image);
}

void* FastBlobDetector::nofail_malloc(size_t size) {
  void *r;
  if (NULL == (r = malloc(size))) {
    ////printf("Out of memory (malloc).");
  }
  return (r);
}

// IMPORTANT: call this first to initialize the blob detection system

void FastBlobDetector::initBlobDetection(int imgWidth, int imgHeight) {
  //scoped lock
  boost::unique_lock<boost::mutex> lock(blob_mutex);

  WIDTH = imgWidth;
  HEIGHT = imgHeight;
  
  // init things to 0
  for (int q = 0; q < 256; q++) {
    R_Colors[q] = 0;
    G_Colors[q] = 0;
    B_Colors[q] = 0;
  }
  // init things to empty
  for (int c = 0; c < 32; ++c) {
    colorlabels[c] = "";
  }

  int i, j;
  int k = 256 / LEVELS;
  for (i = 0; i < LEVELS; i++)
    for (j = i * k; j < i * k + k; j++)
      LevelMapping[j] = i;

  // create the default motion tables (basically +-1 the current entry)
  for (i = 0; i < 256; i++) {
    if (i < MRTHRESH) {
      for (j = 0; j <= i + MRTHRESH; j++) {
        int index = i * 256 + j;
        MTableR[i * 256 + j] = 0;
      }
      for (j = i + MRTHRESH + 1; j < 256; j++) {
        int index = i * 256 + j;
        MTableR[index] = 1;
      }
    } else if (i >= 256 - MRTHRESH) {
      for (j = 0; j < i - MRTHRESH; j++) {
        int index = i * 256 + j;
        MTableR[index] = 1;
      }
      for (j = i - MRTHRESH; j < 256; j++) {
        int index = i * 256 + j;
        MTableR[i * 256 + j] = 0;
      }
    } else {
      for (j = 0; j < i - MRTHRESH; j++) {
        int index = i * 256 + j;
        MTableR[index] = 1;
      }
      for (j = i - MRTHRESH; j <= i + MRTHRESH; j++) {
        int index = i * 256 + j;
        MTableR[index] = 0;
      }
      for (j = i + MRTHRESH + 1; j < 256; j++) {
        int index = i * 256 + j;
        MTableR[index] = 1;
      }
    }
    if (i < MBTHRESH) {
      for (j = 0; j <= i + MBTHRESH; j++) {
        int index = i * 256 + j;
        MTableB[i * 256 + j] = 0;
      }
      for (j = i + MBTHRESH + 1; j < 256; j++) {
        int index = i * 256 + j;
        MTableB[index] = 1;
      }
    } else if (i >= 256 - MBTHRESH) {
      for (j = 0; j < i - MBTHRESH; j++) {
        int index = i * 256 + j;
        MTableB[index] = 1;
      }
      for (j = i - MBTHRESH; j < 256; j++) {
        int index = i * 256 + j;
        MTableB[i * 256 + j] = 0;
      }
    } else {
      for (j = 0; j < i - MBTHRESH; j++) {
        int index = i * 256 + j;
        MTableB[index] = 1;
      }
      for (j = i - MBTHRESH; j <= i + MBTHRESH; j++) {
        int index = i * 256 + j;
        MTableB[index] = 0;
      }
      for (j = i + MBTHRESH + 1; j < 256; j++) {
        int index = i * 256 + j;
        MTableB[index] = 1;
      }
    }
    if (i < MGTHRESH) {
      for (j = 0; j <= i + MGTHRESH; j++) {
        int index = i * 256 + j;
        MTableB[i * 256 + j] = 0;
      }
      for (j = i + MGTHRESH + 1; j < 256; j++) {
        int index = i * 256 + j;
        MTableB[index] = 1;
      }
    } else if (i >= 256 - MGTHRESH) {
      for (j = 0; j < i - MGTHRESH; j++) {
        int index = i * 256 + j;
        MTableG[index] = 1;
      }
      for (j = i - MGTHRESH; j < 256; j++) {
        int index = i * 256 + j;
        MTableG[i * 256 + j] = 0;
      }
    } else {
      for (j = 0; j < i - MGTHRESH; j++) {
        int index = i * 256 + j;
        MTableG[index] = 1;
      }
      for (j = i - MGTHRESH; j <= i + MGTHRESH; j++) {
        int index = i * 256 + j;
        MTableG[index] = 0;
      }
      for (j = i + MGTHRESH + 1; j < 256; j++) {
        int index = i * 256 + j;
        MTableG[index] = 1;
      }
    }
  }


  blobs = (BlobD *) nofail_malloc(sizeof (BlobD) * MAXBLOBS); // allocate memory for the max. number of blobs for each color dimension
  bundles = (LConnector *) nofail_malloc(sizeof (LConnector) * MAXLCONNECTORS); // allocate memory for the max. number of bundles
  image = (ImageBundle *) nofail_malloc(sizeof (ImageBundle) * WIDTH * HEIGHT); // allocate memory for the max. number of bundles
  startblobs = blobs;
}

void FastBlobDetector::setMinsMaxs(const int size, const int* passedMin, const int* passedMax) {
  //scoped lock
  boost::unique_lock<boost::mutex> lock(blob_mutex);

  for (int i = 0; i < size; i++) {
    Min_Sizes[i] = passedMin[i];
    Max_Sizes[i] = passedMax[i];
  }
}

void FastBlobDetector::setColors(const int numcols, const std::string colorNames[], const int* redLow, const int* redHigh, const int* greenLow, const int* greenHigh, const int* blueLow, const int* blueHigh) {
  //scoped lock
  boost::unique_lock<boost::mutex> lock(blob_mutex);

  numcolors = numcols;

  // init blob ranges (will clear out anything already in them, so don't
  //    worry about "bleedover")

  // clear first:
  for (int q = 0; q < 256; q++) {
    R_Colors[q] = 0;
    G_Colors[q] = 0;
    B_Colors[q] = 0;
  }
  for (int c = 0; c < 32; ++c) {
    colorlabels[c] = "";
  }


  // now set the relevant ones:
  for (int c = 0; c < numcolors; c++) {
    colorlabels[c] = colorNames[c];
    //printf("blob colors: %d %d %d to %d %d %d\n", redLow[c], greenLow[c], blueLow[c], redHigh[c], greenHigh[c], blueHigh[c]);
    for (int q = 0; q < 256; q++) {
      if (inRange(redLow[c], redHigh[c], q))
        R_Colors[q] |= (1 << c);
      if (inRange(greenLow[c], greenHigh[c], q))
        G_Colors[q] |= (1 << c);
      if (inRange(blueLow[c], blueHigh[c], q))
        B_Colors[q] |= (1 << c);
    }
  }

  // print the colors
  //    for ( int q = 0; q < 256; q++ )
  //    printf("%d: %d %d %d\n",q,R_Colors[q],G_Colors[q],B_Colors[q]);

  //    for (int i = 0; i < numcolors; ++i) {
  //        printf("names[%d] = %s\n", i, colorlabels[i].c_str());
  //    }

}

void FastBlobDetector::addColor(const std::string name, const int redLow, const int redHigh,
        const int greenLow, const int greenHigh,
        const int blueLow, const int blueHigh,
        const int minSize, const int maxSize) {
  if (numcolors >= 32) {
    LOG4CXX_WARN(logger, "[FastBlobDetector::addColor] too many colors! Can not add any more.\n");
    return;
  }

  // set the new color:
  colorlabels[numcolors] = name;
  for (int q = 0; q < 256; q++) {
    if (inRange(redLow, redHigh, q))
      R_Colors[q] |= (1 << numcolors);
    if (inRange(greenLow, greenHigh, q))
      G_Colors[q] |= (1 << numcolors);
    if (inRange(blueLow, blueHigh, q))
      B_Colors[q] |= (1 << numcolors);
  }

  Min_Sizes[numcolors] = minSize;
  Max_Sizes[numcolors] = maxSize;

  ++numcolors;
}

const int FastBlobDetector::getNumColors() const {
  return numcolors;
}

void FastBlobDetector::getColors(const int size, int redLow[], int redHigh[], int greenLow[], int greenHigh[], int blueLow[], int blueHigh[]) const {
  //scoped lock
  boost::unique_lock<boost::mutex> lock(blob_mutex);

  if (size != numcolors) {
    LOG4CXX_WARN(logger, "[getColors] array sizes do not match!");

    if (size > numcolors) {
      return;
    }
  }

  //     for ( int q = 0; q < 256; q++ )
  //    printf("%d: %d %d %d\n",q,R_Colors[q],G_Colors[q],B_Colors[q]);

  for (int c = 0; c < numcolors; c++) {

    bool redPreviousBitOn = false;
    bool greenPreviousBitOn = false;
    bool bluePreviousBitOn = false;

    for (int q = 0; q < 256; q++) {

      //get red colors
      if (!redPreviousBitOn && (R_Colors[q] & (1 << c))) {
        redLow[c] = q;
        redPreviousBitOn = true;
      } else if (redPreviousBitOn && (R_Colors[q] ^ (1 << c))) {
        redHigh[c] = q - 1;
        redPreviousBitOn = false;
      } else if ((255 == q) && redPreviousBitOn) {
        redHigh[c] = 255;
      }

      //get green colors
      if (!greenPreviousBitOn && (G_Colors[q] & (1 << c))) {
        //going from "off" bit to "on bit means found low range
        greenLow[c] = q;
        greenPreviousBitOn = true;
      } else if (greenPreviousBitOn && (G_Colors[q] ^ (1 << c))) {
        //going from "on" bit to "off" bit means found high range
        greenHigh[c] = q - 1;
        greenPreviousBitOn = false;
      } else if ((255 == q) && greenPreviousBitOn) {
        //green goes to max range
        greenHigh[c] = 255;
      }

      //get blue colors
      if (!bluePreviousBitOn && (B_Colors[q] & (1 << c))) {
        //going from "off" bit to "on bit means found low range
        blueLow[c] = q;
        bluePreviousBitOn = true;
      } else if (bluePreviousBitOn && (B_Colors[q] ^ (1 << c))) {
        blueHigh[c] = q - 1;
        bluePreviousBitOn = false;
      } else if ((255 == q) && bluePreviousBitOn) {
        blueHigh[c] = 255;
      }
    }

    //printf("blob colors: %d %d %d to %d %d %d\n", redLow[c], greenLow[c], blueLow[c], redHigh[c], greenHigh[c], blueHigh[c]);
  }

}

std::string FastBlobDetector::getColorLabel(const int index) const {
  return colorlabels[index];
}

void FastBlobDetector::getSizeMinsMaxs(const int size, int mins[], int maxs[]) const {
  //scoped lock
  boost::unique_lock<boost::mutex> lock(blob_mutex);

  if (size != numcolors) {
    LOG4CXX_WARN(logger, "[getSizeMinsMaxs] array sizes do not match!");

    if (size > numcolors) {
      return;
    }
  }

  for (int i = 0; i < size; i++) {
    mins[i] = Min_Sizes[i];
    maxs[i] = Max_Sizes[i];
  }
}

bool FastBlobDetector::inRange(int lower, int upper, int val) {
  bool result = false;
  if (lower > upper) {
    int temp = lower;
    lower = upper;
    upper = temp;
  }
  if ((val >= lower) && (val <= upper))
    result = true;

  return result;
}


// ===============================  BLOB DETECTION =========================================

// returns the number of detected blobs in all "numcolors" within the bounding box
#ifdef OPENCV

std::vector <BlobInfo> FastBlobDetector::getBlobs(cv::Mat original, int startx, int starty, int width, int height) {
#else

std::vector <BlobInfo> FastBlobDetector::getBlobs(int *original, int startx, int starty, int width, int height) {
#endif

  //scoped lock
  boost::unique_lock<boost::mutex> lock(blob_mutex);

  int x, y, wu, w, k;

  //YW: from blobdetect
  /*CBlobResult blobs;
  CBlob blobWithLargestArea, blobWithLowestArea, blob;
  CvPoint upperLeft, lowerRight;
  CvSize size;
  CvPoint tmp[MAX_BLOBS];
  bool marker[MAX_BLOBS];
  CvPoint blobs2BDrawn[MAX_BLOBS];
  CvPoint thisFrameBlobs[MAX_BLOBS];
  CvPoint* blobMemPtr;

  int i_tmp = 0;
  int i_marker = 0;
  int i_blobs2BDrawn = 0;
  int i_thisFrameBlobs = 0;
  int counter;
   */
  // --------------------


  //  printf("Image dims are  %d %d %d %d %d %d\n",startx,starty,width,height,WIDTH,HEIGHT);

  //    for ( int q = 0; q < 256; q++ )
  //printf("--> %d %d \n",q,LevelMapping[q]);

  int red, green, blue;

  ImageBundle *currentimage, *leftimage;
  currentimage = leftimage = image + WIDTH * starty + startx;
  BlobD *nextfreeblob = blobs;
  LConnector *nextfreebundle = bundles;
  int totBlobs = 0;
  int foundblob = false;
  //std::cout << "please tell me!" << std::endl;
  // first make sure that numcolors is not greater than 32
  numcolors = (numcolors > PARALLELCOLS ? PARALLELCOLS : numcolors);
  //  std::cout << "a" << std::endl;
  // check the first pixel at 0,0
#ifdef OPENCV
  //  std::cout << "b" << std::endl;
  uchar *baseptr = &((uchar *) (original.data + WIDTH * starty * 3))[startx * 3];
  //  std::cout << "c" << std::endl;
#else
  //  std::cout << "d" << std::endl;
  int *baseptr = original + WIDTH * starty * 3 + startx * 3;
  //  std::cout << "e" << std::endl;
#endif
  //  std::cout << "f" << std::endl;
  red = baseptr[2];
  green = baseptr[1];
  blue = baseptr[0];
  //  std::cout << "is it something I did?!" << std::endl;
  //  std::cout << red << " " << green << " " << blue << std::endl;

  //  std::cout << LevelMapping[red] << " " << LevelMapping[green] << " " << LevelMapping[blue] << std::endl;
  // check simultaneously whether the pixels is preset in up to 32 color ranges
  if (foundblob = R_Colors[LevelMapping[red]] & G_Colors[LevelMapping[green]] &
          B_Colors[LevelMapping[blue]]) {

    //std::cout << "wha?" << std::endl;
    currentimage->color = foundblob;
#ifdef MARKBLOB
    if (MARKIT) {
      //        printf("Found pixel %d at 0,0\n",k);
      baseptr[2] = BMARKCOLR;
      baseptr[1] = BMARKCOLG;
      baseptr[0] = BMARKCOLB;
    }
#endif
    int shift = 1;
    //	std::cout << "checkpoint 1" << std::endl;
    for (k = 0; k < numcolors; k++) {
      // check if the k-th color was detected
      if (foundblob & shift) {
        //printf("Found pixel %d at 0,0\n",k);
        nextfreebundle->blob = nextfreeblob;
        nextfreebundle->next = NULL;
        currentimage->bundles[k] = nextfreebundle;
        // initialize the blob
        nextfreeblob->merge = nextfreebundle++;
        nextfreeblob->xcg = startx;
        nextfreeblob->ycg = starty;
        nextfreeblob->area = 1;
        nextfreeblob->top = starty;
        nextfreeblob->left = startx;
        nextfreeblob->right = startx;
        nextfreeblob->bottom = starty;
        //nextfreeblob->alpha = -1;
        //nextfreeblob->beta = -1;
        //nextfreeblob->shape = -1;
        nextfreeblob->colorID = k;
        nextfreeblob->rav = red; // the average of the blob color
        nextfreeblob->gav = green;
        nextfreeblob->bav = blue;
        nextfreeblob->rsq = red*red;
        nextfreeblob->gsq = green*green;
        nextfreeblob->bsq = blue*blue;
        nextfreeblob->nummerged = 1;
        nextfreeblob->indbl = totBlobs++;
        nextfreeblob++;
      }
      shift = (shift << 1);
    }
  } else {
    //printf("NO PIXEL at %0,0\n");
  }
  // set the image color of the current pixel
  currentimage->color = foundblob;
  baseptr += 3;

  // now check every pixel but the first one of the first row separately
  for (x = startx + 1; x < startx + width; x++) {
    // adjust the current and left pixel pointers
    leftimage = currentimage++;
    red = baseptr[2];
    green = baseptr[1];
    blue = baseptr[0];
    /*if ((green == 0) && (blue == 0))
            std::cout << "found a blob pixel" << std::endl;*/
    //printf("G %d %d    %d%d\n",green,LevelMapping[baseptr[1]],blue,LevelMapping[baseptr[2]]);
    if (w = R_Colors[LevelMapping[baseptr[2]]] &
            G_Colors[LevelMapping[baseptr[1]]] &
            B_Colors[LevelMapping[baseptr[0]]]) {
      //      printf("Found pixel at 0,%d\n",x);
#ifdef MARKBLOB
      if (MARKIT) {
        //               printf("Found pixel %d at 0,0\n",k);
        baseptr[2] = BMARKCOLR;
        baseptr[1] = BMARKCOLG;
        baseptr[0] = BMARKCOLB;
      }
#endif
      int shift = 1;
      // NOTE: foundblob is the color of the left pixels
      for (k = 0; k < numcolors; k++) {
        if (w & shift) {
          //printf("Found pixel %d at 0,%d\n",k,x);
          // check if the left pixel had the same color, then we must have a bundle and a blob
          if (foundblob & shift) {
            //printf("Found blob to the left\n");
            // set the current pixel to the bundle to the left pixel and get the blob to which the left bundle points
            currentimage->bundles[k] = leftimage->bundles[k];
            BlobD *current = (currentimage->bundles[k])->blob;
            // currentimage->bundles[k]->next = NULL;   NOT NECESSARY WILL NOT BE USED
            current->xcg += x;
            current->ycg += starty;
            // extend the boundary box to the right
            current->right = x;
            current->area++;
            current->rav += red;
            current->bav += blue;
            current->gav += green;
            current->rsq += red*red;
            current->bsq += blue*blue;
            current->gsq += green*green;
          }// start a new blob
          else {
            nextfreebundle->blob = nextfreeblob;
            nextfreebundle->next = NULL;
            currentimage->bundles[k] = nextfreebundle;
            nextfreeblob->merge = nextfreebundle++;
            nextfreeblob->xcg = x;
            nextfreeblob->ycg = starty;
            nextfreeblob->area = 1;
            nextfreeblob->top = starty;
            nextfreeblob->left = x;
            nextfreeblob->right = x;
            nextfreeblob->bottom = starty;
            //nextfreeblob->alpha = -1;
            //nextfreeblob->beta = -1;
            //nextfreeblob->shape = -1;
            nextfreeblob->colorID = k;
            nextfreeblob->rav = red; // the average of the blob color
            nextfreeblob->gav = green;
            nextfreeblob->bav = blue;
            nextfreeblob->rsq = red*red;
            nextfreeblob->gsq = green*green;
            nextfreeblob->bsq = blue*blue;
            nextfreeblob->nummerged = 1;
            nextfreeblob->indbl = totBlobs++;
            nextfreeblob++;
          }
        }
        shift = (shift << 1);
      }
    } else {
      //printf("NO PIXEL at 0,%d\n",x);
    }
    baseptr += 3;
    currentimage->color = foundblob = w;
  }
  // now check all rows starting with the 2nd
  for (y = starty + 1; y < starty + height; y++) {
    //check the leftmost pixel separately
    //printf("%d  ",currentimage);
    //    currentimage += WIDTH - width + 1 + startx;
    currentimage = image + WIDTH * y + startx;
    //printf("%d\n",currentimage);
    //  red = baseptr[2];
    //  green = baseptr[1];
    //  blue = baseptr[0];
    /*if ((green == 0) && (blue == 0))
            std::cout << "found a blob pixel" << std::endl;*/
#ifdef OPENCV
    baseptr = &((uchar *) (original.data + WIDTH * y * 3))[startx * 3];
#else
    baseptr = original + WIDTH * y * 3 + startx * 3;
#endif
    red = baseptr[2];
    green = baseptr[1];
    blue = baseptr[0];
    if (foundblob = R_Colors[LevelMapping[baseptr[2]]] &
            G_Colors[LevelMapping[baseptr[1]]] & B_Colors[LevelMapping[baseptr[0]]]) {
      currentimage->color = foundblob;
#ifdef MARKBLOB
      if (MARKIT) {
        //printf("Found pixel %d at 0,0\n",k);
        baseptr[2] = BMARKCOLR;
        baseptr[1] = BMARKCOLG;
        baseptr[0] = BMARKCOLB;
      }
#endif
      int shift = 1;
      // get the upper pickels and its color
      ImageBundle *Bupper = currentimage - WIDTH;
      wu = Bupper->color;
      for (k = 0; k < numcolors; k++) {
        // check if the k-th color was detected
        if (foundblob & shift) {
          //printf("Found pixel at %d,0\n",y);
          // check if there was a blob of that color above
          if (wu & shift) {
            //printf("Found blob above\n");
            currentimage->bundles[k] = Bupper->bundles[k];
            BlobD *current = (currentimage->bundles[k])->blob;
            current->xcg += x;
            current->ycg += y;
            // set the new bottom boundary
            current->bottom = y;
            current->area++;
            current->rav += red;
            current->bav += blue;
            current->gav += green;
            current->rsq += red*red;
            current->bsq += blue*blue;
            current->gsq += green*green;
          } else {
            nextfreebundle->blob = nextfreeblob;
            nextfreebundle->next = NULL;
            currentimage->bundles[k] = nextfreebundle;
            nextfreeblob->merge = nextfreebundle++;
            nextfreeblob->xcg = startx;
            nextfreeblob->ycg = y;
            nextfreeblob->area = 1;
            nextfreeblob->top = y;
            nextfreeblob->left = startx;
            nextfreeblob->right = startx;
            nextfreeblob->bottom = y;
            //nextfreeblob->alpha = -1;
            //nextfreeblob->beta = -1;
            //nextfreeblob->shape = -1;
            nextfreeblob->colorID = k;
            nextfreeblob->rav = red; // the average of the blob color
            nextfreeblob->gav = green;
            nextfreeblob->bav = blue;
            nextfreeblob->rsq = red*red;
            nextfreeblob->gsq = green*green;
            nextfreeblob->bsq = blue*blue;
            nextfreeblob->nummerged = 1;
            nextfreeblob->indbl = totBlobs++;

            std::vector <BlobInfo> errorBlobs;
            BlobInfo errorBlob;
            errorBlob.area = -1;
            errorBlobs.push_back(errorBlob);
            if (totBlobs == MAXBLOBS)
              return errorBlobs;
            nextfreeblob++;
          }
        }
        shift = (shift << 1);
      }
    } else {
      //printf("NO PIXEL at %d,0\n",y);
    }
    currentimage->color = foundblob;
    baseptr += 3;

    // now examine the other pixels in this row starting with the 2nd
    for (x = startx + 1; x < startx + width; x++) {
      // adjust the current and left pixel pointers
      leftimage = currentimage++;
      red = baseptr[2];
      green = baseptr[1];
      blue = baseptr[0];
      //printf("%d Colors are %d %d,%d\n",pixcount++,red,green,blue);

      /*if ((green == 0) && (blue == 0))
            std::cout << "found a blob pixel" << std::endl;*/
      if (w = R_Colors[LevelMapping[baseptr[2]]] & G_Colors[LevelMapping[baseptr[1]]] & B_Colors[LevelMapping[baseptr[0]]]) {
        //        printf("Found pixel at %d,%d\n",y,x);
#ifdef MARKBLOB
        if (MARKIT) {
          //printf("Found pixel %d at 0,0\n",k);
          baseptr[2] = BMARKCOLR;
          baseptr[1] = BMARKCOLG;
          baseptr[0] = BMARKCOLB;
        }
#endif
        currentimage->color = foundblob;
        int shift = 1;
        // get the upper pickels and its color
        ImageBundle *Bupper = currentimage - WIDTH;
        wu = Bupper->color;
        for (k = 0; k < numcolors; k++) {
          // check if the k-th color was detected
          if (w & shift) {
            //printf("Found pixel %d at %d,%d\n",k,y,x);
            // check if the color was found to the left
            if (foundblob & shift) {
              //printf("Found blob to the left\n");
              // set the current pixel to the bundle to the left pixel and get the blob to which the left bundle points
              currentimage->bundles[k] = leftimage->bundles[k];
              BlobD *current = (currentimage->bundles[k])->blob;
              current->xcg += x;
              current->ycg += y;
              if (x > current->right)
                current->right = x;
              current->area++;
              current->rav += red;
              current->bav += blue;
              current->gav += green;
              current->rsq += red*red;
              current->bsq += blue*blue;
              current->gsq += green*green;

              // now check if we need to merge
              if ((wu & shift) && ((currentimage->bundles[k])->blob !=
                      (Bupper->bundles[k])->blob)) {
                //printf("MERGING ===========++++++++++++++++++++++++++++++++++==============\n");
                // MERGING CODE...
                LConnector *toL, *fromL;
                toL = fromL = Bupper->bundles[k]->blob->merge;
                BlobD *toB, *fromB;
                toB = fromB = toL->blob;
                int sizeupper = toB->nummerged;
                int min, max;
                min = max = current->nummerged;
                //printf("COMPARING sizes ... current %d, upper %d",current->nummerged,sizeupper);
                if (sizeupper > min) {
                  //printf("CURRENT->MERGE == CURRENTIMAGE->BUNDLES[k] %d",current->merge ==currentimage->bundles[k]);
                  fromL = current->merge;
                  fromB = current;
                  max = sizeupper;
                } else {
                  toL = current->merge;
                  toB = current;
                  min = sizeupper;
                }
                //printf("   DONE\n");
                // copy over the blob information
                toB->xcg += fromB->xcg;
                toB->ycg += fromB->ycg;
                if (toB->left > fromB->left)
                  toB->left = fromB->left;
                if (toB->right < fromB->right)
                  toB->right = fromB->right;
                if (toB->top > fromB->top)
                  toB->top = fromB->top;
                if (toB->bottom < fromB->bottom)
                  toB->bottom = fromB->bottom;
                toB->area += fromB->area;
                toB->rav += fromB->rav;
                toB->gav += fromB->gav;
                toB->bav += fromB->bav;
                toB->rsq += fromB->rsq;
                toB->gsq += fromB->gsq;
                toB->bsq += fromB->bsq;
                //printf("Rearranging pointers...\ntoB: %d, fromL %d\n",toB,fromL);
                //printf("UpperL %d, UpperB %d, UpperBMerge %d\n",Bupper->bundles[k],Bupper->bundles[k]->blob,Bupper->bundles[k]->blob->merge);
                //printf("CurrentL %d, CurrentB %d, CurrentBMerge %d\n",currentimage->bundles[k],currentimage->bundles[k]->blob,currentimage->bundles[k]->blob->merge);
                // set the blob to the LConnector list of the min blob
                toB->merge = fromL;
                // point every LConnector in the LConnector list of the min blob to the toBlob
                fromL->blob = toB;
                //printf("CONNECTING ...");
                while (fromL->next != NULL) {
                  fromL = fromL->next;
                  fromL->blob = toB;
                }
                // connect the two LConnector lists
                fromL -> next = toL;
                // adjust the size of the LConnector list in the toBlob
                toB->nummerged = min + max;
                //printf("   DONE\n");
                //printf("UpperL %d, UpperB %d, UpperBMerge %d\n",Bupper->bundles[k],Bupper->bundles[k]->blob,Bupper->bundles[k]->blob->merge);
                //printf("CurrentL %d, CurrentB %d, CurrentBMerge %d\n",currentimage->bundles[k],currentimage->bundles[k]->blob,currentimage->bundles[k]->blob->merge);

                // move the last blob to fromB in the blob list to keep the list condensed
                /* TODO:  FIX FILL GAPS IN BLOB STRUCTURES
                   totBlobs--;
                   nextfreeblob--;
                   Blob *temp = blobs + (fromB->indbl);
                 *(blobs + (fromB->indbl)) = *nextfreeblob;
                 *nextfreeblob = *temp;
                 */
                // FOR NOW: use -1 in index to indicate the blob is not valid
                fromB->indbl = -1;
                //printf("SIZE: %d\n",toB->area);
              }
            }// check if there was a blob of that color above
            else if (wu & shift) {
              //printf("Found blob above\n");
              currentimage->bundles[k] = Bupper->bundles[k];
              BlobD *current = (currentimage->bundles[k])->blob;
              current->xcg += x;
              current->ycg += y;
              if (y > current->bottom)
                current->bottom = y;
              current->area++;
              current->rav += red;
              current->bav += blue;
              current->gav += green;
              current->rsq += red*red;
              current->bsq += blue*blue;
              current->gsq += green*green;
            } else {
              nextfreebundle->blob = nextfreeblob;
              nextfreebundle->next = NULL;
              currentimage->bundles[k] = nextfreebundle;
              nextfreeblob->merge = nextfreebundle++;
              nextfreeblob->xcg = x;
              nextfreeblob->ycg = y;
              nextfreeblob->area = 1;
              nextfreeblob->top = y;
              nextfreeblob->left = x;
              nextfreeblob->right = x;
              nextfreeblob->bottom = y;
              //nextfreeblob->alpha = -1;
              //nextfreeblob->beta = -1;
              //nextfreeblob->shape = -1;
              nextfreeblob->colorID = k;
              nextfreeblob->rav = red; // the average of the blob color
              nextfreeblob->gav = green;
              nextfreeblob->bav = blue;
              nextfreeblob->rsq = red*red;
              nextfreeblob->gsq = green*green;
              nextfreeblob->bsq = blue*blue;
              nextfreeblob->nummerged = 1;
              nextfreeblob->indbl = totBlobs++;

              std::vector <BlobInfo> errorBlobs;
              BlobInfo errorBlob;
              errorBlob.area = -1;
              errorBlobs.push_back(errorBlob);
              if (totBlobs == MAXBLOBS)
                return errorBlobs;
              nextfreeblob++;
            }
          }
          shift = (shift << 1);
        }
      } else {
        //printf("NO PIXEL at %d,%d\n",y,x);
      }
      baseptr += 3;
      currentimage->color = foundblob = w;
    }
  }

  // need to get the blobs out and sort them, first eliminate blobs that are too small
  int i = 0, j = 0;
  totBlobs--;
  BlobD *b = blobs;
  while (i <= totBlobs) {
    //printf("TotBlobs %i, %d\n",i,(blobs+i)->area);
    if ((b->area < Min_Sizes[b->colorID]) || (b->area > Max_Sizes[b->colorID]) || (b->indbl < 0)) {
      *b = *(blobs + totBlobs--);
    } else {
      // compute the area of the blob and estimate the stdev
      double a = b->area;

      double ravTemp = b->rav / a;
      double gavTemp = b->gav / a;
      double bavTemp = b->bav / a;
      //printf("a: %f\n", a);
      //printf("b->gav %f\n", b->gav);
      b->rstdev = sqrt((b->rsq / (ravTemp * ravTemp)) - 2 * (b->rav) / ravTemp + a);
      b->gstdev = sqrt((b->gsq / (gavTemp * gavTemp)) - 2 * (b->gav) / gavTemp + a);
      b->bstdev = sqrt((b->bsq / (bavTemp * bavTemp)) - 2 * (b->bav) / bavTemp + a);

      b->xcg = (int) (b->xcg / a);
      b->ycg = (int) (b->ycg / a);

      b->rav = (int) (b->rav / a);
      b->gav = (int) (b->gav / a);
      b->bav = (int) (b->bav / a);

      //std::cout << "R_AVG: " << b->rav << " G_AVG: " << b->gav << " B_AVG: " << b->bav << std::endl;
      //std::cout << "R_SDV: " << b->rstdev << " G_SDV: " << b->gstdev << " B_SDV: " << b->bstdev << std::endl;
      //b->rstdev = sqrt((b->rav)*(b->rav)*a/(a-1));
      //b->gstdev = sqrt((b->gav)*(b->gav)*a/(a-1));
      //b->bstdev = sqrt((b->bav)*(b->bav)*a/(a-1));
      b++;
      i++;
    }
  }
  totBlobs++;

  /*TODO: sort the blobs by size
  int which[numcolors];
  // now compute the centroids and sort them according to area, largest first
  for(i=0;i<totBlobs;i++) {
    BlobD *b = blobs+i;

    b->xcg /= b->area;
    b->ycg /= b->area;
    for (j=totBlobs; j>i; j--)
      if(blobs[j].area > blobs[j-1].area) {
        BlobD tmp = blobs[j];
        blobs[j] = blobs[j-1];
        blobs[j-1] = tmp;
      }
  }
   */

  std::vector <BlobInfo> returnBlobs;
  for (i = 0; i < totBlobs; i++) {
    cv::Point pt1((blobs + i)->left, (blobs + i)->top);
    cv::Point pt2((blobs + i)->right, (blobs + i)->bottom);
    BlobInfo blob;
    blob.UL_x = (blobs + i)->left;
    blob.UL_y = (blobs + i)->top;
    blob.LR_x = (blobs + i)->right;
    blob.LR_y = (blobs + i)->bottom;
    blob.C_x = (blob.UL_x + blob.LR_x) / 2;
    blob.C_y = (blob.UL_y + blob.LR_y) / 2;
    blob.colorID = (blobs + i)->colorID;
    blob.colorlabel = colorlabels[(blobs + i)->colorID];
    blob.area = (blobs + i)->area; // this is the actual area, not the bounding box area
    blob.R = (blobs + i)->rav;
    blob.G = (blobs + i)->gav;
    blob.B = (blobs + i)->bav;
    blob.color = blob.B + blob.G * 256 + blob.R * 65536;


    returnBlobs.push_back(blob);

    //std::cout << "\nBlob " << i << " area:  " << blob.area;
    // std::cout << "\nBLOB DETECT:  R_AVG: " << (blobs+i)->rav << " G_AVG: " << (blobs+i)->gav << " B_AVG: " << (blobs+i)->bav << std::endl;
    //printf("Color ID: %d\n",(blobs+i)->colorID);
    //printf("Blob %d: ",i+1);
    //printf("p1->x %d  ",(blobs+i)->left);
    //printf("p1->y %d  ",(blobs+i)->top);
    //printf("p2->x %d  ",(blobs+i)->right);
    //printf("p2->y %d\n",(blobs+i)->bottom);
  }


  // YW: from blob detect
  /*for (int i = 0; i < returnBlobs.size (); i++) {
    blob = blobs.GetBlob (i);
    upperLeft.x = (int) blob.MinX ();
    upperLeft.y = (int) blob.MinY ();
    lowerRight.x = (int) blob.MaxX ();
    lowerRight.y = (int) blob.MaxY ();
    if ((lowerRight.x-upperLeft.x != width) &&
        (lowerRight.y - upperLeft.y != height)) {
      tmp[i_tmp++] = upperLeft;
      tmp[i_tmp++] = lowerRight;
    }
  }


  // resolve overlapping blobs
  for (int i = 0; i < i_tmp; i+=2) {
    for (int j = 0; j < i_tmp; j+=2) {
      if (isAContainedInB (tmp[i], tmp[i+1], tmp[j], tmp[j+1])) {
        tmp[i] = tmp[j];
        tmp[i+1] = tmp[j+1];
      } else if (isBContainedInA (tmp[i], tmp[i+1], tmp[j], tmp[j+1])) {
        tmp[j] = tmp[i];
        tmp[j+1] = tmp[i+1];
      } else if (isAOverlapsB (tmp[i], tmp[i+1], tmp[j], tmp[j+1])) {
        if (tmp[i].x <= tmp[j].x)
          tmp[j].x = tmp[i].x;
        if (tmp[j].x <= tmp[i].x)
          tmp[i].x = tmp[j].x;
	
        if (tmp[i].y <= tmp[j].y)
          tmp[j].y = tmp[i].y;
        if (tmp[j].y <= tmp[i].y)
          tmp[i].y = tmp[j].y;
	
        if (tmp[i+1].x >= tmp[j+1].x)
          tmp[j+1].x = tmp[i+1].x;
        if (tmp[i+1].x <= tmp[j+1].x)
          tmp[i+1].x = tmp[j+1].x;
	
        if (tmp[i+1].y >= tmp[j+1].y)
          tmp[j+1].y = tmp[i+1].y;
        if (tmp[i+1].y <= tmp[j+1].y)
          tmp[i+1].y = tmp[j+1].y;
      }
    }
  }
/*
  //cvRectangle (color, cvPoint(235, 125), cvPoint (250, 135), CV_RGB (100, 200, 50), 1, 8, 0);
  // Remove duplicate blobs & copy to thisFrameBlobs array
  for (int i = 0; i < i_tmp; i+=2) {
    if (i_tmp > 0) {
      if (i == 0) {
        //cvRectangle (color, tmp[i], tmp[i+1], CV_RGB (R, G, B), 1, 8, 0);
        //storeBlobs (tmp[i], tmp[i+1], R, G, B, isSecondary);
        thisFrameBlobs[i_thisFrameBlobs++] = tmp[i];
        thisFrameBlobs[i_thisFrameBlobs++] = tmp[i+1];
      } else if ((tmp[i].x != tmp[i-2].x ||
                 tmp[i+1].x != tmp[i-1].x ||
                   tmp[i].y != tmp[i-2].y ||
                  tmp[i+1].y != tmp[i-1].y) &&
                 i > 0) {
        //cvRectangle (color, tmp[i], tmp[i+1], CV_RGB (R, G, B), 1, 8, 0);
        //storeBlobs (tmp[i], tmp[i+1], R, G, B, isSecondary);
        thisFrameBlobs[i_thisFrameBlobs++] = tmp[i];
        thisFrameBlobs[i_thisFrameBlobs++] = tmp[i+1];
      }
    }
  }
  
  // reset marker field
  for (int i = 0; i < MAX_BLOBS; i++)
    marker[i] = false;

  // reset blobs2BDrawn
  i_blobs2BDrawn = 0;
  
  if (R == 255 && G == 0 && B == 0 && !isSecondary) {
    counter = Rcounter1;
    blobMemPtr = R_blobMem1;
  } else if (G == 255 && R == 0 && B == 0 && !isSecondary) {
    counter = Gcounter1;
    blobMemPtr = G_blobMem1;
  } else if (B == 255 && R == 0 && G == 0 && !isSecondary) {
    counter = Bcounter1;
    blobMemPtr = B_blobMem1;
  } else if (R == 255 && G == 0 && B == 255 && !isSecondary) {
    counter = Pcounter1;
    blobMemPtr = P_blobMem1;
  } else if (R == 255 && G == 0 && B == 0 && isSecondary) {
    counter = Rcounter2;
    blobMemPtr = R_blobMem2;
  } else if (G == 255 && R == 0 && B == 0 && isSecondary) {
    counter = Gcounter2;
    blobMemPtr = G_blobMem2;
  } else if (B == 255 && R == 0 && G == 0 && isSecondary) {
    counter = Bcounter2;
    blobMemPtr = B_blobMem2;
  } else if (R == 255 && G == 0 && B == 255 && isSecondary) {
    counter = Pcounter2;
    blobMemPtr = P_blobMem2;
  }

  




  // -----------------------------------------

  // all non-split blobs from current frame are added to drawing buffer
  for (int i = 0; i < i_thisFrameBlobs; i+=2) {
    if (!marker[i]) {
      blobs2BDrawn[i_blobs2BDrawn++] = thisFrameBlobs[i];
      blobs2BDrawn[i_blobs2BDrawn++] = thisFrameBlobs[i+1];
    }
  }
  
  for(int i = 0; i < i_blobs2BDrawn; i+=2) {
    clearStorage (R, G, B, isSecondary);
    storeBlobs (blobs2BDrawn[i], blobs2BDrawn[i+1], R, G, B, isSecondary);
  }
  

  
  std::vector<BlobInfo> blobInfo;
  BlobInfo blobArray[i_blobs2BDrawn/2];
  for (int i = 0; i < i_blobs2BDrawn/2; i++) {
    blobArray[i].UL_x = blobs2BDrawn[i*2].x;
    blobArray[i].UL_y = blobs2BDrawn[i*2].y;
    blobArray[i].LR_x = blobs2BDrawn[i*2+1].x;
    blobArray[i].LR_y = blobs2BDrawn[i*2+1].y;
    blobArray[i].C_x = (blobArray[i].UL_x + blobArray[i].LR_x) / 2;;
    blobArray[i].C_y = (blobArray[i].UL_y + blobArray[i].LR_y) / 2;;
    blobArray[i].color = 65536*R+256*G+B;
    blobArray[i].R = R;
    blobArray[i].G = G;
    blobArray[i].B = B;
    blobArray[i].area = (blobArray[i].LR_x - blobArray[i].UL_x)
   * (blobArray[i].LR_y - blobArray[i].UL_y);
      
    blobInfo.push_back (blobArray[i]);
  }
   */

  return returnBlobs;
}


//==================================== MOTION DETECTION ====================================

// returns the number of detected motion blobs within the bounding box
#ifdef OPENCV

std::vector <BlobInfo> FastBlobDetector::getMBlobs(cv::Mat newim, int startx, int starty, int
        width, int height) {
#else

std::vector <BlobInfo> FastBlobDetector::getMBlobs(int *newim, int startx, int starty, int width, int height) {
#endif

  //scoped lock
  boost::unique_lock<boost::mutex> lock(blob_mutex);

  int x, y, wu, w, k, motionR, motionB, motionG;

  ImageBundle *currentimage, *leftimage;
  currentimage = leftimage = image + WIDTH * starty + startx;
  BlobD *nextfreeblob = blobs;
  LConnector *nextfreebundle = bundles;
  int totBlobs = 0;
  int foundblob = false;
  //printf("starting... %d, %d, %d\n",sizeof(ImageBundle),image,currentimage);
  // check the first pixel at 0,0
  //std::cout << "checkpoint 1" << std::endl;

  std::cout << "DANGER! THIS WILL DIE IF IN USARSIM!? (oldim is size for camera, not usarsim)" << std::endl;
#ifdef OPENCV
  uchar *baseptrold = &((uchar *) (oldim + WIDTH * starty * 3))[startx * 3];
  uchar *baseptrnew = &((uchar *) (newim.data + WIDTH * starty * 3))[startx * 3];
#else
  int *baseptrold = oldim + WIDTH * starty * 3 + startx * 3;
  int *baseptrnew = newim + WIDTH * starty * 3 + startx * 3;
#endif
  motionR = baseptrnew[2];
  motionR = baseptrold[2] | (motionR << 8);
  baseptrold[2] = baseptrnew[2];
  motionG = baseptrnew[1];
  motionG = baseptrold[1] | (motionG << 8);
  baseptrold[1] = baseptrnew[1];
  motionB = baseptrnew[0];
  motionB = baseptrold[0] | (motionB << 8);
  baseptrold[0] = baseptrnew[0];
  //printf("R: %d  ",motionR);
  //printf("G: %d  ",motionG);
  //printf("B: %d\n",motionB);
  //printf("RESULT: %d\n",MTableR[motionR] & MTableG[motionG] & MTableB[motionB]);
  // check simultaneously whether the pixels is preset in up to 32 color ranges
  if (foundblob = MTableR[motionR] & MTableG[motionG] & MTableB[motionB]) {
    //printf("MBlobYES+++++++++++++++++++++++++++++++++++++++++++++\n");
    currentimage->color = foundblob;
#ifdef MARKBLOB
    if (MARKITM) {
      baseptrnew[2] = MARKCOL;
      baseptrnew[1] = MARKCOL;
      baseptrnew[0] = MARKCOL;
    }
#endif
    nextfreebundle->blob = nextfreeblob;
    nextfreebundle->next = NULL;
    currentimage->bundles[0] = nextfreebundle;
    // initialize the blob
    nextfreeblob->merge = nextfreebundle++;
    nextfreeblob->xcg = startx;
    nextfreeblob->ycg = starty;
    nextfreeblob->area = 1;
    nextfreeblob->top = starty;
    nextfreeblob->left = startx;
    nextfreeblob->right = startx;
    nextfreeblob->bottom = starty;
    //nextfreeblob->alpha = -1;
    //nextfreeblob->beta = -1;
    //nextfreeblob->shape = -1;
    nextfreeblob->colorID = foundblob;
    nextfreeblob->nummerged = 1;
    nextfreeblob->indbl = totBlobs++;
    nextfreeblob++;
  } else {
    //printf("NO PIXEL at %0,0\n");
  }
  // set the image color of the current pixel
  //std::cout << "checkpoint 2" << std::endl;

  baseptrold += 3;
  baseptrnew += 3;
  currentimage->color = foundblob;
  // now check every pixel but the first one of the first row separately
  for (x = startx + 1; x < startx + width; x++) {
    // adjust the current and left pixel pointers
    leftimage = currentimage++;
    motionR = baseptrnew[2];
    motionR = baseptrold[2] | (motionR << 8);
    baseptrold[2] = baseptrnew[2];
    motionG = baseptrnew[1];
    motionG = baseptrold[1] | (motionG << 8);
    baseptrold[1] = baseptrnew[1];
    motionB = baseptrnew[0];
    motionB = baseptrold[0] | (motionB << 8);
    baseptrold[0] = baseptrnew[0];
    //  //printf("R: %d  ",motionR);
    //printf("G: %d  ",motionG);
    //printf("B: %d\n",motionB);
    //printf("RESULT: %d\n",MTableR[motionR] | MTableG[motionG] | MTableB[motionB]);
    if (w = MTableR[motionR] & MTableG[motionG] & MTableB[motionB]) {
#ifdef MARKBLOB
      if (MARKITM) {
        baseptrnew[2] = MARKCOL;
        baseptrnew[1] = MARKCOL;
        baseptrnew[0] = MARKCOL;
      }
#endif
      if (foundblob) {
        //printf("Found blob to the left\n");
        // set the current pixel to the bundle to the left pixel and get the blob to which the left bundle points
        currentimage->bundles[0] = leftimage->bundles[0];
        BlobD *current = (currentimage->bundles[0])->blob;
        current->xcg += x;
        current->ycg += starty;
        // extend the boundary box to the right
        current->right = x;
        current->area++;
      }// start a new blob
      else {
        nextfreebundle->blob = nextfreeblob;
        nextfreebundle->next = NULL;
        currentimage->bundles[0] = nextfreebundle;
        nextfreeblob->merge = nextfreebundle++;
        nextfreeblob->xcg = x;
        nextfreeblob->ycg = starty;
        nextfreeblob->area = 1;
        nextfreeblob->top = starty;
        nextfreeblob->left = x;
        nextfreeblob->right = x;
        nextfreeblob->bottom = starty;
        //nextfreeblob->alpha = -1;
        //nextfreeblob->beta = -1;
        //nextfreeblob->shape = -1;
        nextfreeblob->colorID = w;
        nextfreeblob->nummerged = 1;
        nextfreeblob->indbl = totBlobs++;
        nextfreeblob++;
      }
    } else {
      //printf("NO PIXEL at 0,%d\n",x);
    }

    baseptrold += 3;
    baseptrnew += 3;
    currentimage->color = foundblob = w;
    //    //printf("X=%d\n",x);
  }

  // now check all rows starting with the 2nd
  //std::cout << "checkpoint 3" << std::endl;

  for (y = starty + 1; y < starty + height; y++) {
    //check the leftmost pixel separately
    //printf("%d  ",currentimage);
    //    currentimage += WIDTH - width + 1 + startx;
    currentimage = image + WIDTH * y + startx;

#ifdef OPENCV
    baseptrold = &((uchar *) (oldim + WIDTH * y * 3))[startx * 3];
    baseptrnew = &((uchar *) (newim.data + WIDTH * y * 3))[startx * 3];
#else
    baseptrold = oldim + WIDTH * y * 3 + startx * 3;
    baseptrnew = newim + WIDTH * y * 3 + startx * 3;
#endif
    //printf("%d\n",currentimage);
    motionR = baseptrnew[2];
    motionR = baseptrold[2] | (motionR << 8);
    baseptrold[2] = baseptrnew[2];
    motionG = baseptrnew[1];
    motionG = baseptrold[1] | (motionG << 8);
    baseptrold[1] = baseptrnew[1];
    motionB = baseptrnew[0];
    motionB = baseptrold[0] | (motionB << 8);
    baseptrold[0] = baseptrnew[0];


    // check simultaneously whether the pixels is preset in up to 32 color ranges
    if (foundblob = MTableR[motionR] & MTableG[motionG] & MTableB[motionB]) {
#ifdef MARKBLOB
      if (MARKITM) {
        baseptrnew[2] = MARKCOL;
        baseptrnew[1] = MARKCOL;
        baseptrnew[0] = MARKCOL;
      }
#endif

      //printf("MBlob!!! y=%d\n",y);
      //printf("%d   %d",currentimage,currentimage-WIDTH);
      //printf("    %d\n",image + WIDTH*y + startx);
      // get the upper pickels and its color
      ImageBundle *Bupper = currentimage - WIDTH;
      wu = Bupper->color;
      //printf("After BUPPER\n");

      if (wu) {
        //printf("In WU bundles=%d\n",Bupper->bundles[0]);
        currentimage->bundles[0] = Bupper->bundles[0];
        BlobD *current = (currentimage->bundles[0])->blob;
        //printf("Blob from ABOVE\n");
        current->xcg += x;
        current->ycg += y;
        current->bottom = y;
        current->area++;
      } else {
        nextfreebundle->blob = nextfreeblob;
        nextfreebundle->next = NULL;
        currentimage->bundles[0] = nextfreebundle;
        nextfreeblob->merge = nextfreebundle++;
        nextfreeblob->xcg = startx;
        nextfreeblob->ycg = y;
        nextfreeblob->area = 1;
        nextfreeblob->top = y;
        nextfreeblob->left = startx;
        nextfreeblob->right = startx;
        nextfreeblob->bottom = y;
        nextfreeblob->colorID = foundblob;
        nextfreeblob->nummerged = 1;
        nextfreeblob->indbl = totBlobs++;
        std::vector <BlobInfo> errorBlobs;
        BlobInfo errorBlob;
        errorBlob.area = -1;
        errorBlobs.push_back(errorBlob);
        if (totBlobs == MAXBLOBS)
          return errorBlobs;
        nextfreeblob++;
      }
    } else {
      //printf("NO PIXEL at %d,0\n",y);
    }
    currentimage->color = foundblob;
    baseptrold += 3;
    baseptrnew += 3;

    //printf("Y=%d\n",y);
    // now examine the other pixels in this row starting with the 2nd

    for (x = startx + 1; x < startx + width; x++) {
      // adjust the current and left pixel pointers
      leftimage = currentimage++;
      motionR = baseptrnew[2];
      motionR = baseptrold[2] | (motionR << 8);
      baseptrold[2] = baseptrnew[2];
      motionG = baseptrnew[1];
      motionG = baseptrold[1] | (motionG << 8);
      baseptrold[1] = baseptrnew[1];
      motionB = baseptrnew[0];
      motionB = baseptrold[0] | (motionB << 8);
      baseptrold[0] = baseptrnew[0];
      // check simultaneously whether the pixels is preset in up to 32 color ranges

      if (w = MTableR[motionR] & MTableG[motionG] & MTableB[motionB]) {
#ifdef MARKBLOB
        if (MARKITM) {
          baseptrnew[2] = MARKCOL;
          baseptrnew[1] = MARKCOL;
          baseptrnew[0] = MARKCOL;
        }
#endif

        //printf("MBlob\n");
        currentimage->color = foundblob;
        // get the upper pickels and its color
        ImageBundle *Bupper = currentimage - WIDTH;
        wu = Bupper->color;
        if (foundblob) {
          //printf("Found blob to the left\n");
          // set the current pixel to the bundle to the left pixel and get the blob to which the left bundle points
          currentimage->bundles[0] = leftimage->bundles[0];
          //printf("1111111111111111");
          BlobD *current = (currentimage->bundles[0])->blob;
          current->xcg += x;
          current->ycg += y;
          if (x > current->right)
            current->right = x;
          current->area++;
          // now check if we need to merge
          if (wu && ((currentimage->bundles[0])->blob != (Bupper->bundles[0])->blob)) {
            //printf("MERGING ===========++++++++++++++++++++++++++++++++++==============\n");
            // MERGING CODE...
            LConnector *toL, *fromL;
            toL = fromL = Bupper->bundles[0]->blob->merge;
            BlobD *toB, *fromB;
            toB = fromB = toL->blob;
            int sizeupper = toB->nummerged;
            int min, max;
            min = max = current->nummerged;
            //printf("COMPARING sizes ... current %d, upper %d",current->nummerged,sizeupper);
            if (sizeupper > min) {
              //printf("CURRENT->MERGE == CURRENTIMAGE->BUNDLES[0] %d",current->merge == currentimage->bundles[0]);
              fromL = current->merge;
              fromB = current;
              max = sizeupper;
            } else {
              toL = current->merge;
              toB = current;
              min = sizeupper;
            }
            //printf("   DONE\n");
            // copy over the blob information
            toB->xcg += fromB->xcg;
            toB->ycg += fromB->ycg;
            if (toB->left > fromB->left)
              toB->left = fromB->left;
            if (toB->right < fromB->right)
              toB->right = fromB->right;
            if (toB->top > fromB->top)
              toB->top = fromB->top;
            if (toB->bottom < fromB->bottom)
              toB->bottom = fromB->bottom;
            toB->area += fromB->area;
            toB->merge = fromL;
            // point every LConnector in the LConnector list of the min blob to the toBlob
            fromL->blob = toB;
            //printf("CONNECTING ...");
            while (fromL->next != NULL) {
              fromL = fromL->next;
              fromL->blob = toB;
            }
            // connect the two LConnector lists
            fromL -> next = toL;
            // adjust the size of the LConnector list in the toBlob
            toB->nummerged = min + max;
            // FOR NOW: use -1 in index to indicate the blob is not valid
            fromB->indbl = -1;
            //printf("SIZE: %d\n",toB->area);
          }
        }// check if there was a blob of that color above
        else if (wu) {
          //printf("Found blob above?????\n");
          currentimage->bundles[0] = Bupper->bundles[0];
          BlobD *current = (currentimage->bundles[0])->blob;
          current->xcg += x;
          current->ycg += y;
          if (y > current->bottom)
            current->bottom = y;
          current->area++;
        } else {
          nextfreebundle->blob = nextfreeblob;
          nextfreebundle->next = NULL;
          currentimage->bundles[0] = nextfreebundle;
          nextfreeblob->merge = nextfreebundle++;
          nextfreeblob->xcg = x;
          nextfreeblob->ycg = y;
          nextfreeblob->area = 1;
          nextfreeblob->top = y;
          nextfreeblob->left = x;
          nextfreeblob->right = x;
          nextfreeblob->bottom = y;
          nextfreeblob->colorID = w;
          nextfreeblob->nummerged = 1;
          nextfreeblob->indbl = totBlobs++;
          std::vector <BlobInfo> errorBlobs;
          BlobInfo errorBlob;
          errorBlob.area = -1;
          errorBlobs.push_back(errorBlob);
          if (totBlobs == MAXBLOBS)
            return errorBlobs;
          nextfreeblob++;
        }
      } else {
        //printf("NO PIXEL at %d,%d\n",y,x);
      }
      baseptrold += 3;
      baseptrnew += 3;
      currentimage->color = foundblob = w;
    }
  }
  //std::cout << "checkpoint 4" << std::endl;
  // need to get the blobs out and sort them, first eliminate blobs that are too small
  int i = 0, j = 0;
  totBlobs--;
  BlobD *b = blobs;
  while (i <= totBlobs) {
    //printf("TotBlobs %i, %d\n",i,(blobs+i)->area);
    if ((b->area < MINMBLOBSIZE) || (b->indbl < 0))
      *b = *(blobs + totBlobs--);
    else {
      // compute the area of the blob and estimate the stdev
      int a = b->area;
      b->xcg /= a;
      b->ycg /= a;
      b++;
      i++;
    }
  }
  totBlobs++;

  /* TODO: sort the blobs by size
  int which[numcolors];
  // now compute the centroids and sort them according to area, largest first
  for(i=0;i<totBlobs;i++) {
    BlobD *b = blobs+i;

    b->xcg /= b->area;
    b->ycg /= b->area;
    for (j=totBlobs; j>i; j--)
      if(blobs[j].area > blobs[j-1].area) {
        BlobD tmp = blobs[j];
        blobs[j] = blobs[j-1];
        blobs[j-1] = tmp;
      }
  }
   */
  //std::cout << "checkpoint 5" << std::endl;
  std::vector <BlobInfo> returnBlobs;
#ifdef MARKBLOB
  for (i = 0; i < totBlobs; i++) {
    cv::Point pt1((blobs + i)->left, (blobs + i)->top);
    cv::Point pt2((blobs + i)->right, (blobs + i)->bottom);
    BlobInfo blob;
    blob.UL_x = (blobs + i)->left;
    blob.UL_y = (blobs + i)->top;
    blob.LR_x = (blobs + i)->right;
    blob.LR_y = (blobs + i)->bottom;
    returnBlobs.push_back(blob);
    printf("Blob %d: ", i + 1);
    printf("p1->x %d  ", (blobs + i)->left);
    printf("p1->y %d  ", (blobs + i)->top);
    printf("p2->x %d  ", (blobs + i)->right);
    printf("p2->y %d\n", (blobs + i)->bottom);
    //#ifdef OPENCV
    cv::rectangle(newim, pt1, pt2, CV_RGB(255, 0, 255), 3);
    //#endif
  }
#endif

  return returnBlobs;
}
