/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SiftFeatures.hpp"

#include <boost/shared_array.hpp>


SiftFeatureExtractor::SiftFeatureExtractor(int imgWidth, int imgHeight)
        : width(imgWidth),
          height(imgHeight),
          numPixels(width * height),
          m_gauss_kernel(5),
          m_gauss_dev(1.5) {
  init();
}

SiftFeatureExtractor::~SiftFeatureExtractor() {
  cleanup();
}

void SiftFeatureExtractor::init() {
#ifdef SIFTGPU
  printf("using SIFT GPU\n");
  //init sift
  //char * argv[] = {"-m", "-s", "-v", "1"};
  char * argv[] = {"-m", "-fo", "-1", "-s", "-v", "0", "-pack"};
  //char * argv[] = {"-m", "-s", "-w", "3", "-fo", "-1", "-loweo"};
  //char * argv[] = {"-fo","-1","-v", "1"};

  int argc = sizeof (argv) / sizeof (char*);

  sift = new SiftGPU();
  sift->ParseParam(argc, argv);

  //create an OpenGL context for computation
  liteWindow = new LiteWindow();
  liteWindow->Create();
  liteWindow->MakeCurrent();
  if (sift->VerifyContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
      printf("[SiftFeatureExtractor::init] ERROR: SIFTGPU not initialized properly.\n");
  }

#else
  //create new SIFT filter - TODO: pass in appropriate params
  siftFilter = vl_sift_new(width, height, -1, 3, -1);
  vl_sift_set_peak_thresh(siftFilter, 0.02 / 3.0);        //set to match siftgpu param "-t" DOG thresh
  vl_sift_set_magnif(siftFilter, 3.8);    //default 3
  vl_sift_set_window_size(siftFilter, 1.0);   //default 2
//   std::cout << "Get peaks treashold. " << vl_sift_get_peak_thresh (siftFilter) << std::endl;
//   std::cout << "Get edges threshold. " << vl_sift_get_edge_thresh (siftFilter) << std::endl;
//   std::cout << "Get norm treashold. " << vl_sift_get_norm_thresh (siftFilter) << std::endl;
//   std::cout << "Get the magnification factor. " << vl_sift_get_magnif (siftFilter) << std::endl;
//   std::cout << "Get the Gaussian window size. " << vl_sift_get_window_size (siftFilter) << std::endl;

  siftImg = new vl_sift_pix[numPixels];
#endif  //SIFTGPU

  grayScaleImg = cv::Mat(width, height, CV_8U);
  grayScaleImgBlur = cv::Mat(width, height, CV_8U);
  imgSiftFeatures = SiftFeatures::Ptr(new SiftFeatures(0));
}

void SiftFeatureExtractor::cleanup() {

  //delete SIFT filter
#ifdef SIFTGPU
  delete sift;
  delete liteWindow;
#else
  vl_sift_delete(siftFilter);
  delete[] siftImg;
#endif  //SIFTGPU
}

SiftFeatures::Ptr
SiftFeatureExtractor::calcSiftDescriptors(unsigned long frameNum, const cv::Mat imgSrc, cv::Mat imgDst) {

  //calc new keypoints and descriptors
  calcSiftDescriptors(frameNum, imgSrc);

  //copy src data into dst
//    imgDst = cvCloneImage(imgSrc);
//    cvZero(imgDst);

  //draw img and descriptors on imgDst
  for (int i = 0; i < imgSiftFeatures->vlfeat_keys->size(); ++i) {
    //std::cout << octaveKeypoints[0].x << "  " << octaveKeypoints[0].y << std::endl;
    //std::cout << octaveKeypoints[1].x << "  " << octaveKeypoints[1].y << std::endl;

    cv::Point p;
    p.x = (int) imgSiftFeatures->vlfeat_keys->at(i)->x;
    p.y = (int) imgSiftFeatures->vlfeat_keys->at(i)->y;
    float scale = imgSiftFeatures->vlfeat_keys->at(i)->sigma + 1.0;
    float angle = -imgSiftFeatures->vlfeat_keys->at(i)->ori;
    cv::circle(imgDst, p, static_cast<int>(scale), CV_RGB(255, 0, 0), 1, 8, 0);

    cv::Point line;
    //rotate line by orientation angle
    float si = sin(angle), co = cos(angle);
    line.x = static_cast<int>(co * scale);
    line.y = static_cast<int>(si * scale);

    line.x += p.x; //translate line to keypoint center
    line.y += p.y; //translate line to keypoint center
    cv::line(imgDst, p, line, CV_RGB(255, 0, 0), 1, 8, 0);      //to show orientation

  }

  return imgSiftFeatures;
}

SiftFeatures::Ptr SiftFeatureExtractor::calcSiftDescriptors(unsigned long frameNum, const cv::Mat imgSrc) {
  //verify corrrect img size
  if (imgSrc.depth() != CV_8U && imgSrc.channels() != 1) {
    printf("[SiftFeatureExtractor::calcSiftDescriptors] Wrong image type! Not running SIFT!");
    return imgSiftFeatures;
  }

  currFrameNum = frameNum;

  //clear data from last img
  imgKeypoints.clear();
  imgSiftFeatures.reset(new SiftFeatures(currFrameNum));

  //convert frame to grayscale vl_sift_pix. SIFT only works on grayscale imgs
  cv::cvtColor(imgSrc, grayScaleImg, cv::COLOR_BGR2GRAY);
  //cvConvertImage(imgSrc, grayScaleImg);

  // blur
  cv::GaussianBlur(grayScaleImg, grayScaleImgBlur, cv::Size(m_gauss_kernel, m_gauss_kernel), m_gauss_dev, m_gauss_dev);

#ifdef SIFTGPU
  //use GPU sift version
  liteWindow->MakeCurrent();

  if (sift->RunSIFT(width, height, (unsigned char *) grayScaleImg->imageData, GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
      SiftFeature::Ptr vlfeat_k;

      int num = sift->GetFeatureNum();
      if (num > 0) {
          std::vector<SiftGPU::SiftKeypoint> ks(num);
          std::vector<float> desc(128*num);

          sift->GetFeatureVector(&ks[0], (float*) &desc[0]);

          //copy sift
          for (unsigned i = 0; i < num; i++) {

              //fill vlfeat keys
              vlfeat_k.reset(new SiftFeature());
              vlfeat_k->x = ks[i].x;
              vlfeat_k->y = ks[i].y;
              vlfeat_k->sigma = ks[i].s;
              vlfeat_k->ori = -ks[i].o + 2*PI;      //vlfeat needs angele in [0, 2*PI] range
              //P::CopyVec((float*) & desc[i*128], vlfeat_k->descriptor.get(), 128);
              //copy descriptor
              float* src = &desc[i*128];
              float* dst = vlfeat_k->descriptor.get();
              for (register unsigned n = 0; n < 128; ++n) {
                  *dst++ = *src++;
              }

              imgSiftFeatures->vlfeat_keys->push_back(vlfeat_k);
          }
      } else std::cout << "No SIFT found" << std::endl;
  } else throw P::Except(__HERE__, "SiftGPU Error!");
#else
  //useCPU sift version

  //TODO: EAK: getting negative pixel values? why? how?
  for (int i = 0; i < grayScaleImg.rows * grayScaleImg.cols; ++i) {
    siftImg[i] = (static_cast<unsigned char> (grayScaleImg.data[i]) / 255.0);
    //std::cout << "grayscale value: " << static_cast<unsigned char>(grayScaleImg->imageData[i])/1.0 << " normalized value: " << siftImg[i] << std::endl;
  }

  //START TESTING
//    vl_sift_pix bestdescriptor[128];
//    vl_sift_pix descriptor[128];
//    vl_sift_pix gpudescriptor[128] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
//0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001272, 0.000121, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000009,
//0.335675, 0.015803, 0.002562, 0.003014, 0.000663, 0.000213, 0.002823, 0.182190, 0.108086, 0.010555, 0.005735, 0.078681, 0.050220, 0.011574, 0.012876, 0.101801, 0.001619, 0.000004, 0.000000, 0.015838,
//0.026895, 0.025381, 0.013696, 0.009460, 0.023547, 0.023506, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000017, 0.335675, 0.335675, 0.016342, 0.007699, 0.000448, 0.000053, 0.000700, 0.050831,
//0.223916, 0.203782, 0.040711, 0.109441, 0.054214, 0.043605, 0.040457, 0.059520, 0.006003, 0.002630, 0.002425, 0.022635, 0.071077, 0.117870, 0.051315, 0.024698, 0.025526, 0.028274, 0.000000, 0.000000,
//0.000000, 0.020458, 0.008372, 0.000302, 0.335675, 0.335675, 0.000299, 0.000011, 0.000000, 0.050583, 0.019087, 0.003539, 0.282237, 0.335675, 0.064319, 0.036733, 0.011094, 0.005766, 0.004240, 0.013612,
//0.003486, 0.048362, 0.086782, 0.109814, 0.052557, 0.013409, 0.002312, 0.002783};
//    float bestdist = FLT_MAX;
//    double bestMag = 0;
//    double bestWind = 0;
//    double currMag;
//    double currWind;
//    for (int i = 0; i< 20; ++i) {
//        for (int j = 0; j< 20; ++j) {
//
//            //set params
//            currMag = 0 + i*0.2;
//            currWind = 0 + j*0.2;
//            vl_sift_set_magnif(siftFilter, currMag);    //default 3
//            vl_sift_set_window_size(siftFilter, currWind); //default 2
//
//            //get descr
//            vl_sift_calc_raw_descriptor(siftFilter, siftImg, &descriptor[0], width, height, 2.682200, 2.941263, 1.014252, 6.058855);
//
//            //find dist to gpu descr
//            vl_sift_pix dif, distsq = 0;
//            for (int i = 0; i < 128; i++) {
//                dif = gpudescriptor[i] - descriptor[i];
//                distsq += dif * dif;
//            }
//            printf("distsq: [%f]\n", distsq);
//            if (distsq < bestdist) {
//                bestdist = distsq;
//                bestMag = currMag;
//                bestWind = currWind;
//                //bestdescriptor = descriptor;
//                printf("current best dist: [%f] mag: [%f] win: [%f]\n", bestdist, bestMag, bestWind);
//            }
//        }
//    }
//
//    printf("best dist: [%f] mag: [%f] win: [%f]\n", bestdist, bestMag, bestWind);
  //END TESTING

  //process img with filter - one octave at a time
  int moreOctaves = vl_sift_process_first_octave(siftFilter, siftImg);
  processCurrentOctave();

  while (moreOctaves != VL_ERR_EOF) {
    moreOctaves = vl_sift_process_next_octave(siftFilter);
    processCurrentOctave();
  }

#endif  // USE_SIFTGPU

  return imgSiftFeatures;
}

void SiftFeatureExtractor::processCurrentOctave() {
  //local vars
  double angles[4];
  boost::shared_array<vl_sift_pix> currDescr;//(new vl_sift_pix[128]);
  int numAngles = 0;

  vl_sift_detect(siftFilter);
  VlSiftKeypoint const *octaveKeypoints = vl_sift_get_keypoints(siftFilter);
  for (int i = 0; i < siftFilter->nkeys; ++i) {
    VlSiftKeypoint const *currKeypoint = &(octaveKeypoints[i]);
    imgKeypoints.push_back(currKeypoint);

    numAngles = vl_sift_calc_keypoint_orientations(siftFilter, angles, currKeypoint);
    for (int n = 0; n < numAngles; ++n) {
      currDescr.reset(new vl_sift_pix[128]);
      vl_sift_calc_keypoint_descriptor(siftFilter, currDescr.get(), currKeypoint, angles[n]);

      //fill vlfeat keys
      SiftFeature::Ptr currKey(new SiftFeature());
      currKey->x = currKeypoint->x;
      currKey->y = currKeypoint->y;
      currKey->sigma = currKeypoint->sigma;
      currKey->ori = static_cast<float>(angles[n]);
      currKey->descriptor = currDescr;

      imgSiftFeatures->vlfeat_keys->push_back(currKey);
    }
  }
}

//TODO: find better place to put these??
/////////////////////////////////////////////////////////////////////////////////
////////////// SIFT Matching methods //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/* This searches through the keypoints in klist for the two closest
   matches to key.  If the closest is less than 0.6 times distance to
   second closest, then return the closest match.  Otherwise, return
   NULL.
*/
SiftFeature::Ptr SiftFeatureExtractor::CheckForMatch(SiftFeature::Ptr key, SiftFeatureVectPtr klist) {
  vl_sift_pix dsq, distsq1 = 100000000, distsq2 = 100000000;

  /* Find the two closest matches, and put their squared distances in
     distsq1 and distsq2.
  */
  SiftFeatureVectPtr::element_type::const_iterator kListItr, minkey;
  for (kListItr = klist->begin(); kListItr != klist->end(); ++kListItr) {
    dsq = DistSquared(key, (*kListItr));

    if (dsq < distsq1) {
      distsq2 = distsq1;
      distsq1 = dsq;
      minkey = kListItr;
    } else if (dsq < distsq2) {
      distsq2 = dsq;
    }
  }


  /* Check whether closest distance is less than 0.6 of second. */
  if (10 * 10 * distsq1 < 7 * 7 * distsq2) {
    //std::cout << "Match. Dist 1: " << distsq1 << " Dist 2: " << distsq2 << std::endl;
    return (*minkey);
  } else {
    //std::cout << "No Match. Dist 1: " << distsq1 << " Dist 2: " << distsq2 << std::endl;
    return SiftFeature::Ptr();
  }
}

float SiftFeatureExtractor::GetBestDistance(SiftFeature::Ptr key, SiftFeatureVectPtr klist) {
  vl_sift_pix dsq, distsq1 = 100000000, distsq2 = 100000000;

  /* Find the two closest matches, and put their squared distances in
     distsq1 and distsq2.
  */
  SiftFeatureVectPtr::element_type::const_iterator kListItr, minkey;
  for (kListItr = klist->begin(); kListItr != klist->end(); ++kListItr) {
    dsq = DistSquared(key, (*kListItr));

    if (dsq < distsq1) {
      distsq2 = distsq1;
      distsq1 = dsq;
      minkey = kListItr;
    } else if (dsq < distsq2) {
      distsq2 = dsq;
    }
  }

  //  std::cout << "Dist 1: " << distsq1 << " Dist 2: " << distsq2 << std::endl;
  return distsq1;
}

void SiftFeatureExtractor::RemoveMatch(SiftFeature::Ptr key, SiftFeatureVectPtr klist) {
  vl_sift_pix dsq, distsq1 = 100000000, distsq2 = 100000000;

  /* Find the two closest matches, and put their squared distances in
     distsq1 and distsq2.
  */
  SiftFeatureVectPtr::element_type::iterator kListItr, minkey;
  for (kListItr = klist->begin(); kListItr != klist->end(); ++kListItr) {
    dsq = DistSquared(key, (*kListItr));

    if (dsq < distsq1) {
      distsq2 = distsq1;
      distsq1 = dsq;
      minkey = kListItr;
    } else if (dsq < distsq2) {
      distsq2 = dsq;
    }
  }


  /* Check whether closest distance is less than 0.6 of second. */
  if (10 * 10 * distsq1 < 7 * 7 * distsq2) {
    std::cout << "Removing Match. Dist 1: " << distsq1 << " Dist 2: " << distsq2 << std::endl;
    klist->erase(minkey);
  }
}

/* Return squared distance between two keypoint descriptors.
*/
vl_sift_pix SiftFeatureExtractor::DistSquared(SiftFeature::Ptr k1, SiftFeature::Ptr k2) {
  vl_sift_pix dif, distsq = 0;
  boost::shared_array<vl_sift_pix> pk1, pk2;

  pk1 = k1->descriptor;
  pk2 = k2->descriptor;

  for (int i = 0; i < 128; i++) {
    dif = pk1[i] - pk2[i];
    distsq += dif * dif;
  }
  return distsq;
}
//end SIFT matching methods
