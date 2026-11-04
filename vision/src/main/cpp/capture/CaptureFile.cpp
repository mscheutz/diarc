/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Capture.hpp"
#include "util/CaptureUtilities.hpp"

#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ios>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>

namespace diarc {
  namespace capture {

    class CaptureFile : public Capture {
    public:

      CaptureFile(const std::string &configFile)
              : Capture(configFile),
                imageFilename(""),
                lastFileTime(0) {

        // parse xml for configuration params
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(configFile, pt);

        // get directory of config file
        unsigned found = configFile.find_last_of("/\\");
        std::string dir = configFile.substr(0, found + 1);

        // get imgFilename
        imageFilename = pt.get<std::string>("capture.imgFilename");

        // check if relative path of full path (starts with "/")
        if (imageFilename.find_first_of("/\\") != 0) {
          imageFilename = dir + pt.get<std::string>("capture.imgFilename");
        }

        switch (camMode) {
          case MONO_FILE:
            initMonoFile();
            break;
          case MULTI_FILE_LOG:
            initTSreplay();
            break;
          case MONO_AVI:
            initMonoAVI();
            break;
        }
      };

      ~CaptureFile() {
        //TODO: clean up capture, capture2
      };

      bool captureCurrentCameraFrame() {
        switch (camMode) {
          case MONO_FILE:
            return captureHelperMonoFile();
          case MULTI_FILE_LOG:
            return captureHelperMultiFileFromLog();
          case MONO_AVI:
            return captureHelperMonoAVI();
        }
        return false;
      }

    private:

      void initMonoFile() {
        //set filename of timestamp info.  assumes same basename as image filename.
        size_t pos = imageFilename.find_last_of(".");
        timestampFilename = imageFilename.substr(0, pos + 1) + "time";
        printf("timestamp filename: %s\n", timestampFilename.c_str());
      }

      void initTSreplay() {
        std::ifstream file;
        file.open(imageFilename.c_str());
        std::cout << "open file: " << imageFilename << std::endl;
        if (!file) {
          std::cerr << "Unable to open file datafile.txt";
          exit(1); // call system to stop
        }
        std::string line;
        imgidx = imgStart;
        //first line is the size
        getline(file, line);
        imgNum = atoi(line.c_str());
        std::cout << imgNum << " images: " << sizeof(TSFilename) * imgNum << std::endl;
        tsf = (TSFilename *) malloc(sizeof(TSFilename) * imgNum);
        imgEnd = imgNum - 1;
        int idx = 0;
        std::string buf;
        double ts = 0;
        for (; idx < imgNum && file; idx = idx + 1) {
          file >> buf;
          //cout << "ts: " << buf << std::endl;
          double tsnow = atof(buf.c_str());

          if (idx == 0) {
            tsf[idx].deltatime = 0;
          } else {
            tsf[idx].deltatime = (int) (tsnow - ts);
          }

          std::cout << tsf[idx].deltatime << std::endl;
          ts = tsnow;
          file >> buf;
          //int length = buf.length;
          std::cout << "img: " << buf << " length: " << std::endl;
          tsf[idx].imgname = (char *) malloc(100);
          strcpy(tsf[idx].imgname, buf.c_str());

        }
      }

      void initMonoAVI() {
        capture.open(imageFilename);
        //cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 30);   //currently doesn't work on linux with ffmpeg
      }

      bool captureHelperMonoFile() {
        //fps regulator - tries to keep capture around 30fps
        gettimeofday(&tv, NULL);
        currenttime = (unsigned) (tv.tv_sec * 1000) + (unsigned) (tv.tv_usec / 1000.0); //convert to milliseconds
        timediff = currenttime - lasttime;
        unsigned long long timeToPass = 33; //want approx 1/30 seconds (33 MILLIseconds) to pass between captures
        if (timediff < timeToPass) {
          //       printf("usleep for: %d\n", 33-timediff);
          unsigned long sleeptime = (timeToPass - timediff) * 1000;
          usleep(sleeptime); //in MICROoseconds
        }
        lasttime = currenttime;

        //check last write time from timestamp file
        std::string line;
        unsigned long long currFileTime = 0;
        std::ifstream timeFile; //(timestampFilename.c_str());
        timeFile.open(timestampFilename.c_str());
        if (timeFile.is_open()) {
          getline(timeFile, line);
          currFileTime = atoll(line.c_str());
          //std::cout << "currfiletime: " << currFileTime << std::endl;
          if (currFileTime > lastFileTime) {
            //update timestamp and capture image
            lastFileTime = currFileTime;
          } else {
            return false;
          }
        } else {
          //error opening timestamp file
          static bool displayed = false;
          if (!displayed) {
            printf("WARNING: Error opening timestamp file <%s>. Ignoring timestamp constraint. Only reporting once!\n",
                   timestampFilename.c_str());
            displayed = true;
          }
        }
        timeFile.close();

        //TODO: is there a way to loadImage without allocating imageData??
        cv::Mat tmpFrame = cv::imread(imageFilename, cv::IMREAD_COLOR);
        if (tmpFrame.empty()) {
          LOG4CXX_ERROR(logger, boost::format("Could not load image file: %s.") % imageFilename);
          return false;
        }
        //rescale image
        cv::resize(tmpFrame, frame, cv::Size(imgWidth, imgHeight), 0, 0, cv::INTER_LINEAR);
        //printf("capture image size: %d x %d\n", tmpFrame->width, tmpFrame->height);

        return true;
      }

      bool captureHelperMultiFileFromLog() {

        std::string filename = tsf[imgidx].imgname;

        //subtract the time period between each call from the total sleeping time to be more precise when replaying
        if (imgidx > imgStart && imgidx < imgEnd) {
          gettimeofday(&tv, NULL);
          currenttime = (unsigned) tv.tv_sec * 1000000 + (unsigned) tv.tv_usec;
          timediff = currenttime - lasttime;
          //std::cout << "with pre-time " << timediff << std::endl;

          long actualsleep = tsf[1 + imgidx].deltatime * 1000 - timediff;
          if (actualsleep < 0)
            actualsleep = 0;
          //std::cout << "sleep " << actualsleep << std::endl;
          usleep(actualsleep);
        }
        gettimeofday(&tv, NULL);
        lasttime = (unsigned) tv.tv_sec * 1000000 + (unsigned) tv.tv_usec;

        cv::Mat tmpFrame = cv::imread(filename, cv::IMREAD_COLOR);
        if (tmpFrame.empty()) {
          LOG4CXX_ERROR(logger, boost::format("Could not load image file: %s.") % imageFilename);
          return false;
        }
        //rescale image
        cv::resize(tmpFrame, frame, cv::Size(imgHeight, imgWidth), 0, 0, cv::INTER_LINEAR);
        //printf("capture image size: %d x %d\n", tmpFrame->width, tmpFrame->height);

        //REV: made it so that you don't have the loop, as we don't want it. Also, it was causing segfaults -_-;
        if (imgidx < imgEnd) {
          ++imgidx; //increment
        }

        //std::cout << "CURRENT IDX: " << imgidx << std::endl;
        //REV:else imgidx doesnt change (loop through most recent img)

        return true;
      }

      bool captureHelperMonoAVI() {
        //TODO: cvSetCaptureProperty(...) does not currently work. when it does this function needs to be reworked to use the
        //the built in fps mechanism and the CV_CAP_PROP_POS_FRAMES property to reset avi to begining for looping

        //fps regulator - tries to keep capture around 30fps - TODO: use built in fps (see note above)
        gettimeofday(&tv, NULL);
        currenttime = (unsigned) (tv.tv_sec * 1000) + (unsigned) (tv.tv_usec / 1000.0); //convert to milliseconds
        timediff = currenttime - lasttime;
        if (timediff < 50) { //want approx 1/30 seconds (33 MILLIseconds) to pass between captures
          //       printf("usleep for: %d\n", 33-timediff);
          unsigned long sleeptime = (50 - timediff) * 1000;
          usleep(sleeptime); //in MICROoseconds
        }
        lasttime = currenttime;


        static int frameCount = 0;
        //printf("avi frame num: %e\n", cvGetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES));
        //printf("avi num frames: %e\n", cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT));
        if (capture.get(cv::CAP_PROP_POS_FRAMES) >= capture.get(cv::CAP_PROP_FRAME_COUNT) - 1) {
          //reached end of avi - TODO: use built in reset mechanism (see note above)
          capture.open(imageFilename);
        }
        if (capture.grab()) {
          cv::Mat tmpFrame;
          capture.retrieve(tmpFrame);
          tmpFrame.copyTo(frame); //fill frame without re-allocating imgData
          ++frameCount;
        } else {
          //either error
          LOG4CXX_ERROR(logger, "Error grabbing frame!");
          return false;
        }

        return true;
      }

      cv::VideoCapture capture; // for AVI, V4L, etc
      std::string imageFilename; //for reading from file (single images or video)
      std::string timestampFilename; //tries to use this when reading images from file
      unsigned long long lastFileTime;

      //! for REV's capture from file stuff
      int imgidx, imgNum, imgStart, imgEnd;

      typedef struct _TSFilename {
        int deltatime;
        char *imgname;
      } TSFilename;
      TSFilename *tsf;
    };

  } //namespace capture
} //namespace diarc
