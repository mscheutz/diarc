/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CalculateOpticalFlow.hpp"

CalculateOpticalFlow::CalculateOpticalFlow (const int imgWidth, const int imgHeight)
: size(cv::Size(imgWidth, imgHeight)),
gray_currentFrame(size, CV_8UC1),
gray_prevFrame(size, CV_8UC1) {
}

CalculateOpticalFlow::~CalculateOpticalFlow() {
}

void CalculateOpticalFlow::calcOpticalFlowFarneback(const cv::Mat& prevFrame, const cv::Mat& currentFrame, cv::Mat& flow) {
  cv::cvtColor(currentFrame, gray_currentFrame, cv::COLOR_BGR2GRAY);
  cv::cvtColor(prevFrame, gray_prevFrame, cv::COLOR_BGR2GRAY);
  cv::calcOpticalFlowFarneback(gray_prevFrame, gray_currentFrame, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
}


/* This method is used to draw the calculate the optical flow lines.
   Takes in current frame and previous frame and fills (results) frame
   with optical flow results.  results frame must be allocated with
   same size as currentFrame and prevFrame.
*/

void CalculateOpticalFlow::calcOpticalFlowLK(const cv::Mat currentFrame, const cv::Mat prevFrame, cv::Mat results) {
    //very size of images
    if ((currentFrame.cols != prevFrame.cols) || (currentFrame.cols != results.cols) ||
            (currentFrame.rows != prevFrame.rows) || (currentFrame.rows != results.rows)) {
        printf("curr: %dx%d\n", currentFrame.cols, currentFrame.rows);
        printf("prev: %dx%d\n", prevFrame.cols, prevFrame.rows);
        printf("res: %dx%d\n", results.cols, results.rows);
        printf("[CalculateOpticalFlow::drawOpticalFlow]: Error - frame sizes do not match!");
        return;
    }

  std::vector<cv::Point2d> frame1_features;
  std::vector<cv::Point2d> frame2_features;
  std::vector<uchar> optical_flow_found_feature;
  std::vector<float> optical_flow_feature_error;

  results = cv::Mat::zeros(results.size(), results.type());

  //opencv hack
  //pthread_mutex_lock(&VisionConstants::cvCvtColor_mutex);
  cv::cvtColor (currentFrame, gray_currentFrame, cv::COLOR_BGR2GRAY);
  cv::cvtColor (prevFrame, gray_prevFrame, cv::COLOR_BGR2GRAY);
  //pthread_mutex_unlock(&VisionConstants::cvCvtColor_mutex);

  cv::goodFeaturesToTrack(gray_currentFrame, frame1_features, 0, .00001, .00001);
  cv::calcOpticalFlowPyrLK(gray_prevFrame, gray_currentFrame, frame1_features, frame2_features,
			  optical_flow_found_feature, optical_flow_feature_error);

  for (int i = 0; i < frame1_features.size(); i++) {
    // If Pyramidal Lucas Kanade didn't really find the feature, skip it.
    if ( optical_flow_found_feature[i] == 0 )    continue;
    int line_thickness;              line_thickness = 1;
    // CV_RGB(red, green, blue) is the red, green, and blue components
    // of the color you want, each out of 255.
    
    cv::Scalar R_line_color = CV_RGB(255, 0, 0);
    cv::Scalar G_line_color = CV_RGB(0, 255, 0);
    // Let's make the flow field look nice with arrows. 
    // The arrows will be a bit too short for a nice visualization because of the
    //   high framerate
    //(ie: there's not much motion between the frames). So let's lengthen them
    //by a factor of 3.
    
    cv::Point p, q;
    p.x = (int) frame1_features[i].x;
    p.y = (int) frame1_features[i].y;
    q.x = (int) frame2_features[i].x;
    q.y = (int) frame2_features[i].y;
    double angle;        angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
    double hypotenuse;   hypotenuse = sqrt( (p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x) );
    
    // Here we lengthen the arrow by a factor of three. 
    //q.x = (int) (p.x + 3 * hypotenuse * cos(angle));
    //q.y = (int) (p.y + 3 * hypotenuse * sin(angle));
    
    
    // "frame1" is the frame to draw on.
    // "p" is the point where the line begins.
    // "q" is the point where the line stops.
    // "CV_AA" means antialiased drawing.
    // "0" means no fractional bits in the center cooridinate or radius.      
    
    if(hypotenuse <= 30 && hypotenuse >= 2.2) {
      //printf("%f\n", hypotenuse);
      
      cv::line(results, p, q, G_line_color, line_thickness, cv::LINE_AA, 0 );
      //    cvCircle(results, p, 1, G_line_color, 5, 8, 0);
      // cvCircle(results, q, 1, G_line_color, 5, 8, 0);
      //cvLine(color, p, q, G_line_color, line_thickness, CV_AA, 0 );
      //cvCircle(color, p, 1, G_line_color, 5, 8, 0);
      //cvCircle(color, q, 1, G_line_color, 5, 8, 0);

      p.x = (int) (q.x + 2 * cos(angle + PI / 4));
      p.y = (int) (q.y + 2 * sin(angle + PI / 4));
      cv::line(results, p, q, G_line_color, line_thickness, cv::LINE_AA, 0 );
      p.x = (int) (q.x + 2 * cos(angle - PI / 4));
      p.y = (int) (q.y + 2 * sin(angle - PI / 4));
      cv:line( results, p, q, G_line_color, line_thickness, cv::LINE_AA, 0 );
            
    }
    // Now draw the tips of the arrow. I do some scaling so that the
    // tips look proportional to the main line of the arrow.       
  }

}
