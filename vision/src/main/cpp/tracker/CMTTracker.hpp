/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CMTTracker.hpp
 * Author: Evan Krause
 *
 * Created on October 3 2017
 * 
 * This is a tracker based on the CMT algorithm found here:
 * https://github.com/gnebehay/CppMT
 * 
 * As of this writing (Oct 2017) the CMT algorithm can only handle a single
 * tracked object, so this tracker allows for the possibility of instantiating
 * multiple CMTs, one for each tracked object. All of which run on the same
 * tracking thread.
 */

#ifndef CMTTRACKER_HPP
#define	CMTTRACKER_HPP

#include "ObjectTracker.hpp"
#include "CMT.h"

typedef std::tr1::unordered_map<long long, cmt::CMT*> CMT_MAP;

class CMTTracker : public ObjectTracker
{
public:
  CMTTracker(const long long& processorId, const int imgWidth, const int imgHeight);
  ~CMTTracker();
  
  void loadConfig(const std::string& config);
  
protected:
    virtual void haveNewImage(CaptureNotification::ConstPtr notification);

private:
  virtual bool canTrack(const ade::stm::MemoryObject::Ptr& newMemObj);
  virtual void startTracking(const ade::stm::MemoryObject::Ptr& newMemObj);
  virtual void stopTracking(const ade::stm::MemoryObject::Ptr& existingMemObj);
  virtual void displayResults(CaptureData::ConstPtr capture);
  void drawCMTStatus(cv::Mat& im, cmt::CMT &cmt);
    
  // CMT framework
  CMT_MAP cmts;
  cv::Mat lastFrameGray;
  
  boost::mutex cmts_mutex;
};


#endif	/* CMTTRACKER_HPP */

