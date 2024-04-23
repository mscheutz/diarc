/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   SceneChangeNotification.hpp
 * Author: Evan Krause
 *
 * Created on August 27, 2015, 4:55 PM
 */

#ifndef SCENECHANGENOTIFICATION_HPP
#define	SCENECHANGENOTIFICATION_HPP

#include "common/notification/ImageProcessorNotification.hpp"

/**
 * Scene change notification: can be either start of a scene change or end of a
 * scene change, i.e. when the scene becomes static again.
 */
class SceneChangeNotification : public ImageProcessorNotification {
public:
  typedef boost::shared_ptr<SceneChangeNotification> Ptr;
  typedef boost::shared_ptr<const SceneChangeNotification> ConstPtr;

  SceneChangeNotification(const VisionProcessConstPtr& notifier_,
                          const unsigned long long& frameNumber_,
                          const CaptureData::ConstPtr& captureData_,
                          bool changeStarted_)
          : ImageProcessorNotification(notifier_, SCENE_CHANGE, frameNumber_, captureData_),
            changeStarted(changeStarted_) {
  }

  //! true when the notification is about change start, false otherwise
  const bool changeStarted;
};

#endif	/* SCENECHANGENOTIFICATION_HPP */

