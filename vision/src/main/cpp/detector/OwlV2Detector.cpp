/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#include <cfloat>
#include <boost/lexical_cast.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "capture/util/CaptureUtilities.hpp"
#include "OwlV2Detector.hpp"
#include "display/Display.hpp"
#include "pyvis/PythonUtils.hpp"
#include "pyvis/PythonHelper.hpp"

#include <pybind11/embed.h> // everything needed for embedding

namespace py = pybind11;

using namespace diarc::stm;

OwlV2Detector::OwlV2Detector(const long long& processorId, const int imgWidth, const int imgHeight)
  : ObjectDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "OwlV2Detector";
  logger = log4cxx::Logger::getLogger("diarc.detector.OwlV2Detector");
  detector_initialized = false;

  createDetector();
}

OwlV2Detector::~OwlV2Detector() {
  destroyDetector();
}

void OwlV2Detector::createDetector() {
  if (!detector_initialized) {
    LOG4CXX_DEBUG(logger, "[createDetector]");

    PythonHelper* pyInstance = PythonHelper::getInstance();
    try {
      LOG4CXX_DEBUG(logger, "[createDetector] waiting on GIL...");
      pyInstance->acquireGIL();
      LOG4CXX_DEBUG(logger, boost::format("[createDetector] GIL acquired: %d") % PyGILState_Check());

      LOG4CXX_DEBUG(logger, "[createDetector] 2");
      // Equivalent to "from diarc_vision.owlv2.owlv2_wrapper import Owlv2"
      py::module_ module = pyInstance->importModule("diarc_vision.owlv2.owlv2_wrapper");
      LOG4CXX_DEBUG(logger, "[createDetector] 3");
      OwlV2DetectorClass = module.attr("Owlv2");
      LOG4CXX_DEBUG(logger, "[createDetector] 4");
      // Construct a Python object of class OwlV2DetectorClass
      owlV2Detector = OwlV2DetectorClass(py::arg("device") = "cpu");
      LOG4CXX_DEBUG(logger, "[createDetector] 5");
    } catch (py::error_already_set& e) {
      LOG4CXX_ERROR(logger, boost::format("[createDetector] %s") % e.what());
    }

    pyInstance->releaseGIL();
    detector_initialized = true;
    LOG4CXX_DEBUG(logger, boost::format("[createDetector] done. GIL released: %d") % PyGILState_Check());
  }
}

void OwlV2Detector::destroyDetector() {
  if (detector_initialized) {

    LOG4CXX_DEBUG(logger, "[destroyDetector]...");
    PythonHelper* pyInstance = PythonHelper::getInstance();
    pyInstance->acquireGIL();

    OwlV2DetectorClass = py::object();
    owlV2Detector = py::object();

    pyInstance->releaseGIL();
    detector_initialized = false;
    LOG4CXX_DEBUG(logger, "[destroyDetector] done");
  }
}

void OwlV2Detector::init() {
}

void OwlV2Detector::cleanup() {
}

void OwlV2Detector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  cv::Mat frame = notification->captureData->frame;
  MemoryObject::VecPtr newObjects;

  PythonHelper* pyInstance = PythonHelper::getInstance();
  {
    pyInstance->acquireGIL();
    LOG4CXX_DEBUG(logger, boost::format("[handleCaptureNotification] GIL acquired: %d") % PyGILState_Check());
    // process the image
    // results[0] --> [mask0, mask1, ...], each mask is torch.Tensor?
    // results[1] --> [bbox1, bbox2, ...], each box is [x1, y1, x2, y2]
    // results[2] --> [track_id1, track_id2, ...]
    // results[3] --> [conf1, conf2, ...]
    // results[4] --> [name1, name2, ...]
    py::array_t<uchar> pyFrame = PythonUtils::convertMatToNumpy(frame);
    std::vector<std::string> labels = {"person", "face"}; // TODO: populate this with search predicates

    py::list pyLabels;
    for (const auto& label : labels) {
      pyLabels.append(label);
    }

    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 1");
    py::object tmpResults = owlV2Detector.attr("detect")(pyFrame, pyLabels, 0.2);
    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 2");
    py::tuple results = py::cast<py::tuple>(tmpResults);

    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 3");

    // populate MOs and send notifications
    newObjects = createMemoryObjects(results[0].cast<py::list>(), results[1].cast<py::list>(),
                                                                 notification->captureData);
    sendDetectionNotifications(newObjects);

    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 4");
  }

  pyInstance->releaseGIL();
  LOG4CXX_DEBUG(logger, boost::format("[handleCaptureNotification] GIL released: %d") % PyGILState_Check());

  // draw bounding boxes
  if (getDisplayFlag()) {
    frame.copyTo(displayFrame);

    MemoryObject::Vec::const_iterator newObjectsItr;
    for (newObjectsItr = newObjects->begin(); newObjectsItr != newObjects->end(); ++newObjectsItr) {
      const cv::Rect &moRect = (*newObjectsItr)->getDetectionMask()->getBoundingBox();
      cv::rectangle(displayFrame, cv::Point(moRect.x, moRect.y),
                    cv::Point(moRect.x + moRect.width, moRect.y + moRect.height),
                    CV_RGB(255, 0, 0), 2, 8, 0);
    }

    diarc::Display::displayFrame(displayFrame, getDisplayName());
  }
}

MemoryObject::VecPtr OwlV2Detector::createMemoryObjects(py::list boxes, py::list confs,
                                                          CaptureData::ConstPtr captureData) {
  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();

  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  MemoryObject::VecPtr newObjs(new MemoryObject::Vec());
  LOG4CXX_DEBUG(logger, boost::format("[createMemoryObjects] num detected objects: %d") % boxes.size());
  for (TypesByDescriptor::const_iterator descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    for (int i = 0; i < boxes.size(); ++i) {
      py::list pyBox = py::cast<py::list>(boxes[i]);
      cv::Rect bbox = cv::Rect(pyBox[0].cast<int>(), pyBox[1].cast<int>(), pyBox[2].cast<int>(), pyBox[3].cast<int>());
      float conf = confs[i].cast<float>();

      // create MemoryObject for each relevant typeId
      for (typeIds_itr = descriptor_iter->second.begin(); typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
        MemoryObjectMask::Ptr moMask(new MemoryObjectMask(captureData, bbox));
        MemoryObject::Ptr newObj(
          new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), captureData, moMask));
        newObj->addValidationResult(conf, descriptor_iter->first);
        newObjs->push_back(newObj);

        auto bbox = newObj->getDetectionMask()->getBoundingBox();
        LOG4CXX_DEBUG(logger, boost::format("Detected MO with box:(%d,%d) %dx%d.")
            % bbox.x % bbox.y % bbox.width % bbox.height);
      }

//      cv::imshow("binary", binary);
//      cv::waitKey(0);
    }
  }

  return newObjs;
}
