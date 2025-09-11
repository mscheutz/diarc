/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#include <cfloat>
#include <boost/lexical_cast.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "capture/util/CaptureUtilities.hpp"
#include "FastSamDetector.hpp"
#include "display/Display.hpp"
#include "pyvis/PythonUtils.hpp"
#include "pyvis/PythonHelper.hpp"

#include <pybind11/embed.h> // everything needed for embedding

namespace py = pybind11;

using namespace diarc::stm;

FastSamDetector::FastSamDetector(const long long& processorId, const int imgWidth, const int imgHeight)
  : ObjectDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "FastSamDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.FastSamDetector");
  segmenter_initialized = false;

  createSegmenter();
}

FastSamDetector::~FastSamDetector() {
  destroySegmenter();
}

void FastSamDetector::createSegmenter() {
  if (!segmenter_initialized) {
    LOG4CXX_DEBUG(logger, "[createSegmenter]");

    PythonHelper* pyInstance = PythonHelper::getInstance();
    try {
      LOG4CXX_DEBUG(logger, "[createSegmenter] waiting on GIL...");
      pyInstance->acquireGIL();
      LOG4CXX_DEBUG(logger, boost::format("[createSegmenter] GIL acquired: %d") % PyGILState_Check());

      LOG4CXX_DEBUG(logger, "[createSegmenter] 2");
      // Equivalent to "from vector_perception.segmentation import Sam2DSegmenter"
      py::module_ module = pyInstance->importModule("vector_perception.segmentation");
      LOG4CXX_DEBUG(logger, "[createSegmenter] 3");
      Sam2DSegmenter = module.attr("Sam2DSegmenter");
      LOG4CXX_DEBUG(logger, "[createSegmenter] 4");
      // Construct a Python object of class Sam2DSegmenter
      segmenter = Sam2DSegmenter(
        py::arg("model_path") = "models/FastSAM-s.pt",
        py::arg("device") = "cpu",
        py::arg("min_analysis_interval") = 4.0,
        py::arg("use_tracker") = false,
        py::arg("use_analyzer") = false
      );
      LOG4CXX_DEBUG(logger, "[createSegmenter] 5");
    } catch (py::error_already_set& e) {
      LOG4CXX_ERROR(logger, boost::format("[createSegmenter] %s") % e.what());
    }

    pyInstance->releaseGIL();
    segmenter_initialized = true;
    LOG4CXX_DEBUG(logger, boost::format("[createSegmenter] done. GIL released: %d") % PyGILState_Check());
  }
}

void FastSamDetector::destroySegmenter() {
  if (segmenter_initialized) {

    LOG4CXX_DEBUG(logger, "[destroySegmenter]...");
    PythonHelper* pyInstance = PythonHelper::getInstance();
    pyInstance->acquireGIL();

    Sam2DSegmenter = py::object();
    segmenter = py::object();

    pyInstance->releaseGIL();
    segmenter_initialized = false;
    LOG4CXX_DEBUG(logger, "[destroySegmenter] done");
  }
}

void FastSamDetector::init() {
}

void FastSamDetector::cleanup() {
}

void FastSamDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  cv::Mat frame = notification->captureData->frame;

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
    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 1");
    py::object tmpResults = segmenter.attr("process_image")(pyFrame);
    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 2");
    py::tuple results = py::cast<py::tuple>(tmpResults);

    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 3");

    // Visualize results
    py::object overlay = segmenter.attr("visualize_results")(
      pyFrame,
      results[0].cast<py::list>(),
      results[1].cast<py::list>(),
      results[2].cast<py::list>(),
      results[3].cast<py::list>(),
      results[4].cast<py::list>()
    );

    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 4");

    displayFrame = PythonUtils::convertNumpyToMat(overlay.cast<py::array_t<uchar>>());

    // populate MOs and send notifications
    MemoryObject::VecPtr newClusterObjects = createMemoryObjects(results[0].cast<py::list>(), results[3].cast<py::list>(),
                                                                 notification->captureData);
    sendDetectionNotifications(newClusterObjects);

    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 5");
  }

  pyInstance->releaseGIL();
  LOG4CXX_DEBUG(logger, boost::format("[handleCaptureNotification] GIL released: %d") % PyGILState_Check());

  if (getDisplayFlag()) {
    diarc::Display::displayFrame(displayFrame, getDisplayName());
  }
}

MemoryObject::VecPtr FastSamDetector::createMemoryObjects(py::list masks, py::list confs,
                                                          CaptureData::ConstPtr captureData) {
  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();

  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  MemoryObject::VecPtr newObjs(new MemoryObject::Vec());
  LOG4CXX_DEBUG(logger, boost::format("[createMemoryObjects] num detected objects: %d") % masks.size());
  for (TypesByDescriptor::const_iterator descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    for (int i = 0; i < masks.size(); ++i) {
       auto pyMask = masks[i].attr("cpu")().attr("numpy")(); // convert torch.Tensor to numpy

      cv::Mat cvMask = PythonUtils::convertNumpyToMat(pyMask.cast<py::array_t<uchar>>());
      cv::Mat binary;
      cv::threshold(cvMask, binary, 0.5, 255, cv::THRESH_BINARY);
      float conf = confs[i].cast<float>();

      // create MemoryObject for each relevant typeId
      for (typeIds_itr = descriptor_iter->second.begin(); typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
        MemoryObjectMask::Ptr moMask(new MemoryObjectMask(captureData, binary));
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
