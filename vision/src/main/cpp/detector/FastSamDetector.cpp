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
  salmap = cv::Mat_<float>::ones(imgHeight, imgWidth);
  visionProcessName = "FastSamDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.FastSamDetector");
}

FastSamDetector::~FastSamDetector() {
}

void FastSamDetector::init() {
  LOG4CXX_DEBUG(logger, "[init]");

  PythonHelper* pyInstance = PythonHelper::getInstance();

  LOG4CXX_DEBUG(logger, "initializing python...");
  pyInstance->initializePython(); // this holds the GIL after returning
  LOG4CXX_DEBUG(logger, "done initializing python.");

  try {
    LOG4CXX_DEBUG(logger, "[init] waiting on GIL...");
    pyInstance->acquireGIL();
    LOG4CXX_DEBUG(logger, "[init] GIL acquired.");

    // LOG4CXX_DEBUG(logger, "[init] 2");
    // py::module_ module = pyInstance->importModule("vector_perception.segmentation");
    // LOG4CXX_DEBUG(logger, "[init] 3");
    // Equivalent to "from vector_perception.segmentation import Sam2DSegmenter"
    // py::module_::import("vector_perception.segmentation");
    // Sam2DSegmenter = module.attr("Sam2DSegmenter");
    Sam2DSegmenter = pyInstance->importFromModule("vector_perception.segmentation", "Sam2DSegmenter");
    LOG4CXX_DEBUG(logger, "[init] 4");
    // Construct a Python object of class Sam2DSegmenter
    segmenter = Sam2DSegmenter(
      py::arg("model_path") = "models/FastSAM-s.pt",
      py::arg("device") = "cpu",
      py::arg("min_analysis_interval") = 4.0,
      py::arg("use_tracker") = false,
      py::arg("use_analyzer") = false
    );
    LOG4CXX_DEBUG(logger, "[init] 5");
  }
  catch (py::error_already_set& e) {
    LOG4CXX_ERROR(logger, boost::format("[init] %s") % e.what());
  }

  pyInstance->releaseGIL();
  LOG4CXX_DEBUG(logger, "[init] done");
}

void FastSamDetector::cleanup() {
  LOG4CXX_DEBUG(logger, "[cleanup]...");
  PythonHelper* pyInstance = PythonHelper::getInstance();
  pyInstance->acquireGIL();

  Sam2DSegmenter = py::object();
  segmenter = py::object();

  pyInstance->releaseGIL();
  LOG4CXX_DEBUG(logger, "[cleanup] done");
}

void FastSamDetector::handleSaliencyNotification(SaliencyNotification::ConstPtr notification) {
  notification->saliencyMap.copyTo(salmap);
}

void FastSamDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  cv::Mat frame = notification->captureData->frame;

  PythonHelper* pyInstance = PythonHelper::getInstance();
  LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 1");
  {
    pyInstance->acquireGIL();
    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 2");
    // process the image
    // results[0] --> [mask0, mask1, ...], each mask is binary image mask?
    // results[1] --> [bbox1, bbox2, ...], each box is [x1, y1, x2, y2]
    // results[2] --> [track_id1, track_id2, ...]
    // results[3] --> [conf1, conf2, ...]
    // results[4] --> [name1, name2, ...]
    py::array_t<uchar> pyFrame = PythonUtils::convertMatToNumpy(frame);
    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 2.1");
    py::object tmpResults = segmenter.attr("process_image")(pyFrame);
    LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 2.2");
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

  LOG4CXX_DEBUG(logger, "[handleCaptureNotification] 6");

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
  for (TypesByDescriptor::const_iterator descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    for (int i = 0; i < masks.size(); ++i) {
      cv::Mat cvMat = PythonUtils::convertNumpyToMat(masks[i].cast<py::array_t<uchar>>());
      float conf = confs[i].cast<float>();

      // create MemoryObject for each relevant typeId
      for (typeIds_itr = descriptor_iter->second.begin(); typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
        MemoryObjectMask::Ptr moMask(new MemoryObjectMask(captureData, cvMat));
        MemoryObject::Ptr newObj(
          new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), captureData, moMask));
        newObj->addValidationResult(conf, descriptor_iter->first);
        newObjs->push_back(newObj);
      }
    }
  }

  return newObjs;
}
