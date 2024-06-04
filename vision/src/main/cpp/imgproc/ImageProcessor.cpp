/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ImageProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <iterator>
#include <string>

#include "capture/Capture.hpp"
#include "ChangeDetector.hpp"
#include "display/Display.hpp"
#include "OpticalFlowProcessor.hpp"
#include "SiftProcessor.hpp"
#include "validator/ColorValidator.hpp"
#include "validator/DefinitionValidator.hpp"
#include "validator/SizeValidator.hpp"

#include "PlaneProcessor.hpp"
#include "validator/GlobalFeatureValidator.hpp"
#include "validator/SpatialRelationValidator.hpp"

#include "saliency/MasterSaliencyProcessor.hpp"
#ifdef USE_V4R_V0
//#include "saliency/RelativeHeightProcessor.hpp"
//#include "saliency/SurfaceOrientationProcessor.hpp"
//#include "saliency/SurfaceCurvatureProcessor.hpp"
#include "saliency/IKNSaliencyMapProcessor.hpp"
#include "saliency/ColorProcessor.hpp"
#include "saliency/SymmetryProcessor.hpp"
#include "saliency/LocationSaliencyProcessor.hpp"
#include "saliency/OrientationProcessor.hpp"
#include "saliency/MasterSaliencyProcessor.hpp"
#include "validator/ShapeValidator.hpp"
#include "validator/SurfaceMarkingValidator.hpp"
#endif //USE_V4R_V0

// NOTE: not named "ade.imgproc.ImageProcessor" because there could be non-static
// loggers named "ade.imgproc.ImageProcessor"
log4cxx::LoggerPtr ImageProcessor::factoryLogger = log4cxx::Logger::getLogger("ade.imgproc.ImageProcessor.Factory");

ImageProcessor::Ptr ImageProcessor::get(const ImageProcessorType type, const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo) {
  switch (type) {
    case SIFT:
      return SiftProcessor::Ptr(new SiftProcessor(processorId, imgWidth, imgHeight, isStereo));
      break;
    case CHANGEDETECTION:
      return ChangeDetector::Ptr(new ChangeDetector(processorId, imgWidth, imgHeight, isStereo));
      break;
    case OPTICALFLOW:
      return OpticalFlowProcessor::Ptr(new OpticalFlowProcessor(processorId, imgWidth, imgHeight, isStereo));
      break;
      //    case RELATIVEHEIGHT:
      //#ifdef USE_V4R_V0
      //      return RelativeHeightProcessor::Ptr(new RelativeHeightProcessor(processorId, imgWidth, imgHeight, isStereo));
      //#else
      //      LOG4CXX_ERROR(factoryLogger, "RelativeHeightProcessor not available. Did you compile vision with V4R?");
      //#endif
      //      break;
      //    case SURFACEORIENTATION:
      //#ifdef USE_V4R_V0
      //      return SurfaceOrientationProcessor::Ptr(new SurfaceOrientationProcessor(processorId, imgWidth, imgHeight, isStereo));
      //#else
      //            LOG4CXX_ERROR(factoryLogger, "SurfaceOrientationProcessor not available. Did you compile vision with V4R?");
      //#endif
      //      break;
      //    case SURFACECURVATURE:
      //#ifdef USE_V4R_V0
      //      return SurfaceCurvatureProcessor::Ptr(new SurfaceCurvatureProcessor(processorId, imgWidth, imgHeight, isStereo));
      //#else
      //      LOG4CXX_ERROR(factoryLogger, "SurfaceCurvatureProcessor not available. Did you compile vision with V4R?");
      //#endif
      //      break;
    case IKNSALIENCYMAP:
#ifdef USE_V4R_V0
      return IKNSaliencyMapProcessor::Ptr(new IKNSaliencyMapProcessor(processorId, imgWidth, imgHeight, isStereo));
#else
      LOG4CXX_ERROR(factoryLogger, "IKNSaliencyMapProcessor not available. Did you compile vision with V4R?");
#endif
      break;
    case COLORSALIENCYMAP:
#ifdef USE_V4R_V0
      return ColorProcessor::Ptr(new ColorProcessor(processorId, imgWidth, imgHeight, isStereo));
#else
      LOG4CXX_ERROR(factoryLogger, "ColorProcessor not available. Did you compile vision with V4R?");
#endif
      break;
    case SYMMETRYSALIENCYMAP:
#ifdef USE_V4R_V0
      return SymmetryProcessor::Ptr(new SymmetryProcessor(processorId, imgWidth, imgHeight, isStereo));
#else
      LOG4CXX_ERROR(factoryLogger, "SymmetryProcessor not available. Did you compile vision with V4R?");
#endif
      break;
    case LOCATIONSALIENCY:
#ifdef USE_V4R_V0
      return LocationSaliencyProcessor::Ptr(new LocationSaliencyProcessor(processorId, imgWidth, imgHeight, isStereo));
#else
      LOG4CXX_ERROR(factoryLogger, "LocationSaliencyProcessor not available. Did you compile vision with V4R?");
#endif
      break;
    case ORIENTATIONSALIENCY:
#ifdef USE_V4R_V0
      return OrientationProcessor::Ptr(new OrientationProcessor(processorId, imgWidth, imgHeight, isStereo));
#else
      LOG4CXX_ERROR(factoryLogger, "OrientationProcessor not available. Did you compile vision with V4R?");
#endif
      break;
    case MASTERSALIENCY:
      return MasterSaliencyProcessor::Ptr(new MasterSaliencyProcessor(processorId, imgWidth, imgHeight, isStereo));
    case PLANE:
      return PlaneProcessor::Ptr(new PlaneProcessor(processorId, imgWidth, imgHeight, isStereo));
    case COLORVALIDATOR:
      return ColorValidator::Ptr(new ColorValidator(processorId, imgWidth, imgHeight, isStereo));
    case DEFINITIONVALIDATOR:
      return DefinitionValidator::Ptr(new DefinitionValidator(processorId, imgWidth, imgHeight));
    case GLOBALFEATUREVALIDATOR:
      return GlobalFeatureValidator::Ptr(new GlobalFeatureValidator(processorId, imgWidth, imgHeight));
    case SIZEVALIDATOR:
      return SizeValidator::Ptr(new SizeValidator(processorId, imgWidth, imgHeight, isStereo));
    case SHAPEVALIDATOR:
#ifdef USE_V4R_V0
      return ShapeValidator::Ptr(new ShapeValidator(processorId, imgWidth, imgHeight, isStereo));
#else
      LOG4CXX_ERROR(factoryLogger, "ShapeValidator not available. Did you compile vision with V4R?");
#endif
      break;
    case SPATIALRELATIONVALIDATOR:
      return SpatialRelationValidator::Ptr(new SpatialRelationValidator(processorId, imgWidth, imgHeight, isStereo));
    case SURFACEMARKINGVALIDATOR:
#ifdef USE_V4R_V0
      return SurfaceMarkingValidator::Ptr(new SurfaceMarkingValidator(processorId, imgWidth, imgHeight, isStereo));
#else
      LOG4CXX_ERROR(factoryLogger, "SurfaceMarkingValidator not available. Did you compile vision with V4R?");
#endif
      break;
  }

  return ImageProcessor::Ptr();
}

ImageProcessor::ImageProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: VisionProcess(processorId, imgWidth, imgHeight),
is_stereo(isStereo) {
  ignoreOlderNotifications = true;

  logger = log4cxx::Logger::getLogger("ade.imgproc.ImageProcessor");
}

ImageProcessor::~ImageProcessor() {
}

void ImageProcessor::init() {
  ade::capture::Capture::registerForNotification(shared_from_this());
}

void ImageProcessor::cleanup() {
  ade::capture::Capture::unregisterForNotification(shared_from_this());
}
