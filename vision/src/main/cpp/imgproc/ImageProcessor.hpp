/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef IMAGEPROCESSOR_HPP
#define IMAGEPROCESSOR_HPP

#include "visionproc/VisionProcess.hpp"

enum ImageProcessorType {
  SIFT,
  CHANGEDETECTION,
  OPTICALFLOW,
  RELATIVEHEIGHT, // computes height of the objects relative to the main plane
  SURFACEORIENTATION, // detect surfaces parallel to the given plane
  SURFACECURVATURE, // detect surfaces curvature
  IKNSALIENCYMAP, // ikn saliency map
  COLORSALIENCYMAP, // distance in color space
  SYMMETRYSALIENCYMAP, // symmentry map
  LOCATIONSALIENCY, // location
  ORIENTATIONSALIENCY, // orientation
  MASTERSALIENCY, // master saliency map
  PLANE, // table plane detection
  COLORVALIDATOR, // color validation
  DEFINITIONVALIDATOR, // relabels scene graph according to definition (partOf(orange,knife) --> handle)
  GLOBALFEATUREVALIDATOR, // surface shape/texture/marking validation
  SHAPEVALIDATOR, // shape validation
  SPATIALRELATIONVALIDATOR, // spatial relation between object parts (e.g., near, on, etc)
  SURFACEMARKINGVALIDATOR, // surface shape/texture/marking validation
  SIZEVALIDATOR // global size validation (e.g., small, medium, large)
};

class ImageProcessor : public VisionProcess {
public:
  typedef boost::shared_ptr<ImageProcessor> Ptr;
  typedef boost::shared_ptr<const ImageProcessor> ConstPtr;

  /**
   * Factory method to construct requested ImageProcessor.
   * @param type ImageProcessorType
   * @param processorId VisionProcessor ID
   * @param imgwidth image width
   * @param imgheight image height
   * @param isStereo if operating in stereo configuration
   * @return ImageProcessor::Ptr
   */
  static ImageProcessor::Ptr get(const ImageProcessorType type, const long long& processorId, const unsigned int imgwidth, const unsigned int imgheight, const bool isStereo);

  virtual ~ImageProcessor();
  
  /**
   * ImageProcessor uses this method to register for capture notifications.
   * If overriding class needs capture notifications, either call 
   * ImageProcessor::init explicity or register for capture notifications.
   */
  virtual void init();
  
  /**
   * ImageProcessor uses this method to un-register for capture notifications.
   * If overriding class needs to un-register for capture notifications, either call 
   * ImageProcessor::cleanup explicity or un-register for capture notifications.
   */
  virtual void cleanup();

protected:
  ImageProcessor(const long long& processorId, const unsigned int imgwidth,
          const unsigned int imgheight, const bool isStereo);

  const bool is_stereo;
  
  //! For logging in factory method only.
  static log4cxx::LoggerPtr factoryLogger;
};

#endif  //IMAGEPROCESSOR_HPP
