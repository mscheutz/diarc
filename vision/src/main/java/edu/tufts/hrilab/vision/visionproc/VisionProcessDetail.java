/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.visionproc;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Base class to be used by VisionProcesses to keep track of any core info
 * used to instantiate a VisionProcess.
 *
 * @author evankrause
 */

//Thread-safe

//TODO: see if addDependency should be protected ?
public class VisionProcessDetail<T extends Enum<T>> {
  private final T type;
  private final int img_width;
  private final int img_height;
  private final Set<ImageProcessorType> dependencies = new HashSet<>(); //image proc types this depends on (e.g., sift, blort detector, etc.)
  private final Set<Predicate> searchDependencies = new HashSet<>(); //predicate based dependencies (e.g., "face(X)")

  public VisionProcessDetail(final T detectorType, final int imgWidth, final int imgHeight) {
    type = detectorType;
    img_width = imgWidth;
    img_height = imgHeight;
  }

  @Override
  public String toString() {
    return type.toString().toLowerCase();
  }

  public T getType() {
    return type;
  }

  public int getWidth() {
    return img_width;
  }

  public int getHeight() {
    return img_height;
  }

  public synchronized void addDependency(ImageProcessorType dependency) {
    if (!dependencies.contains(dependency)) {
      dependencies.add(dependency);
    }
  }

  public synchronized void addSearchDependency(Predicate searchPredicate) {
    searchDependencies.add(searchPredicate);
  }

  public synchronized List<ImageProcessorType> getDependencies() {
    return new ArrayList<>(dependencies);
  }

  public synchronized List<Predicate> getSearchDependencies() {
    return new ArrayList<>(searchDependencies);
  }

}


