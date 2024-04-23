/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.consultant;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class VisionReferenceJson {
    /**
   * Unique reference resolution id (e.g., pose_2).
   */
  final public String refId;
  /**
   * Free-variable in the list of properties is the object of interest.
   */
  final public String variable;
  /**
   * List of properties relevant to this reference.
   */
  final public List<String> properties;

  public VisionReferenceJson(String ref, String variable) {
    this.refId = ref;
    this.variable = variable;
    this.properties = new ArrayList<>();
  }

  public VisionReferenceJson(String ref, String variable, List<String> properties) {
    this(ref, variable);
    this.properties.addAll(properties);
  }

  public VisionReferenceJson(VisionReference visionReference) {
    this.refId = visionReference.refId.toString();
    this.variable = visionReference.variable.toString();
    this.properties = visionReference.properties.stream().map(ref -> ref.toString()).collect(Collectors.toList());
  }
}
