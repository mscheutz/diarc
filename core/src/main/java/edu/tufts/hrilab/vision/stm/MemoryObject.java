/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.stm;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.util.Utilities;
import edu.tufts.hrilab.vision.util.PredicateHelper;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import java.awt.Rectangle;
import java.io.Serializable;
import java.math.BigDecimal;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * MemoryObject - A Java class-wrapper for the data to be passed between the
 * vision system and the rest of the Java system.
 * <p>
 * Changes to this class MUST be appropriately reflected in native classes
 * MemoryObjectInterface and MemoryObject, as well as
 * TrackedObjects.addMemoryObjectToSend(...)
 */
//TODO: change all member fields to private
//Currently, not thread safe.
public class MemoryObject implements Serializable {

  /**
   * Non-unique type ID.
   */
  private long typeId = -1L;
  /**
   * Unique MemoryObject token ID.
   */
  private long tokenId = -1L;
  /**
   * Frame number from which object was last updated.
   */
  private long frameNum = -1L;
  /**
   * Variable from initial search request that is satisfied by this
   * MemoryObject.
   */
  private Variable variable;
  /**
   * Unique identifier (e.g., token5, where 5 is the token id).
   */
  private Symbol identifier;
  /**
   * List of predicate descriptors.
   */
  private List<Term> descriptors = new ArrayList();
  /**
   * Current coordinate system that all values are relative to.
   * TODO: have this set dynamically.
   */
  private String currCoordinateFrame = "vision_frame";
  /**
   * Pixel coordinates (for drawing).
   */
  private Rectangle boundingBox = new Rectangle();     // 2D bounding box - in pixel coordinates
  /**
   * 3D dimensions of object (meters).
   */
  private Point3d dimensions = new Point3d(Double.NaN, Double.NaN, Double.NaN);
  /**
   * 2D pixel mask of object. Indices into image.
   */
  private int[] mask = new int[0];
  /**
   * 3D location in world coordinates (meters).
   */
  private Point3d location = new Point3d(Double.NaN, Double.NaN, Double.NaN);
  /**
   * Unit direction vector pointing to object from origin. TODO: use this in
   * place of pan/tilt
   */
  private Vector3d direction = new Vector3d(Double.NaN, Double.NaN, Double.NaN);
  /**
   * Detection confidence in the range [0 1].
   */
  private double detectionConfidence = 0;
  /**
   * Tracking confidence in the range [0 1].
   */
  private double trackingConfidence = 0;
  /**
   * 3D point cloud. [point-index][dim] where dim 0=x, 1=y, 2=z
   */
  private double[][] pointCloud = new double[0][0];
  /**
   * 3D mesh face indices. Indices into pointCloud array.
   */
  private int[][] faceIndices = new int[0][0];
  /**
   * Orientations for grasp info. This is only relevant for grasp MemoryObjects
   * (usually has descriptor grasp_point(X)). This probably shouldn't be in this
   * class (maybe a MemoryObject subclass??) as it doesn't apply to all
   * MemoryObjects.
   */
  private double[][] orientations = new double[0][0];
  /**
   * Transform from base to vision coordinate frame.
   */
  private Matrix4d baseTransform = null;
  /**
   * Primitive MemoryObject shapes for use when instantiating MemortObjects in
   * Gazebo.
   */
  private List<MemoryPrimitive> primitives = new ArrayList<>();
  /**
   * Log4j2 logger.
   */
  private static Logger log = LoggerFactory.getLogger(MemoryObject.class);

  //////////////////////////////////////////////////////////////////
  ////////////////////// START: New Stuffs /////////////////////////
  //////////////////////////////////////////////////////////////////
  /**
   * Children MemoryObjects that make up scene graph (nodes). Hashed by
   * MemoryObject's variable.
   */
  private final Map<Symbol, List<MemoryObject>> children = new HashMap<>();

  /**
   * Related memory objects (edges) in the scene graph (hashed by relationship
   * name (e.g., on, near, etc.)).
   */
  private final Map<String, List<MemoryObjectRelation>> relations = new HashMap<>();

  /**
   * Add a child MemoryObject to the scene graph.
   *
   * @param child
   */
  public void addChild(MemoryObject child) {
    Variable var = child.getVariable();
    if (children.containsKey(var)) {
      children.get(var).add(child);
    } else {
      List<MemoryObject> moList = new ArrayList<>();
      moList.add(child);
      children.put(var, moList);
    }
  }

  /**
   * Add new relation.
   *
   * @param confidence    [0 1] value
   * @param descriptor    Term description of relation
   * @param relatedObject other MemoryObject in relation
   * @param reciprocate   if relation should be added to relatedObject too
   */
  public void addRelation(float confidence, Term descriptor, MemoryObject relatedObject, boolean reciprocate) {
    addRelation(confidence, descriptor, relatedObject);

    if (reciprocate) {
      relatedObject.addRelation(confidence, descriptor, this, false);
    }
  }

  /**
   * Convenience method to populate relations from native side.
   *
   * @param confidence
   * @param descriptor
   * @param relatedObject
   * @param reciprocate
   */
  public void addRelation(float confidence, String descriptor, MemoryObject relatedObject, boolean reciprocate) {
    log.debug("[addRelation] descriptor: " + descriptor);
    Predicate descriptorPred = PredicateHelper.createPredicate(descriptor);
    addRelation(confidence, descriptorPred, relatedObject, reciprocate);
  }

  /**
   * Add new relation to this MemoryObject only.
   *
   * @param confidence    [0 1] value
   * @param descriptor    Term description of relation
   * @param relatedObject other MemoryObject in relation
   */
  private void addRelation(float confidence, Term descriptor, MemoryObject relatedObject) {
    if (descriptor.getOrderedVars().size() != 2) {
      log.error("[addRelation] descriptor must have exactly two variable matching two MO's variable names.");
      return;
    }

    // update local relation info
    String relationName = descriptor.getName();
    MemoryObjectRelation newRelation = new MemoryObjectRelation(confidence, descriptor, relatedObject);
    if (relations.containsKey(relationName)) {
      // check if relation already exists before adding it
      boolean relationAlreadyExists = false;
      List<MemoryObjectRelation> relationList = relations.get(relationName);
      for (MemoryObjectRelation relation : relationList) {
        if (relation.getRelatedObject() == relatedObject
                && relation.getDescriptor() == descriptor
                && relation.getConfidence() == confidence) {
          relationAlreadyExists = true;
          break;
        }
      }
      if (!relationAlreadyExists) {
        relationList.add(newRelation);
      }
    } else {
      List<MemoryObjectRelation> relationList = new ArrayList<>();
      relationList.add(newRelation);
      relations.put(relationName, relationList);
    }
  }

  /**
   * Get all immediate children of this MemoryObject.
   *
   * @return all immediate children in a List
   */
  public List<MemoryObject> getChildren() {
    List<MemoryObject> childList = new ArrayList<>();
    for (List<MemoryObject> childrenByVar : children.values()) {
      childList.addAll(childrenByVar);
    }
    return childList;
  }

  /**
   * Get all relations this MemoryObject has.
   *
   * @return list of MemoryObjectRelations (potentially empty)
   */
  public List<MemoryObjectRelation> getRelations() {
    List<MemoryObjectRelation> relationsList = new ArrayList<>();
    for (List<MemoryObjectRelation> relationsVals : relations.values()) {
      relationsList.addAll(relationsVals);
    }
    return relationsList;
  }

  /**
   * Get all relations this MemoryObject has that match the specified relation.
   *
   * @param relation
   * @return list of MemoryObjectRelations (potentially empty)
   */
  public List<MemoryObjectRelation> getRelations(Term relation) {
    List<MemoryObjectRelation> returnRelations;
    if (relations.containsKey(relation.getName())) {
      returnRelations = new ArrayList<>(relations.get(relation.getName()));
    } else {
      returnRelations = new ArrayList<>();
    }
    return returnRelations;
  }

  /**
   * Find the MemoryObject in the scene graph matching the identifier. Returns
   * null if no match is found.
   *
   * @param identifier
   * @return
   */
  public MemoryObject getMemoryObject(Symbol identifier) {
    Queue<MemoryObject> frontier = new ArrayDeque<>();
    frontier.add(this);
    while (!frontier.isEmpty()) {
      MemoryObject currMO = frontier.poll();
      if (identifier.equals(currMO.getIdentifier())) {
        return currMO;
      }
      frontier.addAll(currMO.getChildren());
    }

    return null;
  }

  //////////////////////////////////////////////////////////////////
  ////////////////////// END: New Stuffs ///////////////////////////
  //////////////////////////////////////////////////////////////////

  /**
   * Default constructor.
   */
  public MemoryObject() {
  }

  /**
   * Deep copy constructor.
   *
   * @param rhs
   */
  public MemoryObject(MemoryObject rhs) {
    this.frameNum = rhs.getFrameNum();
    this.boundingBox = rhs.getBoundingBox();
    this.location = rhs.getLocation();
    this.direction = rhs.getDirection();
    this.dimensions = rhs.getDimensions();
    this.detectionConfidence = rhs.getDetectionConfidence();
    this.trackingConfidence = rhs.getTrackingConfidence();
    this.baseTransform = rhs.getBaseTransform();
    this.descriptors = rhs.getDescriptors();
    this.currCoordinateFrame = rhs.getCoordinateFrame();
    this.primitives = rhs.getPrimitives();
  }

  /**
   * If MemoryObject is a valid STM MemoryObject (i.e., have the appropriate
   * fields been set). Currently, this returns true if the ID has been set, and
   * is no longer "-1".
   *
   * @return
   */
  public boolean isValid() {
    return (tokenId != -1 && typeId != -1);
  }

  public ArrayList<String> getStringsForVisualization() {
    ArrayList<String> result = new ArrayList();
    result.add(descriptors.toString() + tokenId);
    result.add("detectConfs=" + Double.toString(roundToXDigits(detectionConfidence, 3)));
    result.add("trackConf=" + Double.toString(roundToXDigits(trackingConfidence, 3)));
    result.add("typeID=" + Long.toString(typeId));
    return result;
  }

  // to round a double to two digits (for better readability of debugging text)
  public static double roundToXDigits(double num, int x) {
    BigDecimal bd = new BigDecimal(num);
    bd = bd.setScale(x, BigDecimal.ROUND_UP);
    return bd.doubleValue();
  }

  @Override
  public String toString() {
    return String.valueOf(tokenId);
  }

  public String getDisplayString() {
    StringBuilder sb = new StringBuilder();
    sb.append("<html>");
    sb.append("Frame Number: ").append(frameNum).append(".<br>");
    sb.append("TokenID: ").append(tokenId).append(". TypeID: ").append(typeId).append(".<br>");
    sb.append("Description: ").append(descriptors).append(".<br>");
    sb.append("Relations: ").append(getRelationInfo()).append(".<br>");
    sb.append("DetectConf: ").append(Double.toString(roundToXDigits(detectionConfidence, 3)));
    sb.append(". TrackConf:").append(Double.toString(roundToXDigits(trackingConfidence, 3))).append(".<br>");
    sb.append("B-Box: [").append(boundingBox.x).append(", ").append(boundingBox.y)
            .append(", ").append(boundingBox.width).append(", ").append(boundingBox.height).append("].<br>");
    sb.append("Location: ").append(location == null ? "[none]" : location).append(".<br>");
    sb.append("Direction: ").append(direction == null ? "[none]" : direction).append(".<br>");
    sb.append("Dimensions: ").append(dimensions == null ? "[none]" : dimensions).append(".<br>");
//    sb.append("Transform: ").append(baseTransform == null ? "[none]" : baseTransform).append(".<br>");
    sb.append("</html>");

    return sb.toString();
  }

  private String getRelationInfo() {
    StringBuilder sb = new StringBuilder();
    for (String relationStr : relations.keySet()) {
      sb.append(relationStr).append(" {");
      for (MemoryObjectRelation relation : relations.get(relationStr)) {
        sb.append(relation.getRelatedObject().getTokenId()).append(" ");
      }
      sb.append("}");
    }
    return sb.toString();
  }

  public void printSceneGraph() {
    StringBuilder sb = new StringBuilder("Scene graph:\n");
    Queue<MemoryObject> frontier = new ArrayDeque<>();
    frontier.add(this);
    while (!frontier.isEmpty()) {
      MemoryObject currNode = frontier.poll();
      frontier.addAll(currNode.getChildren());
      sb.append(currNode.toString()).append("\n");
    }
    log.info(sb.toString());
  }

  public void setTypeId(long typeId) {
    this.typeId = typeId;
  }

  public void setTokenId(long tokenId) {
    this.tokenId = tokenId;
    identifier = new Symbol("token" + tokenId);
  }

  public void setFrameNum(long fNum) {
    frameNum = fNum;
  }

  public void setVariable(Variable variable) {
    this.variable = variable;
  }

  public void setVariable(String variableName) {
    this.variable = new Variable(variableName, PredicateHelper.varType);
  }

  @Deprecated
  public void setConfidence(double conf) {
    //detectionConfidence = conf;
    trackingConfidence = conf;
  }

  public void setDetectionConfidence(double conf) {
    detectionConfidence = conf;
  }

  public void setTrackingConfidence(double conf) {
    trackingConfidence = conf;
  }

  public void setLocation(double x, double y, double z) {
    location = new Point3d(x, y, z);
  }

  public void setDirection(double x, double y, double z) {
    direction = new Vector3d(x, y, z);
  }

  public void setBoundingBox(int x, int y, int width, int height) {
    boundingBox = new Rectangle(x, y, width, height);
  }

  public void setDimensions(double x, double y, double z) {
    dimensions = new Point3d(x, y, z);
  }

  /**
   * Set the transform to go from vision to base coordinate system. Should only
   * be set once, from native side.
   *
   * @param transform
   */
  public void setBaseTransform(Matrix4d transform) {
    if (transform == null) {
      log.warn("[setBaseTransform] transform NULL!");
    }
    baseTransform = transform;
  }

  /**
   * Set the transform to go from vision to base coordinate system. Should only
   * be set once, from native side.
   *
   * @param transform
   */
  public void setBaseTransform(double[] transform) {
    if (transform == null) {
      log.error("[setBaseTransform] transform NULL!");
      return;
    }
    if (transform.length != 16) {
      log.error("[setBaseTransform] transform not correct size!");
      return;
    }
    if (null != baseTransform) {
      log.error("[setBaseTransform] transform already set!");
      return;
    }

    //init transform
    setBaseTransform(new Matrix4d(transform));
  }

  /**
   * Set number of points in 3D point cloud.
   *
   * @param size
   */
  public void setNumPoints(int size) {
    pointCloud = new double[size][3];
  }

  /**
   * Add point to point cloud. Assumes setNumPoints has been called to allocate
   * space for points.
   *
   * @param index
   * @param x
   * @param y
   * @param z
   */
  public void addPoint(int index, double x, double y, double z) {
    pointCloud[index][0] = x;
    pointCloud[index][1] = y;
    pointCloud[index][2] = z;
  }

  /**
   * Set number of faces in 3D mesh.
   *
   * @param size
   */
  public void setNumFaces(int size) {
    //don't allocate space for second dimension bc we don't know num of vertices
    //in polygons (usually 3, but can be more)
    faceIndices = new int[size][];
  }

  /**
   * Add face to 3D mesh. Assumes setNumFaces has been called to allocate space
   * for faces. Faces can contain three or more vertices.
   *
   * @param face
   */
  public void addFace(int index, int[] face) {
    if (face.length < 3) {
      log.error("[addFace] not enough vertices!");
      return;
    }

    faceIndices[index] = face;
  }

  /**
   * Set number of orientations.
   *
   * @param size
   */
  public void setNumOrientations(int size) {
    orientations = new double[size][4];
  }

  /**
   * Add orientation. Assumes setNumOrientations has been called to allocate
   * space for orientations.
   *
   * @param index
   * @param x
   * @param y
   * @param z
   * @param w
   */
  public void addOrientation(int index, double x, double y, double z, double w) {
    log.debug(String.format("[addOrientation]: index: %d. orientation: (%f,%f,%f,%f).", index, x, y, z, w));
    orientations[index][0] = x;
    orientations[index][1] = y;
    orientations[index][2] = z;
    orientations[index][3] = w;
  }

  /**
   * Set number of indices in mask.
   *
   * @param size
   */
  public void setNumMaskIndices(int size) {
    mask = new int[size];
  }

  /**
   * Add index to mask. Assumes setNumMaskIndices has been called to allocate
   * space for mask.
   *
   * @param indexNumber index into mask data structure
   * @param index       image index value
   */
  public void addMaskIndex(int indexNumber, int index) {
    mask[indexNumber] = index;
  }

  public void setCoordinateFrame(String frame) {
    currCoordinateFrame = frame;
  }

  public void setPrimitives(List<MemoryPrimitive> primitives) {
    this.primitives = primitives;
  }

  @Deprecated
  public void addDescriptor(final Term descriptor) {
    addDescriptor(descriptor, 1.0f);
  }

  public void addDescriptor(final Term descriptor, float confidence) {
    //TODO: save confidence!!
    //todo: this previously set all variables to have type :object. updates needed in other scene generators to reflect this change
//    PredicateHelper.setVariableType(descriptor, PredicateHelper.varType);
    if (descriptor.getOrderedVars().size() != 1 || !descriptor.getOrderedVars().get(0).equals(variable)) {
      log.error("[addDescriptor] descriptor (" + descriptor + ") must have exactly one variable and match variable name: " + variable);
      return;
    }

    descriptors.add(descriptor);
  }

  /**
   * Convenience method for adding descriptors from native side.
   *
   * @param descriptor String that originated from Predicate
   */
  public void addDescriptor(String descriptor, float confidence) {
    log.debug("[addDescriptor] descriptor: " + descriptor + " confidence: " + confidence);

    Predicate descriptorPred = PredicateHelper.createPredicate(descriptor);
    addDescriptor(descriptorPred, confidence);
  }

  /**
   * Return search type ID.
   *
   * @return
   */
  public long getTypeId() {
    return typeId;
  }

  /**
   * Get unique token ID.
   *
   * @return
   */
  public long getTokenId() {
    return tokenId;
  }

  public long getFrameNum() {
    return frameNum;
  }

  public Symbol getIdentifier() {
    return identifier;
  }

  public Variable getVariable() {
    return variable;
  }

  @Deprecated
  public double getConfidence() {
    //return (detectionConfidence * trackingConfidence);
    return Double.NaN;
  }

  public double getDetectionConfidence() {
    return detectionConfidence;
  }

  public double getTrackingConfidence() {
    return trackingConfidence;
  }

  public Rectangle getBoundingBox() {
    return new Rectangle(boundingBox);
  }

  public Point3d getDimensions() {
    return new Point3d(dimensions);
  }

  @Deprecated
  public double getDist() {
    double distance = Double.NaN;
    if (location != null && !Double.isNaN(location.x) && !Double.isNaN(location.y) && !Double.isNaN(location.z)) {
      distance = Math.sqrt(Math.pow(location.x, 2) + Math.pow(location.y, 2) + Math.pow(location.z, 2));
    }
    return distance;
  }

  public Point3d getLocation() {
    return new Point3d(location);
  }

  public Vector3d getDirection() {
    return new Vector3d(direction);
  }

  /**
   * Get pan angle (radians). This assumes camera coordinate frame (i.e., +pan
   * is -x, +tilt is -y).
   *
   * @return angle in radians
   * @deprecated
   */
  @Deprecated
  public double getPan() {
    double pan = Double.NaN;
    if (direction != null && !Double.isNaN(direction.x) && !Double.isNaN(direction.y) && !Double.isNaN(direction.z)) {
      pan = Math.atan2(-direction.x, direction.z);
    }
    return pan;
  }

  /**
   * Get tilt angle (radians). This assumes camera coordinate frame (i.e., +pan
   * is -x, +tilt is -y).
   *
   * @return angle in radians
   * @deprecated
   */
  @Deprecated
  public double getTilt() {
    double tilt = Double.NaN;
    if (direction != null && !Double.isNaN(direction.x) && !Double.isNaN(direction.y) && !Double.isNaN(direction.z)) {
      tilt = Math.atan2(-direction.y, direction.z);
    }
    return tilt;
  }

  public double[][] getPointCloud() {
    return pointCloud;
  }

  public int[][] getFaceIndices() {
    return faceIndices;
  }

  public double[][] getOrientations() {
    return orientations;
  }

  public int[] getMaskIndices() {
    return mask;
  }

  public Matrix4d getBaseTransform() {
    return new Matrix4d(baseTransform);
  }

  public List<Term> getDescriptors() {
    return new ArrayList<>(descriptors);
  }

  public boolean containsDescriptors(final List<Term> descriptors) {
    for (Term descriptor : descriptors) {
      if (!containsDescriptor(descriptor)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Check if this MemoryObject contains the descriptor. If compared Terms both
   * have a Variable as an argument, the Variable does not need to be equal. For
   * example, on(X,Y) matches on(Y,Z).
   *
   * @param descriptor
   * @return
   */
  public boolean containsDescriptor(final Term descriptor) {
    for (Term currDescriptor : descriptors) {
      if (Utilities.predicatesMatch(currDescriptor, descriptor)) {
        return true;
      }
    }
    return false;
  }

  public String getCoordinateFrame() {
    return currCoordinateFrame;
  }

  public List<MemoryPrimitive> getPrimitives() {
    return primitives;
  }

  public boolean transformToBase() {
    log.trace("transforming to base: " + baseTransform);
    log.trace("curent coordinate frame: " + currCoordinateFrame);
    if (baseTransform != null) {
      if (currCoordinateFrame.equalsIgnoreCase("vision_frame") || currCoordinateFrame.equalsIgnoreCase("camera_frame")) {
        return transform(new Matrix4d(baseTransform), "base_link");
      } else if (currCoordinateFrame.equalsIgnoreCase("base_link")) {
        return true;
      }
    }

    return false;
  }

  public boolean transformToVision() {
    if (baseTransform != null) {
      if (currCoordinateFrame.equalsIgnoreCase("vision_frame")) {
        return true;
      } else if (currCoordinateFrame.equalsIgnoreCase("base_link")) {
        Matrix4d inv_transform = new Matrix4d();
        baseTransform.invert(inv_transform);
        return transform(inv_transform, "vision_frame");
      }
    }

    return false;
  }

  public boolean transform(Matrix4d transform, String dest) {
    log.trace("[transform] method entered with transform: " + transform);

    if (location != null && !Double.isNaN(location.x) && !Double.isNaN(location.y) && !Double.isNaN(location.z)) {
      // transform location
      log.trace(String.format("pre-transform location: (%f, %f, %f)", location.x, location.y, location.z));
      transform.transform(location);
      log.trace(String.format("post-transform location: (%f, %f, %f)", location.x, location.y, location.z));

      // transform direction
      direction.x = location.x;
      direction.y = location.y;
      direction.z = location.z;
      direction.normalize();
    } else if (direction != null && !Double.isNaN(direction.x) && !Double.isNaN(direction.y) && !Double.isNaN(direction.z)) {
      // transform direction
      log.trace(String.format("transform: %s", transform.toString()));
      log.trace(String.format("pre-transform direction: (%f, %f, %f)", direction.x, direction.y, direction.z));
      //Vector3d invDirection = new Vector3d(direction);
      transform.transform(direction);
      log.trace(String.format("post-transform direction: (%f, %f, %f)", direction.x, direction.y, direction.z));
//      transform.invert();
//      transform.transform(invDirection);
//      log.trace(String.format("post-inv-transform direction: (%f, %f, %f)", invDirection.x, invDirection.y, invDirection.z));
//      log.trace(String.format("inv-transform: %s", transform.toString()));
    }

    // transform point cloud
    Point3d p = new Point3d();
    for (double[] v : pointCloud) {
      p.x = v[0];
      p.y = v[1];
      p.z = v[2];
      transform.transform(p);
      v[0] = p.x;
      v[1] = p.y;
      v[2] = p.z;
    }

    // transform orientation(s)
    for (double[] o : orientations) {
      Matrix4d orientMat = new Matrix4d(new Quat4d(o), new Vector3d(), 1.0);
      Matrix4d newMat = new Matrix4d();
      newMat.mul(transform, orientMat);
      Quat4d orient = new Quat4d();
      newMat.get(orient);
      o[0] = orient.x;
      o[1] = orient.y;
      o[2] = orient.z;
      o[3] = orient.w;
    }

    // transform primitives
    for (MemoryPrimitive primitive : primitives) {
      primitive.pose.mul(transform);
    }

    // trasform children
    for (List<MemoryObject> childList : children.values()) {
      for (MemoryObject child : childList) {
        Matrix4d tmpTransform = new Matrix4d(transform);
        child.transform(tmpTransform, dest);
      }
    }

    // TODO: is this right?
    // update transform to base
    if (baseTransform != null) {
      //baseTransform.mul(transform);
      transform.invert();
      baseTransform.mul(transform);
    }

    // TODO: keep track of passed transform if not Vision or Base transform,
    // so that transofrmToVision and transformToBase are always valid!
    log.trace("[transform] done.");
    currCoordinateFrame = dest;

    return true;
  }
}
