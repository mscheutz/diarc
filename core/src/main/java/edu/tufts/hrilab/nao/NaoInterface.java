/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.nao;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.interfaces.SpeechProductionInterface;
import edu.tufts.hrilab.interfaces.VelocityInterface;

import javax.vecmath.Point3d;
import java.util.List;

import ai.thinkingrobots.trade.TRADEService;

/**
 * The <code>NaoComponent</code> interface.
 */
public interface NaoInterface extends VelocityInterface, SpeechProductionInterface {

  /**
   * Query whether or not robot is moving.
   *
   * @return
   */
    @TRADEService
    @Action
    boolean isMoving();

  /**
   * Play the specified audio file
   *
   * @param fileName : the path of the file to play
   */
    @TRADEService
    void playFile(String fileName);


  /**
   * Move into specified posture.
   *
   * @param targetPosture
   * @return
   */
    @TRADEService
    @Action
    boolean goToPosture(String targetPosture);

  /**
   * Makes the robot move to the given pose in the ground plane, relative to
   * FRAME_ROBOT. This is a not a blocking call. TODO: this needs to be generalized
   * into a DIARC interface so that it's not specific to the nao component.
   *
   * @param x Distance along the X axis in meters.
   * @param y Distance along the Y axis in meters.
   * @param theta Rotation around the Z axis in radians [-3.1415 to 3.1415].
   * @return
   */
    @TRADEService
    @Action
    boolean moveTo(float x, float y, float theta);

  /**
   * Makes the robot move to the given pose in the ground plane, relative to
   * FRAME_ROBOT. This is a blocking call. TODO: this needs to be generalized
   * into a DIARC interface so that it's not specific to the nao component.
   *
   * @param x Distance along the X axis in meters.
   * @param y Distance along the Y axis in meters.
   * @param theta Rotation around the Z axis in radians [-3.1415 to 3.1415].
   * @return
   */
    @TRADEService
    @Action
    boolean moveToBlocking(float x, float y, float theta);

  //TODO:brad: refactor these to "setAngleInterpolation"?
  /**
   * Move NAO joint to a particular angle.
   * @param name - joint name to move
   * @param angle - 
   * @param time - how long to take to move to desired joint angle
   * @param isAbsolute
   */
    @TRADEService
    @Action
    void angleInterpolation(String name, float angle, float time, boolean isAbsolute);

  /**
   * Move NAO joints to a particular angle. A different time values can be used for each joint.
   * @param names - joint name to move
   * @param angles - angles for joints
   * @param times - how long to take to move to desired joint angles
   * @param isAbsolute
   */
    @TRADEService
    @Action
    void angleInterpolation(List<String> names, List<Float> angles, List<Float> times, boolean isAbsolute);

  /**
   * Move NAO joints to a particular angle. A single time value is used for all joints.
   * @param names - joint name to move
   * @param angles - angles for joints
   * @param time - how long to take to move to desired joint angles
   * @param isAbsolute
   */
    @TRADEService
    @Action
    void angleInterpolation(List<String> names, List<Float> angles, float time, boolean isAbsolute);

  /**
   * Get NAO joint angle.
   * @param name - joint name
   * @return
   */
    @TRADEService
    float getAngle(String name);
  
  /**
   * Points to the (x,y,z) location in the robot's base coordinate frame. Units
   * are in meters.
   * 
   * @param x
   * @param y
   * @param z
   * @return
   */
    @TRADEService
    @Action
    boolean pointTo(float x, float y, float z);

  /**
   * Rests robot's left/right arm by placing thr hands on its lap and turning the motors off.
   * Takes 1500ms to complete (blocking). Also disables hip motors!!!
   *
   * @return
   */
    @TRADEService
    @Action
    boolean restLeftArm();

    @TRADEService
    @Action
    boolean restRightArm();
  
    /**
   * Points head to the (x,y,z) location in the robot's base coordinate frame. Units
   * are in meters.
   * 
   * @param x
   * @param y
   * @param z
   * @return
   */
    @TRADEService
    @Action
    boolean pointHeadTo(float x, float y, float z);

  /**
   * Checks if there is floor support in front of the robot. TODO: this needs to
   * be generalized into a DIARC interface so that it's not specific to the nao
   * component.
   *
   * @return true if it's safe to move forward, false otherwise
   */
    @TRADEService
    @Action
    boolean checkFloorSupport();

  /**
   * Checks if there is an obstacle in front of the robot. TODO: this needs to
   * be generalized into a DIARC interface so that it's not specific to the nao
   * component.
   *
   * @return true if it's safe to move forward, false otherwise
   */
    @TRADEService
    @Action
    boolean checkObstacle();

  /**
   * Get current posture.
   *
   * @return
   */
    @TRADEService
    @Action
    String getPosture();

  /**
   * Get list of all possible postures.
   *
   * @return
   */
    @TRADEService
    @Action
    List<String> getPostureList();
  
  /**
   * Perform text-to-speech on input text.
   * @param text
   * @return
   */
    @TRADEService
    @Action
    boolean sayText(String text);

  /**
   * To save tts output to file instead of saying it outloud.
   * @param text
   * @param filename
   * @return
   */
    @TRADEService
    boolean sayToFile(String text, String filename);

  /**
   * Sets joint/actuator stiffness
   * @param name Name of joint/actuator. See nao doc.
   * @param stiffness Value between 0 and 1
   * @return
   */
    @TRADEService
    @Action
    boolean setStiffness(String name, float stiffness);
    
  /**
   * Puts the nao in the rest position, relaxing all motors.
   * @return
   */
    @TRADEService
    @Action
    boolean rest();

  /**
   * Opens the hand
   * @param name Name of joint/actuator (LHand or RHand).
   * @return
   */
    @TRADEService
    @Action
    boolean openHand(String name);

  /**
   * Closes the hand
   * @param name Name of joint/actuator (LHand or RHand).
   * @return
   */
    @TRADEService
    @Action
    boolean closeHand(String name);

  /**
   * Computes position of pixel in robot frame.
   * @param pX x position of pixel
   * @param pY y position of pixel
   * @param width Width of image
   * @param height Height of image
   * @param camID Camera ID (0 [top], 1 [bottom])
   * @return Position of pixel projected on floor/table.
   */
    @TRADEService
    @Action
    Point3d getPixelPositionInRobotFrame(int pX, int pY, int width, int height, int camID);

    @TRADEService
    @Action
    boolean sayAnimated(String annotatedText);

    @TRADEService
    boolean welcome();

    @TRADEService
    @Action
    boolean setBreathEnabled(String par, boolean enable);

    @TRADEService
    @Action
    boolean ledOn(String name);

    @TRADEService
    @Action
    boolean ledOff(String name);

    @TRADEService
    @Action
    boolean blink();
}
