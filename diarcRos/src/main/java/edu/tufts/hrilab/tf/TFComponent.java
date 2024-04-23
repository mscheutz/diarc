/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tf;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.CoordinateFramesInterface;
import edu.tufts.hrilab.diarcros.common.RosConfiguration;
import edu.tufts.hrilab.diarcros.tf.TF;
import edu.tufts.hrilab.diarcros.util.Convert;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.vecmath.Matrix4d;
import java.util.ArrayList;
import java.util.List;

public class TFComponent extends DiarcComponent implements CoordinateFramesInterface {
  protected TF tf;
  protected RosConfiguration rosConfig = new RosConfiguration();
  private String baseFrame = "base_link";

  public TFComponent() {
    super();
  }

  @Override
  protected void init() {
    tf = new TF(rosConfig);
    tf.waitForNode();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("ns").longOpt("namespace").hasArg().argName("namespace").desc("Set a ROS namespace pointing to this ROS component. Default: empty.").build());
    options.add(Option.builder("tf_prefix").hasArg().argName("namespace").desc("Set a tf prefix. Note that this appears similar to a namespace, but ROS namespaces do not impact tf tree links (making this parameter necessary). Read the TF docs for more detail. Default: empty.").build());
    options.add(Option.builder("rosmasteruri").hasArg().argName("rosmasteruri").desc("Override ROS_MASTER_URI environment variable").build());
    options.add(Option.builder("baselink").hasArg().argName("base_link").desc("Override default TF base frame").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("namespace")) {
      rosConfig.namespace = cmdLine.getOptionValue("namespace");
    }
    if (cmdLine.hasOption("tf_prefix")) {
      rosConfig.tfPrefix = cmdLine.getOptionValue("tf_prefix");
      baseFrame = rosConfig.getPrefixedFrame(baseFrame);
    }
    if (cmdLine.hasOption("rosmasteruri")) {
      String uri = cmdLine.getOptionValue("rosmasteruri");
      rosConfig.setUriFromString(uri);
    }
    if (cmdLine.hasOption("baselink")) {
      baseFrame = rosConfig.getPrefixedFrame(cmdLine.getOptionValue("baselink"));
    }
  }

  @Override
  public Matrix4d getTransform(String dstFrame) {
    return tf.getTransform(baseFrame, dstFrame);
  }

  @Override
  public Matrix4d getTransform(String srcFrame, String dstFrame) {
    return tf.getTransform(srcFrame, dstFrame);
  }

  @Override
  public List<String> getCoordinateFrames() {
    return new ArrayList<>(tf.getTransformNames());
  }

  @Override
  public void addLocalStaticTransform(String parent, String child, Matrix4d transform) {
    edu.tufts.hrilab.diarcros.msg.geometry_msgs.TransformStamped transformStamped = Convert.convertToTransformStamped(transform);
    tf.addLocalStaticTransform(parent, child, transformStamped.getTransform().getTranslation(), transformStamped.getTransform().getRotation());
  }
}


