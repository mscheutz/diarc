/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.imu;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.cli.Option;
import org.ros.node.topic.Subscriber;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.namespace.GraphName;
import org.ros.message.MessageListener;
import org.ros.message.Time;

import java.io.FileNotFoundException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.lang.Math;
import java.util.HashMap;
import java.util.List;
import java.io.FileReader;
import java.util.Map;

import com.google.gson.Gson;

import edu.tufts.hrilab.diarc.DiarcComponent;
import org.apache.commons.cli.CommandLine;

/**
 * FloorCalculatorComponent
 * Cameron Yuen
 *
 * This class allows any IMU equipped robot to calculate what imu it's traveling to on the
 * elevator. It uses the IMU, vertical linear acceleration data to
 * determine its vertical displacement.
 */
public class FloorCalculatorComponent extends DiarcComponent {
  /* ROS Connectivity Variables */
  private ConnectedNode node;

  /* Calculate imu variables */
  private Double runningAccelerationAvg;
  private int accelerationCount;
  private double currVelocity;
  private Boolean isMoving;
  private Double verticalDisplacement;
  private Double gravity; /* NOTE: this is average gravity observed from the IMU sensor */
  private Double thresholdVelocity;
  private Double baseTime;
  private Double[] floorHeights;
  private Integer floorNum;

  public class FloorData {
    private Double gravity; /* NOTE: this is average gravity observed from the IMU sensor */
    private Double thresholdVelocity;
    private Double[] floorHeights;

    public Double getGravity() {
      return gravity;
    }

    public Double getThresholdVelocity() {
      return thresholdVelocity;
    }

    public Double[] getFloorHeights() {
      return floorHeights;
    }

    public void setGravity(Double gravity) {
      this.gravity = gravity;
    }

    public void setThresholdVelocity(Double thresholdVelocity) {
      this.thresholdVelocity = thresholdVelocity;
    }

    public void setFloorHeights(Double[] floorHeights) {
      this.floorHeights = floorHeights;
    }
  }

  /**
   * The constructor sets all the variables for calculating the displacement
   * to default values.
   * NOTE: possible issue is the default value of verticalDisplacement;
   * this value depends on the imu the robot starts on, which can get screwed
   * up if the robot ever restarts
   *
   * It also instantiates the nodeMain for ROS, creates a subscriber to /imu,
   * allowing us to obtain acceleration and time stamp information
   */
  public FloorCalculatorComponent() {
    runningAccelerationAvg = 0.0;
    accelerationCount = 0;
    currVelocity = 0;
    isMoving = false;
    verticalDisplacement = 0.0;
    floorNum = null; /* needs to be configured to whatever imu you start on */

    NodeMain nodeMain = new AbstractNodeMain() {
      /**
       *  Obtains the node name of this new imu calculator node
       * @return
       */
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("ade/imu/floor_calculator_node");
      }

      /**
       *  Sets up subscriber to /imu, and has it listen; also implements the
       *  onNewMessage callback
       *
       * @param connectedNode
       */
      @Override
      public void onStart(ConnectedNode connectedNode) {
        node = connectedNode;
        Subscriber<sensor_msgs.Imu> IMUSub = node.newSubscriber("imu", sensor_msgs.Imu._TYPE);

        IMUSub.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
          /**
           *  Is the callback function whenever the subscriber sees a new message;
           *  constantly calculates running acceleration averages over a 0.4 second timeframe,
           *  while also checking if the elevator is moving up or down.
           *
           *  If running average acceleration detects non-gravity acceleration, it calculates
           *  velocity to obtain the change in displacement.
           *
           *  It then uses the change in displacement to find the imu its currently on.
           *
           * @param msg
           */
          @Override
          public void onNewMessage(sensor_msgs.Imu msg) {
            Double currentAcceleration = msg.getLinearAcceleration().getZ() - gravity;
            Time stamp = msg.getHeader().getStamp();
            Double currentTime = stamp.secs + (stamp.nsecs / 1_000_000_000.0);

            handleAcceleration(currentAcceleration, currentTime);
          }
        });
      }
    };

    /* ROS Connectivity and NodeMain setup and execution */
    try {
      URI ros_master_uri = new URI(System.getenv("ROS_MASTER_URI"));
      String host = InetAddressFactory.newNonLoopback().getHostAddress();
      NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, ros_master_uri);
      NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      nodeMainExecutor.execute(nodeMain, nodeConfiguration);
    } catch (URISyntaxException e) {
      log.error("Error trying to create URI: " + e);
    }
  }

  /**
   * Handles the running acceleration averages over 0.4 seconds and indicates if we've
   * begun moving; if we've started moving, pass control over to adjustDisplacement()
   *
   * @param currentAcceleration
   * @param currentTime
   */
  protected void handleAcceleration(Double currentAcceleration, Double currentTime) {
        /* Should only run once -- when baseTime is uninitialized and needs a reference
           time to calculate changes in time */
    if (baseTime == null) {
      baseTime = currentTime;
    } else {
      /* Calculates the running average every 0.4 seconds */
      double timeFrame = currentTime - baseTime;
      runningAccelerationAvg += currentAcceleration;
      accelerationCount++;
      if (timeFrame >= 0.4) {
        runningAccelerationAvg /= accelerationCount;
                /* starts collecting and calculating displacement if robot detects
                   upward or downwards movement */
        if (runningAccelerationAvg > 0.1 || runningAccelerationAvg < -0.1) {
          isMoving = true;
        }

                /* if you're collecting, do double integration on the average acceleration
                   to obtain change in displacement */
        if (isMoving) {
          adjustDisplacement(timeFrame);
        }
        runningAccelerationAvg = 0.0;
        accelerationCount = 0;
        baseTime = currentTime;
      }
    }
  }

  /**
   * Changes the verticalDisplacement based on the acceleration averages through double
   * integration; indicates stop moving when the velocity is virtually 0
   *
   * @param timeFrame
   */
  protected void adjustDisplacement(Double timeFrame) {
    currVelocity += runningAccelerationAvg * timeFrame;
    verticalDisplacement += currVelocity * timeFrame;
        /* translation for below: if you've stopped, find what
           imu you're on */
    if (Math.abs(currVelocity) <= thresholdVelocity) {
      log.info("Stopped at: " + verticalDisplacement + " m");
      currVelocity = 0.0;
      isMoving = false;
      double minDifference = 1000.0;
      for (int i = 0; i < floorHeights.length; i++) {
        double absoluteDifference = Math.abs(floorHeights[i] - verticalDisplacement);
        if (absoluteDifference < minDifference) {
          minDifference = absoluteDifference;
          floorNum = i;
        }
      }
      verticalDisplacement = floorHeights[floorNum];
      if (floorNum == 0) {
        floorNum = null;
      }
    }
  }

  /**
   * Adds the config parameter to FloorCalculator which takes a .json file.
   *
   * @return
   */
  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("config").hasArg().argName("config").desc("Set the config file.").build());
    return options;
  }

  /**
   * Parse the arguments in the command line; look for "config" and a valid .json file
   *
   * This .json file should include:
   * gravity, thresholdVelocity, and floorHeights
   *
   * @param cmdLine
   */
  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("config")) {
      String filename = Resources.createFilepath("config/edu/tufts/hrilab/imu/", cmdLine.getOptionValue("config"));
      try {
        FileReader reader = new FileReader(filename);
        Gson gson = new Gson();
        FloorData data = gson.fromJson(reader, FloorData.class);

        gravity = data.getGravity();
        thresholdVelocity = data.getThresholdVelocity();
        floorHeights = data.getFloorHeights();
      } catch (FileNotFoundException e) {
        log.error("Error loading config file: {}", filename, e);
      }
    }
  }

  /**
   * Retrieves what imu the robot is currently on.
   *
   * @return
   */
  @TRADEService
  @Action
  public int getFloorNum() {
    return floorNum;
  }

  /**
   * Sets the current imu number to the one given as an argument.
   *
   * @param floorNum
   */
  @TRADEService
  @Action
  public void setFloorNum(int floorNum) {
    // TODO: make this work for all buildings; this is currently designed for  Cummings
    this.floorNum = floorNum;
    if (floorNum == -1) {
      this.verticalDisplacement = floorHeights[0];
    } else {
      this.verticalDisplacement = floorHeights[floorNum];
    }
  }

  /**
   * Wait up to timeout for the target floor to be reached.
   * @param targetFloorNum
   * @return
   */
  @TRADEService
  @Action
  public Justification waitForFloor(int targetFloorNum) {
    long timeout = 100000;
    long startTime = System.currentTimeMillis();
    boolean reachedTargetFloor = false;
    while (!reachedTargetFloor && (System.currentTimeMillis() - startTime) < timeout) {
      reachedTargetFloor = targetFloorNum == floorNum;
    }

    return new ConditionJustification(reachedTargetFloor);
  }

  /**
   * Observer for checking the current floor.
   * NOTE: this currently assumes that the ?actor is always bound, while
   * the floor number can be a variable or floor number
   *
   * @param state
   * @return
   */
  @TRADEService
  @Observes({"fluent_equals(on_floor(?actor), ?x)"})
  List<Map<Variable, Symbol>> observeCurrentFloor(Predicate state) {
    List<Map<Variable, Symbol>> results = new ArrayList<>();

    Symbol floorQuery = state.get(1);
    if (floorQuery.isVariable()) {
      // if asking what floor, add result
      Map<Variable, Symbol> binding = new HashMap<>();
      binding.put((Variable) floorQuery, Factory.createSymbol(String.valueOf(getFloorNum())));
      results.add(binding);
    } else {
      // check if floor in state is current floor
      int queryFloorNum = Integer.valueOf(floorQuery.getName());
      if (queryFloorNum == getFloorNum()) {
        // add empty map indicating observation was made
        Map<Variable, Symbol> binding = new HashMap<>();
        results.add(binding);
      }
    }

    return results;
  }

}
