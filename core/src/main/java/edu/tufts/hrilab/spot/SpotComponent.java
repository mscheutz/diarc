package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.interfaces.ArmInterface;
import edu.tufts.hrilab.interfaces.MoveBaseInterface;
import edu.tufts.hrilab.ros2.ros2jrosclient.generated.AddTwoIntsRequestMessage;
import edu.tufts.hrilab.ros2.ros2jrosclient.generated.AddTwoIntsResponseMessage;
import edu.tufts.hrilab.ros2.ros2jrosclient.generated.AddTwoIntsServiceDefinition;
import edu.tufts.hrilab.spot.msgs.*;
import id.jros2client.JRos2Client;
import id.jros2client.JRos2ClientConfiguration;
import id.jros2client.JRos2ClientFactory;
import id.jros2messages.geometry_msgs.TwistWithCovarianceStampedMessage;
import id.jrosclient.TopicSubscriber;
import id.jrosmessages.geometry_msgs.TwistMessage;
import id.jrosmessages.geometry_msgs.Vector3Message;
import id.jrosmessages.std_msgs.StringMessage;
import id.jrosmessages.std_msgs.UInt16Message;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.Pair;
import pinorobotics.jros2actionlib.JRos2ActionClient;
import pinorobotics.jros2actionlib.JRos2ActionClientFactory;
import pinorobotics.jros2services.JRos2ServiceClient;
import pinorobotics.jros2services.JRos2ServiceClientFactory;
import pinorobotics.rtpstalk.RtpsTalkConfiguration;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class SpotComponent extends DiarcComponent implements ArmInterface, MoveBaseInterface {

  private String map_path;
  private JRos2Client client;
  //  private JRos2ActionClient<NavigateToGoalMessage, NavigateToResultMessage> navigateAction;
  private JRos2ServiceClient<TriggerRequestMessage, TriggerResponseMessage> standService;
  private JRos2ServiceClient<TriggerRequestMessage, TriggerResponseMessage> sitService;
  private JRos2ServiceClient<TriggerRequestMessage, TriggerResponseMessage> openGripperService;
  private JRos2ServiceClient<TriggerRequestMessage, TriggerResponseMessage> closeGripperService;
  private JRos2ServiceClient<TriggerRequestMessage, TriggerResponseMessage> claimService;
  private JRos2ServiceClient<TriggerRequestMessage, TriggerResponseMessage> powerOnService;
  private JRos2ServiceClient<DockRequestMessage, DockResponseMessage> dockService;
  private JRos2ServiceClient<SetVelocityRequestMessage, SetVelocityResponseMessage> setVelocityService;
  JRos2ServiceClient<AddTwoIntsRequestMessage, AddTwoIntsResponseMessage> addtwo;

  public SpotComponent() {
    super();
  }


  @Override
  protected void init() {
    super.init();
//    JRos2ClientConfiguration.Builder configBuilder = new JRos2ClientConfiguration.Builder();
//    RtpsTalkConfiguration.Builder rtpsConfigBuilder = new RtpsTalkConfiguration.Builder();
//    configBuilder.rtpsTalkConfiguration(rtpsConfigBuilder.networkInterface("lo").build());
    client = new JRos2ClientFactory().createClient();
//    navigateAction = new JRos2ActionClientFactory().createClient(client, new NavigateToActionDefinition(), "navigate_to");
    standService = new JRos2ServiceClientFactory().createClient(client, new TriggerServiceDefinition(), "/stand");
    dockService = new JRos2ServiceClientFactory().createClient(client, new DockServiceDefinition(), "/dock");
    sitService = new JRos2ServiceClientFactory().createClient(client, new TriggerServiceDefinition(), "/sit");
    claimService = new JRos2ServiceClientFactory().createClient(client, new TriggerServiceDefinition(), "/claim");
    powerOnService = new JRos2ServiceClientFactory().createClient(client, new TriggerServiceDefinition(), "/power_on");
    openGripperService = new JRos2ServiceClientFactory().createClient(client, new TriggerServiceDefinition(), "/open_gripper");
    closeGripperService = new JRos2ServiceClientFactory().createClient(client, new TriggerServiceDefinition(), "/close_gripper");
    addtwo = new JRos2ServiceClientFactory().createClient(client, new AddTwoIntsServiceDefinition(), "/add_two_ints");
    setVelocityService = new JRos2ServiceClientFactory().createClient(client, new SetVelocityServiceDefinition(), "/max_velocity");


    try {
      SetVelocityResponseMessage res = setVelocityService.sendRequestAsync(new SetVelocityRequestMessage().withVelocityLimit(new TwistMessage().withAngular(new Vector3Message(1, 1, 2)).withLinear(new Vector3Message(3, 1, 3)))).get();
      log.info(res.toString());
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }
//    claimService.sendRequestAsync(new TriggerRequestMessage());
//    powerOnService.sendRequestAsync(new TriggerRequestMessage());
//    stand();

    log.info("Completed init");
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("map").hasArg().argName("path").desc("Path to nav graph").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("map")) {
      map_path = cmdLine.getOptionValue("path");
    }
  }

  @Action
  @TRADEService
  public Justification claim() {
    claimService.sendRequestAsync(new TriggerRequestMessage());
    return new ConditionJustification(true);
  }

  @Action
  @TRADEService
  public Justification testRos(int a, int b) {
    try {
      AddTwoIntsResponseMessage res = addtwo.sendRequestAsync(new AddTwoIntsRequestMessage().withA(a).withB(b)).get();
      log.info(String.valueOf(res.sum));
      return new ConditionJustification(true);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error with test service", e);
    }
    return new ConditionJustification(false);
  }

  @Action
  @TRADEService
  public Justification dock(int id) {
    DockResponseMessage res;
    DockRequestMessage req = new DockRequestMessage().withDockId(new Int16Message().withData((short) id));
    try {
      res = dockService.sendRequestAsync(req).get();
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }
    return new ConditionJustification(res.success);
//    return new ConditionJustification(true);
  }


  @Action
  @TRADEService
  public Justification stand() {
    TriggerResponseMessage res;
    try {
      res = standService.sendRequestAsync(new TriggerRequestMessage()).get();
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }
    return new ConditionJustification(res.success);
  }

  @Action
  @TRADEService
  public Justification sit() {
    TriggerResponseMessage res;
    try {
      res = sitService.sendRequestAsync(new TriggerRequestMessage()).get();
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }
    return new ConditionJustification(res.success);
  }

  @Override
  public boolean moveTo(String groupName, Point3d point, Quat4d orientation) {
    return false;
  }

  @Override
  public boolean moveTo(String groupName, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r) {
    return false;
  }

  @Override
  public Justification moveTo(String groupName, Symbol refId) {
//    NavigateToGoalMessage goal = new NavigateToGoalMessage();
//    goal.initial_localization_fiducial = false;
//    goal.upload_path = new StringMessage("/home/mfawn/code/graph");
//    goal.navigate_to = new StringMessage("waypoint_45");
//    NavigateToResultMessage result;
//    try {
//      result = navigateAction.sendGoalAsync(goal).get();
//    } catch (InterruptedException | ExecutionException e) {
//      throw new RuntimeException(e);
//    }
//
//    if (!result.success) {
//      log.error(result.message);
//    }
//    return new ConditionJustification(result.success);
    return new ConditionJustification(true);
  }

  @Override
  public Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints) {
    return null;
  }

  @Override
  public Justification moveToRelative(String groupName, Point3d point, Quat4d orientation) {
    return null;
  }

  @Override
  public Justification graspObject(String groupName, Symbol refId, float position) {
    return null;
  }

  @Override
  public Justification releaseObject(String groupName, Symbol refId, float position) {
    return null;
  }

  @Override
  public boolean pointTo(String groupName, Symbol objectRef) {
    return false;
  }

  @Override
  public Pair<Point3d, Quat4d> getEEPose(String groupName) {
    return null;
  }

  @Override
  public Pair<Point3d, Quat4d> getPose(String linkName) {
    return null;
  }

  @Override
  public boolean recordPose(Symbol poseName) {
    return false;
  }

  @Override
  public boolean saveEEPosesToFile(String filename) {
    return false;
  }

  @Override
  public void loadEEPosesFromFile(String filename) {

  }

  @Override
  public boolean savePosesToFile(String filename) {
    return false;
  }

  @Override
  public void loadPosesFromFile(String filename) {

  }

  @Override
  public boolean goToPose(String groupName, Symbol poseName) {
    return false;
  }

  @Override
  public boolean goToPose(Symbol poseName) {
    return false;
  }

  @Override
  public boolean goToStartPose(boolean safe) {
    return false;
  }

  @Override
  public void startRecordingTrajectory(String trajectoryName) {

  }

  @Override
  public void stopRecordingTrajectory() {

  }

  @Override
  public boolean executeTrajectory(String trajectoryName) {
    return false;
  }

  @Override
  public boolean saveTrajectoriesToFile(String filename) {
    return false;
  }

  @Override
  public void loadTrajectoriesFromFile(String filename) {

  }

  @Override
  public Justification closeGripper(String groupName) {
    TriggerResponseMessage res;
    try {
      res = closeGripperService.sendRequestAsync(new TriggerRequestMessage()).get();
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }
    return new ConditionJustification(res.success);
  }

  @Override
  public Justification openGripper(String groupName) {
    TriggerResponseMessage res;
    try {
      res = openGripperService.sendRequestAsync(new TriggerRequestMessage()).get();
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }
    return new ConditionJustification(res.success);
  }

  @Override
  public boolean moveGripper(String groupName, float meters) {
    return false;
  }

  @Override
  public float getGripperPosition(String groupName) {
    return 0;
  }

  @Override
  public Justification pressObject(String group_name, Point3d object_location, Quat4d object_orientation) {
    return null;
  }

  @Override
  public Justification pressObject(String group_name, Symbol refID) {
    return null;
  }

  @Override
  public double[] getPoseGlobalQuat() {
    return new double[0];
  }

  @Override
  public void setPoseGlobal(double x, double y, double theta) {

  }

  @Override
  public Justification goToLocation(Symbol location, boolean wait) {
    return null;
  }

  @Override
  public Justification goToLocation(Symbol location) {
    return null;
  }

  @Override
  public Justification goToLocation(Symbol desiredLocation, Symbol initialLocation) {
    return null;
  }

  @Override
  public Justification goToLocation(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait) {
    return null;
  }

  @Override
  public Justification approachLocation(Symbol location) {
    return null;
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation, Symbol initialLocation) {
    return null;
  }

  @Override
  public Justification stop() {
    return null;
  }

  @Override
  public Justification isMoving() {
    return null;
  }

  @Override
  public List<Map<Variable, Symbol>> checkAt(Term locationTerm) {
    return List.of();
  }
}
