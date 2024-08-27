package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.interfaces.ArmInterface;
import edu.tufts.hrilab.interfaces.DockingInterface;
import edu.tufts.hrilab.interfaces.MoveBaseInterface;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationConsultant;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationReference;
import edu.tufts.hrilab.spot.msgs.*;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import id.jros2client.JRos2Client;
import id.jros2client.JRos2ClientFactory;
import id.jrosmessages.geometry_msgs.PointMessage;
import id.jrosmessages.geometry_msgs.QuaternionMessage;
import id.jrosmessages.geometry_msgs.Vector3Message;
import id.jrosmessages.std_msgs.StringMessage;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import pinorobotics.jros2actionlib.JRos2ActionClient;
import pinorobotics.jros2actionlib.JRos2ActionClientFactory;
import pinorobotics.jros2services.JRos2ServiceClient;
import pinorobotics.jros2services.JRos2ServiceClientFactory;
import pinorobotics.jros2tf2.JRos2Tf2;
import pinorobotics.jros2tf2.JRos2Tf2Factory;
import pinorobotics.jros2tf2.tf2_msgs.LookupTransformResultMessage;
import pinorobotics.jrostf2.exceptions.JRosTf2Exception;

import javax.vecmath.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;

public class SpotComponent extends DiarcComponent implements ArmInterface, MoveBaseInterface, DockingInterface {

  private String map_path;
  private SpotNavGraphLocationConsultant locationConsultant;
  private Symbol currentLocationId;
  private String refsConfigFile;
  private String refsConfigDir = "config/edu/tufts/hrilab/spot";

  // Define ROS Services
  private JRos2Client client;
  private JRos2ServiceClient<DiarcCommandRequestMessage, DiarcCommandResponseMessage> commandService;
  private JRos2ServiceClient<DiarcDockRequestMessage, DiarcDockResponseMessage> dockService;
  private JRos2ActionClient<NavigateToGoalMessage, NavigateToResultMessage> navigateAction;
  private JRos2ServiceClient<GraphNavGetLocalizationPoseUnstampedRequestMessage, GraphNavGetLocalizationPoseUnstampedResponseMessage> localizationService;
  private JRos2ServiceClient<DiarcPickUpRequestMessage, DiarcPickUpResponseMessage> pickUpService;
  private JRos2ServiceClient<DiarcMoveToRequestMessage, DiarcMoveToResponseMessage> ikService;

  private JRos2Tf2 tf2;
  private JRos2ServiceClient<GazeRequestMessage, GazeResponseMessage> gazeService;
  private JRos2ServiceClient<ArmCommandRequestMessage, ArmCommandResponseMessage> armService;


  public SpotComponent() {
    super();
  }

  @Override
  protected void init() {
    super.init();

    // Instantiate ROS2 services and actions
    client = new JRos2ClientFactory().createClient();
    commandService = new JRos2ServiceClientFactory().createClient(client, new DiarcCommandServiceDefinition(), "/diarc_command");
    dockService = new JRos2ServiceClientFactory().createClient(client, new DiarcDockServiceDefinition(), "/diarc_dock");
    localizationService = new JRos2ServiceClientFactory().createClient(client, new GraphNavGetLocalizationPoseUnstampedServiceDefinition(), "/graph_nav_get_localization_pose_unstamped");
    navigateAction = new JRos2ActionClientFactory().createClient(client, new NavigateToActionDefinition(), "/navigate_to");
    ikService = new JRos2ServiceClientFactory().createClient(client, new DiarcMoveToServiceDefinition(), "/diarc_move_to");
    locationConsultant = new SpotNavGraphLocationConsultant("location");
    locationConsultant.loadReferencesFromFile(refsConfigDir, refsConfigFile);
    pickUpService = new JRos2ServiceClientFactory().createClient(client, new DiarcPickUpServiceDefinition(), "/diarc_pick_up");
    gazeService = new JRos2ServiceClientFactory().createClient(client, new GazeServiceDefinition(), "/gaze");
    armService = new JRos2ServiceClientFactory().createClient(client, new ArmCommandServiceDefinition(), "/arm_command");

    tf2 = new JRos2Tf2Factory().createTf2Client(client);

    List<String> groups = this.getMyGroups();
    groups.add("location");
    try {
      TRADE.registerAllServices(locationConsultant, groups);
    } catch (TRADEException e) {
      log.error("[init] error registering location consultant with TRADE", e);
    }

//    claim();
//    powerOn();
//    stand();

    log.info("Completed init");
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("map").hasArg().argName("map").desc("Path to nav graph").build());
    options.add(Option.builder("locationRefs").hasArg().argName("locationRefs").desc("Location ref json filename").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("map")) {
      map_path = cmdLine.getOptionValue("map");
    }
    if (cmdLine.hasOption("locationRefs")) {
      refsConfigFile = cmdLine.getOptionValue("locationRefs");
    }
  }

  @Action
  @TRADEService
  public void setInitialLocation(Symbol refId) {
    SpotNavGraphLocationReference ref = locationConsultant.getRefWithId(refId);
    if (ref == null) {
      log.error("[setInitialLocation] current location id does not match a known location");
    }
    currentLocationId = refId;
  }

  @Action
  @TRADEService
  public Justification setLocationWithFiducial(Symbol refId) {
    SpotNavGraphLocationReference ref = locationConsultant.getRefWithId(refId);
    if (ref == null) {
      log.error("[setInitialLocation] current location id does not match a known location");
    }
    currentLocationId = refId;
    return goToLocationHelper(refId, refId, true);
  }

  // Mostly just for testing! You should make a new action for the command you want
  @Action
  @TRADEService
  public Justification command(String input) {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(input)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error calling command", e);
      return new ConditionJustification(false);
    }
  }

  @Action
  @TRADEService
  public Justification claim() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.CLAIM)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error claiming", e);
      return new ConditionJustification(false);
    }
  }

  @Action
  @TRADEService
  public Justification release() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.RELEASE)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error releasing", e);
      return new ConditionJustification(false);
    }
  }

  private Pair<Point2d, Quat4d> getLocalizationInformation() {
    try {
      GraphNavGetLocalizationPoseUnstampedResponseMessage res = localizationService.sendRequestAsync(new GraphNavGetLocalizationPoseUnstampedRequestMessage().withTrash(new StringMessage("empty"))).get();
      if (!res.success) {
        log.error("[getLocalizationInformation] failed to localize");
        log.warn(res.pose.toString());
      } else {
        PointMessage position = res.pose.position;
        QuaternionMessage orientation = res.pose.orientation;
        return new ImmutablePair<>(new Point2d(position.x, position.y), new Quat4d(orientation.x, orientation.y, orientation.z, orientation.w));
      }
    } catch (InterruptedException | ExecutionException e) {
      log.error("[getLocalizationInformation] Error getting localization information");
    }
    return null;
  }

  @Action
  @TRADEService
  public void bindLocationToCurrentPose(Symbol refId) {
    SpotNavGraphLocationReference ref = locationConsultant.getRefWithId(refId);
    Pair<Point2d, Quat4d> pose = getLocalizationInformation();
    ref.setPose(pose.getLeft(), pose.getRight());
  }

  @Action
  @TRADEService
  public Justification dock(int id) {
    try {
      DiarcDockResponseMessage res = dockService.sendRequestAsync(new DiarcDockRequestMessage().withDockid(new StringMessage().withData(String.valueOf(id)))).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error docking to dock id " + id, e);
      return new ConditionJustification(false);
    }
  }

  @Override
  public Justification dock(Symbol dockId) {
    try {
      DiarcDockResponseMessage res = dockService.sendRequestAsync(new DiarcDockRequestMessage().withDockid(new StringMessage().withData(dockId.getName()))).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error docking to dock id " + dockId, e);
      return new ConditionJustification(false);
    }
  }

  @Override
  public Justification undock() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.UNDOCK)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error undocking", e);
      return new ConditionJustification(false);
    }
  }

  @Action
  @TRADEService
  public Justification powerOn() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.POWER_ON)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error powering on", e);
      return new ConditionJustification(false);
    }
  }

  @Action
  @TRADEService
  public Justification powerOff() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.POWER_OFF)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error powering off", e);
      return new ConditionJustification(false);
    }
  }


  @Action
  @TRADEService
  public Justification stand() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.STAND)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error standing", e);
      return new ConditionJustification(false);
    }
  }

  @Action
  @TRADEService
  public Justification sit() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.SIT)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error sitting", e);
      return new ConditionJustification(false);
    }
  }

  @TRADEService
  @Action
  public Justification stow() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.ARM_STOW)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error stowing arm", e);
      return new ConditionJustification(false);
    }
  }

  @TRADEService
  @Action
  public Justification unstow() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.ARM_UNSTOW)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error un-stowing arm", e);
      return new ConditionJustification(false);
    }
  }

  @Action
  @TRADEService
  public Justification carry() {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.ARM_CARRY)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error setting arm to carry mode", e);
      return new ConditionJustification(false);
    }
  }

  @Action
  @TRADEService
  public Justification pickUpObject(Symbol refId) {
    MemoryObject mo = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      mo = tsi.call(MemoryObject.class, refId, MemoryObject.class);
    } catch (TRADEException e) {
      log.error("Could not get MemoryObject for reference {}", refId, e);
    }

    try {
      DiarcPickUpRequestMessage msg = new DiarcPickUpRequestMessage();
      mo.transformToBase();
      Point3d start = new Point3d(0,0,0);
      getTransform( "hand_color_image_sensor").transform(start);
      msg.sx = start.x;
      msg.sy = start.y;
      msg.sz = start.z;
      msg.ex = mo.getDirection().x * 4;
      msg.ey = mo.getDirection().y * 4;
      msg.ez = mo.getDirection().z * 4;
      msg.frame_name = new StringMessage("body");
      log.info("[pickUpObject] ray to object is from " + msg.sx + "," + msg.sy + "," + msg.sz + " to " + msg.ex + "," +msg.ey + "," + msg.ez);
      DiarcPickUpResponseMessage res = pickUpService.sendRequestAsync(msg).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error calling pickUpObject", e);
      return new ConditionJustification(false);
    }
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
    return null;
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

  @TRADEService
  @Action
  public Justification closeGripper() {
    return closeGripper("");
  }

  @Override
  public Justification closeGripper(String groupName) {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.CLOSE_GRIPPER)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(true); // todo: figure out why we're recevied an exception
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error claiming", e);
      return new ConditionJustification(false);
    }
  }

  @TRADEService
  @Action
  public Justification openGripper() {
    return openGripper("");
  }

  @Override
  public Justification openGripper(String groupName) {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.OPEN_GRIPPER)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(true);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error opening gripper", e);
      return new ConditionJustification(false);
    }
  }

  @TRADEService
  @Action
  public Justification detectionPose() {
    try {
      GazeResponseMessage res = gazeService.sendRequestAsync(new GazeRequestMessage(5, 0, 1.25f, "body", -0.75f, 0.6f, 0, -0.15f)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error moving to detection pose", e);
      return new ConditionJustification(false);
    }
  }

  @TRADEService
  @Action
  public Justification goToArmPose(String poseString, String frame) {
    List<String> poseArgs = List.of(poseString.split("\\s+"));
    float x = Float.parseFloat(poseArgs.get(0));
    float y = Float.parseFloat(poseArgs.get(1));
    float z = Float.parseFloat(poseArgs.get(2));
    float rx = Float.parseFloat(poseArgs.get(3));
    float ry = Float.parseFloat(poseArgs.get(4));
    float rz = Float.parseFloat(poseArgs.get(5));
    float rw = Float.parseFloat(poseArgs.get(6));
    try {
      ArmCommandResponseMessage res = armService.sendRequestAsync(new ArmCommandRequestMessage(frame, x, y, z, rx, ry, rz, rw)).get();
      if (!res.success) {
        log.error("[goToArmPose] failed: " + res.message);
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("[goToArmPose] error moving arm to pose: " + poseString + " in frame: " + frame);
      return new ConditionJustification(false);
    }
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
    return goToLocationHelper(currentLocationId, location, false);
  }

  public Justification goToLocationHelper(Symbol current, Symbol destination, boolean localizeWithFiducial) {
    log.info("[goToLocationHelper] going to location with refId: " + destination);
    String destinationId = locationConsultant.getRefWithId(destination).getGraphId();
    log.info("[goToLocationHelper] location has graphId: " + destinationId);
    if (destinationId == null) {
      log.error("[goToLocationHelper] refId " + destinationId + " does not correspond to a reference in the location consultant");
    }
    NavigateToGoalMessage goal = new NavigateToGoalMessage();
    goal.initial_localization_fiducial = localizeWithFiducial;
    goal.initial_localization_waypoint = new StringMessage(locationConsultant.getRefWithId(currentLocationId).getGraphId());
    goal.upload_path = new StringMessage(map_path);
    goal.navigate_to = new StringMessage(destinationId);
    NavigateToResultMessage result;
    try {
      result = navigateAction.sendGoalAsync(goal).get();
      log.debug("[goToLocationHelper] response from ros navigation goal: " + result);
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }
    if (!result.success) {
      log.error(result.message.toString());
      //todo: give useful failure justification
    }
    currentLocationId = destination;
    return new ConditionJustification(result.success);
  }

  @TRADEService
  @Action
  public Justification goToLocationUnsafe(Symbol location) {
    return goToLocation(location);
  }

  @TRADEService
  @Action
  public boolean hasBeenLocalized() {
    return currentLocationId != null;
  }

  @TRADEService
  @Action
  public Justification localize() {
    Pair<Point2d, Quat4d> pose = getLocalizationInformation();
    if (pose == null) {
      return new ConditionJustification(false);
    } else {
      Symbol locationId = locationConsultant.getClosestLocationToPoint(pose);
      if (locationId != null) {
        this.currentLocationId = locationId;
        log.info("[localize] robot is localized to " + currentLocationId);
        return new ConditionJustification(true);
      } else {
        log.warn("[localize] robot could not be localized. Has pose mapping been performed?");
        return new ConditionJustification(false);
      }
    }
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
    List<Map<Variable, Symbol>> toReturn = new ArrayList<>();
    Symbol closestLocation = locationConsultant.getClosestLocationToPoint(getLocalizationInformation());
    Symbol locationSymbol = locationTerm.get(1);
    log.debug("[checkAt] checking if at " + locationTerm);
    log.debug("[checkAt] closest location to current position is " + closestLocation);
    if (locationSymbol.equals(closestLocation)) {
      toReturn.add(new HashMap<>());
    } else {
      String locationId = locationConsultant.getRefWithId(locationSymbol).getGraphId();
      String closestId = locationConsultant.getRefWithId(closestLocation).getGraphId();
      log.error("[checkAt] failed to verify location. intended location is " + locationSymbol + " with code " + locationId + " but closest location is " + closestLocation + " with code " + closestId);
    }

    return toReturn;
  }

  @TRADEService
  public int getNumberOfKnownLocations() {
    return (int) locationConsultant.getActivatedEntities().entrySet().stream().filter(
            e -> e.getValue() > 0.0).count();
  }

  @TRADEService
  public Justification writeLocationReferencesToFile(String absolutePath) {
    boolean succeeded = locationConsultant.writeRefrencesToFile(absolutePath);
    return new ConditionJustification(succeeded);
  }

  @TRADEService
  public Matrix4d getTransform(String dstFrame) {
    return getTransform("base_link", dstFrame);
  }

  @TRADEService
  public Matrix4d getTransform(String srcFrame, String dstFrame) {
    try {
      LookupTransformResultMessage t = tf2.lookupTransform(srcFrame, dstFrame);
      Matrix4d tmp = new Matrix4d();
      tmp.setIdentity();
      QuaternionMessage r = t.transform.transform.rotation;
      tmp.setRotation(new Quat4d(r.x, r.y, r.z, r.w));
      Vector3Message s = t.transform.transform.translation;
      tmp.setTranslation(new Vector3d(s.x, s.y, s.z));
      return tmp;
    } catch (JRosTf2Exception e) {
      log.error("[getTransform] failed to get transform between " + srcFrame + " and " + dstFrame, e);
    }
    return null;
  }

}
