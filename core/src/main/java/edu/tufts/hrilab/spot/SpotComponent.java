package edu.tufts.hrilab.spot;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.interfaces.ArmInterface;
import edu.tufts.hrilab.interfaces.MoveBaseInterface;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationConsultant;
import edu.tufts.hrilab.spot.consultant.SpotNavGraphLocationReference;
import edu.tufts.hrilab.spot.msgs.*;
import id.jros2client.JRos2Client;
import id.jros2client.JRos2ClientFactory;
import id.jrosmessages.std_msgs.StringMessage;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.Pair;
import pinorobotics.jros2actionlib.JRos2ActionClient;
import pinorobotics.jros2actionlib.JRos2ActionClientFactory;
import pinorobotics.jros2services.JRos2ServiceClient;
import pinorobotics.jros2services.JRos2ServiceClientFactory;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;

public class SpotComponent extends DiarcComponent implements ArmInterface, MoveBaseInterface {

  private String map_path;
  private JRos2Client client;
  private JRos2ServiceClient<DiarcCommandRequestMessage, DiarcCommandResponseMessage> commandService;
  private JRos2ServiceClient<DiarcDockRequestMessage, DiarcDockResponseMessage> dockService;
  private JRos2ActionClient<NavigateToGoalMessage, NavigateToResultMessage> navigateAction;
  private SpotNavGraphLocationConsultant locationConsultant;
  private Symbol currentLocationId;
  private String refsConfigFile;
  private String refsConfigDir = "config/edu/tufts/hrilab/spot";
  private JRos2ServiceClient<GraphNavGetLocalizationPoseUnstampedRequestMessage, GraphNavGetLocalizationPoseUnstampedResponseMessage> localizationService;

  public SpotComponent() {
    super();
  }

  @Override
  protected void init() {
    super.init();
    client = new JRos2ClientFactory().createClient();
    commandService = new JRos2ServiceClientFactory().createClient(client, new DiarcCommandServiceDefinition(), "/diarc_command");
    dockService = new JRos2ServiceClientFactory().createClient(client, new DiarcDockServiceDefinition(), "/diarc_dock");
    localizationService = new JRos2ServiceClientFactory().createClient(client, new GraphNavGetLocalizationPoseUnstampedServiceDefinition(), "/graph_nav_get_localization_pose_unstamped");
    navigateAction = new JRos2ActionClientFactory().createClient(client, new NavigateToActionDefinition(), "/navigate_to");
    locationConsultant = new SpotNavGraphLocationConsultant("location");
    List<String> groups = this.getMyGroups();
    groups.add("location");
    try {
      TRADE.registerAllServices(locationConsultant, groups);
    } catch (TRADEException e) {
      log.error("[init] error registering location consultant with TRADE", e);
    }
    locationConsultant.loadReferencesFromFile(refsConfigDir, refsConfigFile);

    claim();
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

  private Point2d getLocalizationInformation() {
    try {
      GraphNavGetLocalizationPoseUnstampedResponseMessage res = localizationService.sendRequestAsync(new GraphNavGetLocalizationPoseUnstampedRequestMessage().withTrash(new StringMessage("empty"))).get();
      if (!res.success) {
        log.warn(res.pose.toString());
      } else {
        return new Point2d(res.pose.position.x, res.pose.position.y);
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
    Point2d location = getLocalizationInformation();
    ref.setLocation(location);
  }

  @Action
  @TRADEService
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

  @Override
  public Justification closeGripper(String groupName) {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.STAND)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error claiming", e);
      return new ConditionJustification(false);
    }
  }

  @Override
  public Justification openGripper(String groupName) {
    try {
      DiarcCommandResponseMessage res = commandService.sendRequestAsync(new DiarcCommandRequestMessage(DiarcCommandRequestMessage.Command.STAND)).get();
      if (!res.success) {
        log.warn(res.message.toString());
      }
      return new ConditionJustification(res.success);
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error claiming", e);
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
    log.debug("[goToLocation] going to location with refId: " + location);
    String destination = locationConsultant.getRefWithId(location).getGraphId();
    if (destination == null) {
      log.error("[goToLocation] refId " + location + " does not correspond to a reference in the location consultant");
    }
    NavigateToGoalMessage goal = new NavigateToGoalMessage();
    goal.initial_localization_fiducial = false;
    goal.initial_localization_waypoint = new StringMessage(locationConsultant.getRefWithId(currentLocationId).getGraphId());
    goal.upload_path = new StringMessage(map_path);
    goal.navigate_to = new StringMessage(destination);
    NavigateToResultMessage result;
    try {
      result = navigateAction.sendGoalAsync(goal).get();
    } catch (InterruptedException | ExecutionException e) {
      throw new RuntimeException(e);
    }

    if (!result.success) {
      log.error(result.message.toString());
    }
    currentLocationId = location;
    return new ConditionJustification(result.success);
  }

  @TRADEService
  @Action
  public Justification goToLocationUnsafe(Symbol location) {
    return goToLocation(location);
  }

  @TRADEService
  @Action
  public void localize() {
    this.currentLocationId = locationConsultant.getClosestLocationToPoint(getLocalizationInformation());
    log.info("[localize] robot is localized to " + currentLocationId);
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
    }
    return toReturn;
  }

  @TRADEService
  public int getNumberOfKnownLocations() {
    return (int) locationConsultant.getActivatedEntities().entrySet().stream().filter(
            e -> e.getValue() > 0.0).count();
  }

}
