/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.movebase;

import ai.thinkingrobots.trade.TRADEService;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarcros.msg.nav_msgs.LoadMapRequest;
import edu.tufts.hrilab.diarcros.msg.nav_msgs.LoadMapResponse;
import edu.tufts.hrilab.diarcros.map.MapServer;
import edu.tufts.hrilab.util.RosPackPathHelper;
import org.apache.commons.lang3.tuple.Pair;

import java.io.File;

/**
 * This component allows for map swapping functionality
 */
public class ChangeMapComponent extends DiarcComponent {
  private MapServer mapServer;
  private RosPackPathHelper rosPackageHelper;


  /**
   * ChangeMapComponent Constructor. Simply starts the MapServer ade ROS ndoe
   */
  public ChangeMapComponent() {
    super();
    mapServer = new MapServer();
    mapServer.waitForNode();
    rosPackageHelper = new RosPackPathHelper();
  }

    /**
     * Changes the robot map to the given map.
     *
     * @param mapAbsPath The absolute path of the map.yaml file
     * @return If the action was successful or not
     */
    @TRADEService
    public boolean changeMap(String mapAbsPath) {
      Pair<String, String> packageInfo = rosPackageHelper.getPackageAndRelativePath(mapAbsPath);
      return changeMap(packageInfo.getLeft(), packageInfo.getRight());
    }

  /**
   * Changes the robot map to the given map based off a package and relative file path. For an example:
   * <pre> changeMap("map_package", "map.yaml") </pre>
   * Calls this example absolute path:
   * <pre> /home/user/catkin_ws/src/map_package/map.yaml </pre>
   *
   * @param mapPackName    The package that contains the map.yaml file
   * @param mapRelPackPath The map.yaml file path relative to the given package.
   * @return
   */
  @TRADEService
  public boolean changeMap(String mapPackName, String mapRelPackPath) {
    LoadMapRequest request = new LoadMapRequest();
    LoadMapResponse response = new LoadMapResponse();

    request.setMapUrl("package://" + mapPackName + "/" + mapRelPackPath);

    mapServer.callChangeMap(request, response);
    log.debug("[changeMap] Request response result: "+ response.getResult());
    return response.getResult() == LoadMapResponse.RESULT_SUCCESS;
  }
}
    
