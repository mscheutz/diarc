/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.tracker;

import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.stm.SearchManager;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker.TrackerType;
import edu.tufts.hrilab.vision.util.NonEDTListModel;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import edu.tufts.hrilab.vision.visionproc.Requirement;
import edu.tufts.hrilab.util.xml.Xml;

import java.util.HashMap;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Object Tracker Factory. This contains the tracker types available to the
 * system, instantiates and dispenses Trackers to the system, and manages the
 * instantiated Trackers. Trackers are made available to the system through a
 * getInstance call, and all Tracker clients must notify the factory when a
 * Tracker is no longer needed by calling release. Thread-safe!
 *
 * @author Evan Krause
 */
public class AvailableTrackers {
  //available tracker types

  private Map<TrackerType, TrackerDetail> typesDetail = new HashMap();
  private List<TrackerDetail> orderedTypesDetail = new ArrayList(); //for gui display purposes
  //instantiated Trackers
  private Map<TrackerType, HashSet<Tracker>> instantiatedTrackers = new HashMap();
  private Map<Tracker, HashSet<Object>> instantiatedTrackerClients = new HashMap();
  //for GUI
  private NonEDTListModel<Tracker> orderedTypes = new NonEDTListModel<>();
  private static Logger log = LoggerFactory.getLogger(AvailableTrackers.class);

  public AvailableTrackers(final int imgWidth, final int imgHeight, final String configFile) {

    //parse configuration file
    if (configFile != null && !configFile.isEmpty()) {
      TrackerType type;
      String filename;
      String[] requirements;
      String[] dependencies;

      try {
        //get directory    
        File file = new File(configFile);
        String dir = file.getParent() + "/";
        log.debug("load tracker config directory: " + dir);

        //get node
        Xml config = new Xml(configFile, "trackers");

        //get node list
        List<Xml> manyTrackers = config.children("tracker");
        for (Xml tracker : manyTrackers) {

          //get and check type
          try {
            type = TrackerType.valueOf(tracker.string("type").toUpperCase());
          } catch (Exception e) {
            log.error(String.format("\"%s\" does not name a valid TrackerType. Ignoring requested Tracker.", tracker.string("type")), e);
            continue;
          }

          //check processor type's runtime requirements
          if (tracker.containsAttribute("required")) {
            try {
              requirements = tracker.string("required").split(",\\s*");
              boolean requirementsSatisfied = true;
              for (String requirement : requirements) {
                if (!Requirement.isRequirementSatisfied(requirement)) {
                  log.info(type + "'s runtime requirement (" + requirement + ") not satisfied. Skipping this tracker type.");
                  requirementsSatisfied = false;
                  break;
                }
              }

              if (!requirementsSatisfied) {
                continue;
              }
            } catch (RuntimeException e) {
              log.error("Could not parse " + type + "'s runtime requirements. Skipping this tracker type.");
              continue;
            }
          }

          //get detector type's image processing dependencies
          dependencies = null;
          if (tracker.containsAttribute("depends")) {
            try {
              dependencies = tracker.string("depends").split(",\\s*");
            } catch (RuntimeException e) {
              log.error("Could not parse " + type + "'s dependencies.");
            }
          }

          //get and check filename
          filename = null;
          if (tracker.containsAttribute("config")) {
            try {
              filename = tracker.string("config");
            } catch (RuntimeException e) {
              log.error("Could not parse " + type + "'s config filename.");
            }
          }

          // this is just a check to make sure the file exists
          if (filename != null && !filename.isEmpty()) {
            filename = dir + filename;
            File tmpDir = new File(filename);
            if (!tmpDir.exists()) {
              log.error(String.format("\"%s\" does not exist. Ignoring requested \"%s\" Tracker.", filename, type.toString()));
              continue;
            }
          }

          //add to availableTypes
          if (!typesDetail.containsKey(type)) {
            TrackerDetail newTrackerDetail = new TrackerDetail(type, imgWidth, imgHeight);
            try {
              if (dependencies != null) {
                for (String dependency : dependencies) {
                  newTrackerDetail.addDependency(ImageProcessorType.valueOf(dependency.toUpperCase()));
                }
              }
              if (filename != null && !filename.isEmpty()) {
                newTrackerDetail.setConfig(filename);
              }
            } catch (Exception e) {
              log.error(String.format("Problem parsing info for \"%s\" detector type. All such detector types will not be loaded.", type.toString()), e);
              continue;
            }
            typesDetail.put(type, newTrackerDetail);
            orderedTypesDetail.add(newTrackerDetail);
          } else {
            log.warn(String.format("\"%s\" tracker has already been loaded. Ignoring duplicate request.", type.toString()));
          }
        }
      } catch (Exception e) {
        log.error(String.format("Error parsing config file: %s.", e.toString()), e);
      }
    }

  }

  /**
   * Should only be used by the GUI!
   *
   * @return container of all instantiated Trackers
   */
  public synchronized NonEDTListModel<Tracker> getAll() {
    return orderedTypes;
  }

  /**
   * Get Tracker instance of specified type.
   *
   * @param client - Java object requesting Tracker instace
   * @param tType  - type if Tracker
   * @return instantiated Tracker
   */
  public synchronized Tracker getInstance(final SearchManager client, final TrackerType tType) {
    TrackerDetail trackerDetail = typesDetail.get(tType);
    if (trackerDetail == null) {
      return null;
    }

    boolean singleInstance = false;

    //only single instances of these allowed
    TrackerType trackerType = trackerDetail.getType();
    if (trackerType == TrackerType.CMT || trackerType == TrackerType.KCF || trackerType == TrackerType.TLD) {
      singleInstance = true;
    }

    //get list of trackers of specified type
    HashSet<Tracker> trackerSet = instantiatedTrackers.get(trackerType);
    if (trackerSet == null) {
      trackerSet = new HashSet<>();
      instantiatedTrackers.put(trackerType, trackerSet);
    }

    //find existing, or instantiate new, image processor
    Tracker trackerInstance;
    if (trackerSet.isEmpty() || !singleInstance) {   //create new instance
      trackerInstance = new Tracker(trackerDetail);
      trackerSet.add(trackerInstance);
      orderedTypes.add(trackerInstance);

      //load config, if specified
      String config = trackerDetail.getConfig();
      //System.out.println("ip getInstance type: " + type + " config: " + config);
      if (config != null) {
        trackerInstance.loadConfig(config);
      }
    } else {    //use existing instance
      trackerInstance = trackerSet.iterator().next();
    }

    addClient(client, trackerInstance);
    return trackerInstance;
  }

  public synchronized boolean hasAvailableTracker(TrackerType trackerType) {
    return typesDetail.containsKey(trackerType);
  }

  //helper function to add getInstance caller to client list
  private void addClient(final Object client, final Tracker tracker) {
    HashSet<Object> clientList = instantiatedTrackerClients.get(tracker);
    if (clientList == null) {
      clientList = new HashSet();
      instantiatedTrackerClients.put(tracker, clientList);
    }

    if (clientList.contains(client)) {
      log.error("[addClient] Calling object is already a client of the Tracker.");
      return;
    } else {
      //add client to ImageProcessor's client list
      clientList.add(client);
    }
  }

  /**
   * Client needs to let managing factory know when it's done using a Tracker
   * instance, so that Detectors can be properly managed and garbage collected.
   */
  public synchronized void release(final Object client, final Tracker tracker) {
    HashSet<Object> clientList = instantiatedTrackerClients.get(tracker);
    if (clientList == null || !clientList.contains(client)) {
      log.error("[release] Calling object is not a client of the Tracker.");
      return;
    } else {
      clientList.remove(client);

      //if Tracker doesn't have any clients, clean up references
      if (clientList.isEmpty()) {
        //remove it from all containers so it can be garbage collected
        instantiatedTrackerClients.remove(tracker);
        instantiatedTrackers.get(tracker.getType()).remove(tracker);
        orderedTypes.remove(tracker);

        //"delete" tracker
        tracker.terminate();

        //TODO: if the processor is the last one of its kind (i.e., DetectorTpe),
        //instead of removing from containers, perhaps the detector
        //should be "cleaned up"and kept around for future use, instead
        //of having to instantiate a new one ??
      }
    }
  }

  /**
   * Helper class to hold the information needed to instantiate a particular
   * tracker type option.
   */
  private class InstanceInfo {

    public final TrackerType type;
    public String config;

    InstanceInfo(final TrackerType trackerType, final String configFile) {
      type = trackerType;
      config = configFile;
    }
  }
}
