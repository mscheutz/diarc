/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.detector;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.detector.swig.NativeDetector.DetectorType;
import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.stm.SearchManager;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker.TrackerType;

import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import edu.tufts.hrilab.vision.util.NonEDTListModel;

import java.awt.EventQueue;
import java.io.File;
import java.util.HashMap;
import java.util.Map;
import javax.swing.DefaultComboBoxModel;

import edu.tufts.hrilab.vision.visionproc.Requirement;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.util.Util;
import edu.tufts.hrilab.util.xml.Xml;

/**
 * Object Detector Factory. This contains the detector types available to the
 * system, instantiates and dispenses Detectors to the system, and manages the
 * instantiated Detectors. Detectors are made available to the system through a
 * {@code getInstances} call, and all Detector clients must notify the factory when a
 * Detector is no longer needed by calling {@code release}.
 *
 * @author evankrause
 */
//Thread-safe!
public class AvailableDetectors {
  //available detector types
  private Map<DetectorType, DetectorDetail> typesDetail = new HashMap();
  //instantiated Detectors
  //private HashMap<DetectorType, HashSet<Detector>> instantiatedDetectors = new HashMap();
  private Map<DetectorType, DefaultComboBoxModel> instantiatedDetectors = new HashMap();
  private final Map<Detector, HashSet<Object>> instantiatedDetectorClients = new HashMap();
  //info needed to instantiate detectors of a certain Predicate
  //(e.g., face(X) = functorname(X,descriptor))
  //hashed by descriptor in outermost map, and then functorname
  private Map<String, List<InstanceInfo>> detectorOptionInfo = new HashMap();
  //only for GUI
  private NonEDTListModel<Detector> orderedTypes = new NonEDTListModel<>();
  private List<Predicate> advertisedPredicates = new ArrayList(); //Predicate advertisements -- 
  private static final Logger log = LoggerFactory.getLogger(AvailableDetectors.class);

  public AvailableDetectors(final int imgWidth, final int imgHeight, final String configFile) {
    //parse configuration file
    if (configFile != null && !configFile.isEmpty()) {

      DetectorType detectorType;
      String[] requirements;
      String[] dependencies;
      String[] trackers;
      String detectorConfigFile;

      try {
        //get directory
        File file = new File(configFile);
        String dir = file.getParent() + "/";

        //get node
        Xml config = new Xml(configFile, "detectors");

        //get node list
        List<Xml> manyDetectors = config.children("detector");
        for (Xml singleDetectorType : manyDetectors) {

          //get and check detector type
          try {
            detectorType = DetectorType.valueOf(singleDetectorType.string("type").toUpperCase());
          } catch (Exception e) {
            log.error(String.format("\"%s\" does not name a valid DetectorType. Ignoring requested Detector.", singleDetectorType.string("type")));
            continue;
          }

          //check detector type's runtime requirements
          if (singleDetectorType.containsAttribute("required")) {
            try {
              requirements = singleDetectorType.string("required").split(",\\s*");
              boolean requirementsSatisfied = true;
              for (String requirement : requirements) {
                if (!Requirement.isRequirementSatisfied(requirement)) {
                  log.info(detectorType + "'s runtime requirement (" + requirement + ") not satisfied. Skipping this detection type.");
                  requirementsSatisfied = false;
                  break;
                }
              }

              if (!requirementsSatisfied) {
                continue;
              }
            } catch (RuntimeException e) {
              log.error("Could not parse " + detectorType + "'s runtime requirements. Skipping this detection type.");
              continue;
            }
          }

          //get detector type's image processing dependencies
          dependencies = null;
          if (singleDetectorType.containsAttribute("depends")) {
            try {
              dependencies = singleDetectorType.string("depends").split(",\\s*");
            } catch (RuntimeException e) {
              log.error("Could not parse " + detectorType + "'s dependencies.");
            }
          }

          //get detector type's available trackers
          trackers = null;
          if (singleDetectorType.containsAttribute("trackers")) {
            try {
              trackers = singleDetectorType.string("trackers").split(",\\s*");
            } catch (RuntimeException e) {
              log.error("Could not parse " + detectorType + "'s trackers. Skipping detector type.");
            }
          } else {
            log.error("Could not parse " + detectorType + "'s trackers. At least one tracker option must be specified.");
            continue;
          }

          //get config file secifying supported predicates
          detectorConfigFile = singleDetectorType.string("config");

          //add to available types if doesn't exist
          if (!typesDetail.containsKey(detectorType)) {
            DetectorDetail newDetectorDetail = new DetectorDetail(detectorType, imgWidth, imgHeight);
            try {
              if (dependencies != null) {
                for (String dependency : dependencies) {
                  newDetectorDetail.addDependency(ImageProcessorType.valueOf(dependency.toUpperCase()));
                }
              }
              if (trackers != null) {
                for (String tracker : trackers) {
                  newDetectorDetail.addUsableTracker(TrackerType.valueOf(tracker.toUpperCase()));
                }
              }
            } catch (Exception e) {
              log.error(String.format("Problem parsing info for \"%s\" detector type. All such detector types will not be loaded.", detectorType), e);
              continue;
            }
            typesDetail.put(detectorType, newDetectorDetail);
          } else {
            log.warn(String.format("\"%s\" detector type has already been loaded. Ignoring duplicate request.", detectorType));
            continue;
          }

          //parse Detector's config file to get info for each predicate option
          try {
            if (detectorConfigFile != null && !detectorConfigFile.isEmpty()) {

              detectorConfigFile = dir + detectorConfigFile; // add dir to path
              if (detectorConfigFile.endsWith(".json")) {
                parseDetectorFileJson(detectorConfigFile, detectorType);
              } else if (detectorConfigFile.endsWith(".xml")) {
                parseDetectorFileXml(detectorConfigFile, detectorType);
              } else {
                log.error("Invalid detector config file format. Must be XML or JSON. File: " + detectorConfigFile);
              }
            }
          } catch (Exception e) {
            log.error(String.format("Error parsing config file: %s.", detectorConfigFile), e);
          }
        }
      } catch (Exception e) {
        log.error(String.format("Error parsing config file: %s.", configFile), e);
      }
    }
  }

  private void parseDetectorFileJson(String detectorConfigFile, DetectorType detectorType) {
    Predicate advertisement;
    String predicateType;
    int arity;
    String predicateName;

    try {
      JsonParser parser = new JsonParser();
      JsonObject root = parser.parse(new FileReader(detectorConfigFile)).getAsJsonObject();
      predicateType = root.getAsJsonObject("processor").get("type").getAsString();
      arity = root.getAsJsonObject("processor").get("arity").getAsInt();

      Iterator<JsonElement> predicateItr = root.getAsJsonArray("predicates").iterator();
      while (predicateItr.hasNext()) {
        JsonObject predicateObj = predicateItr.next().getAsJsonObject();

        predicateName = predicateObj.get("name").getAsString();
        if (arity == 1) {
          advertisement = new Predicate(predicateName, new Variable("X", PredicateHelper.varType));
        } else if (arity == 2) {
          advertisement = new Predicate(predicateName, new Variable("X", PredicateHelper.varType), new Variable("Y", PredicateHelper.varType));
        } else {
          log.error("Predicate arity not supported: {}", arity);
          continue;
        }

        //add Detector option
        addDetectorOption(advertisement, detectorType, detectorConfigFile);
      }
    } catch (Exception e) {
      log.error(String.format("Error parsing config file: %s.", detectorConfigFile), e);
    }
  }

  private void parseDetectorFileXml(String detectorConfigFile, DetectorType detectorType) {
    Predicate advertisement;
    String predicateType;
    int arity;
    String predicateName;

    try {
      Xml processor = new Xml(detectorConfigFile, "processor");
      predicateType = processor.string("type");
      arity = processor.numInt("arity");

      for (Xml predicate : processor.children("predicate")) {

        predicateName = predicate.string("name");
        if (arity == 1) {
          advertisement = new Predicate(predicateName, new Variable("X", PredicateHelper.varType));
        } else if (arity == 2) {
          advertisement = new Predicate(predicateName, new Variable("X", PredicateHelper.varType), new Variable("Y", PredicateHelper.varType));
        } else {
          log.error(String.format("Can't handle Predicates with arity: %d.", arity));
          continue;
        }

        //add Detector option
        addDetectorOption(advertisement, detectorType, detectorConfigFile);
      }
    } catch (Exception e) {
      log.error(String.format("Error parsing config file: %s.", detectorConfigFile), e);
    }
  }

  private void addDetectorOption(Predicate descriptor, DetectorType detectorType, String detectorConfigFile) {
    String predicateName = descriptor.getName();

    List<InstanceInfo> detectorOptions = detectorOptionInfo.get(predicateName);
    if (detectorOptions == null) {
      detectorOptions = new ArrayList<>();
      detectorOptionInfo.put(predicateName, detectorOptions);
    }

    advertisedPredicates.add(descriptor);
    detectorOptions.add(new InstanceInfo(detectorType, detectorConfigFile));
  }

  /**
   * Should only be used by the GUI!
   *
   * @return container of all instantiated Trackers
   */
  public synchronized NonEDTListModel<Detector> getAll() {
    return orderedTypes;
  }

  /**
   * Get set of available predicates advertised to the rest of the system.
   * Should only be used by CameraControl GUI.
   *
   * @return
   */
  public synchronized List<Predicate> getOptions() {
    return new ArrayList(advertisedPredicates);
  }

  /**
   * Get instance info for Predicate. If there are multiple options, return all
   * options.
   *
   * @param term
   * @return
   */
  private List<InstanceInfo> getAllInstanceInfo(final Term term) {
    //based on descriptor name
    List<InstanceInfo> detectorOptions = detectorOptionInfo.get(PredicateHelper.getRepresentativeString(term));
    if (detectorOptions == null) {
      log.debug(String.format("[getInstanceInfo] No detectors match the description: %s.", term));
      return null;
    }

    return detectorOptions;
  }

  /**
   * Get instance info for Predicate. If there are multiple options, the first
   * one is returned.
   *
   * @param term
   * @return
   */
  private InstanceInfo getInstanceInfo(final Term term) {
    List<InstanceInfo> detectorOptions = getAllInstanceInfo(term);

    if (detectorOptions == null || detectorOptions.isEmpty()) {
      log.debug(String.format("[getInstanceInfo] No detectors match the description: %s.", term.toString()));
      return null;
    }

    if (detectorOptions.size() > 1) {
      //TODO: how to deal with this case?
      log.info(String.format("[getInstanceInfo] More than one detector matches the "
              + "description: %s. Returning first found matching instance.", term.toString()));
    }

    return detectorOptions.get(0);
  }

  /**
   * Find out if there is a Detector that is capable of detecting objects that
   * meet the specified description.
   *
   * @param term - Predicate description
   * @return - boolean
   */
  public boolean hasCapableDetector(final Term term) {
    if (getInstanceInfo(term) == null) {
      return false;
    }

    return true;
  }

  /**
   * Get all available Detectors that match predicate.
   *
   * @param client - SearchManager making request
   * @param term
   * @return list of available detectors matching term
   */
  public synchronized List<Detector> getInstances(final SearchManager client, final Term term) {
    //results to fill
    List<Detector> resultDetectors = new ArrayList();

    List<InstanceInfo> detectorOptions = getAllInstanceInfo(term);

    if (detectorOptions == null) {
      return resultDetectors;
    }

    //get instances of all detector options.
    DetectorType dType;
    String config;
    for (InstanceInfo detectorOption : detectorOptions) {
      dType = detectorOption.type;
      config = detectorOption.config;

      DetectorDetail detectorDetail = typesDetail.get(dType);
      if (detectorDetail == null) {
        log.error(String.format("[getInstances] No DetectorDetail for DetectorType: %s.", dType.toString()));
      }

      boolean singleInstance = false;

      //only single instances of these allowed
      //TODO: add this info to config file
      if (dType == DetectorType.CLUSTER_ADVANCED
              || dType == DetectorType.CLUSTER) {
        singleInstance = true;
      }
      log.info("");

      //get list of instantiated detectors of requested type
      DefaultComboBoxModel detectorList = instantiatedDetectors.get(dType);
      if (detectorList == null) {
        detectorList = new DefaultComboBoxModel();
        instantiatedDetectors.put(dType, detectorList);
      }

      //find existing, or instantiate new, detector
      Detector detectorInstance;
      if (detectorList.getSize() == 0 || !singleInstance) { //instantiate new Detector

        detectorInstance = new Detector(detectorDetail);
        if (config != null && !config.isEmpty()) {
          detectorInstance.loadConfig(config);
        }

        orderedTypes.add(detectorInstance);

        //DefaultComboBoxModel needs to be modified on the GUI EventQueue
        if (EventQueue.isDispatchThread()) {
          detectorList.addElement(detectorInstance);
        } else {
          final DefaultComboBoxModel detectorList_final = detectorList;
          final Detector newDetectorInstance_final = detectorInstance;
          // FIXME
//          try {
//            EventQueue.invokeAndWait(() -> detectorList_final.addElement(newDetectorInstance_final));  // causes deadlock
//          } catch (InterruptedException | InvocationTargetException e) {
//            log.error("[getInstances] problem adding new detector instance to detector list.");
//          }
          EventQueue.invokeLater(() -> detectorList_final.addElement(newDetectorInstance_final));
          Util.Sleep(2000);
        }
      } else { //use existing detector
        detectorInstance = (Detector) detectorList.getElementAt(0);
      }

      addClient(client, detectorInstance);
      resultDetectors.add(detectorInstance);
    }

    return resultDetectors;
  }

  //helper function to add getInstance caller to client list
  private void addClient(final Object client, final Detector detector) {
    HashSet<Object> clientList = instantiatedDetectorClients.get(detector);
    if (clientList == null) {
      clientList = new HashSet();
      instantiatedDetectorClients.put(detector, clientList);
    }

    if (clientList.contains(client)) {
      log.warn(String.format("[addClient] Calling object (%s) is already a client of the Detector: %s.",
              client.toString(), detector.toString()));
      return;
    } else {
      //add client to ImageProcessor's client list
      clientList.add(client);
    }
  }

  /**
   * Client needs to let managing factory know when it's done using a Detector
   * instance, so that Detectors can be properly managed and garbage collected.
   */
  public synchronized void release(final Object client, final Detector detector) {
    HashSet<Object> clientList = instantiatedDetectorClients.get(detector);
    if (clientList == null || !clientList.contains(client)) {
      log.warn(String.format("[release] Calling object (%s) is not a client of the Detector: %s.",
              client.toString(), detector.toString()));
      return;
    } else {
      clientList.remove(client);

      //if Detector doesn't have any clients, clean up references
      if (clientList.isEmpty()) {
        //remove it from all containers so it can be garbage collected
        instantiatedDetectorClients.remove(detector);
        instantiatedDetectors.get(detector.getType()).removeElement(detector);
        orderedTypes.remove(detector);

        //"delete" detector
        detector.terminate();

        //TODO: if the processor is the last one of its kind (i.e., DetectorTpe),
        //instead of removing from containers, perhaps the detector
        //should be "cleaned up"and kept around for future use, instead
        //of having to instantiate a new one ??
      }
    }
  }

  /**
   * Helper class to hold the information needed to instantiate a particular
   * detectable object type option.
   */
  private class InstanceInfo {

    public final DetectorType type;
    public final String config;

    /**
     * Constructor.
     *
     * @param detectorType Detector Enum defined in native ObjectDetector.
     * @param configFile   configuration file containing all predicates this
     *                     DetectorType supports and the necessary run-time info for the native
     *                     side.
     */
    InstanceInfo(final DetectorType detectorType, final String configFile) {
      type = detectorType;
      config = configFile;
    }
  }
}
