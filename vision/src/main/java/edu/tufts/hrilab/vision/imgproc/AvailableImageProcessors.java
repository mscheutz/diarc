/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.imgproc;

import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.common.swig.CommonModule;
import java.util.ArrayList;
import java.util.HashSet;
import javax.swing.AbstractListModel;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.vision.util.NonEDTListModel;
import edu.tufts.hrilab.vision.util.PredicateHelper;
import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import edu.tufts.hrilab.vision.visionproc.Requirement;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.util.xml.Xml;

/**
 * ImageProcessor Factory. This contains the ImageProcessor types available to
 * the system, instantiates and dispenses ImageProcessors to the system, and
 * manages the instantiated ImageProcessors. ImageProcessors are made available
 * to the system through a {@code getInstance} call, and all ImageProcessor clients
 * must notify the factory when a ImageProcessor is no longer needed by calling
 * {@code release}. Basically, every {@code getInstance} call should be paired with a
 * {@code release} call.
 *
 * @author evankrause
 */
//Thread-safe!
public class AvailableImageProcessors extends AbstractListModel {

  /**
   * Available ImageProcessorTypes ("compile time" info) hashed by type.
   */
  private final HashMap<ImageProcessorType, ImageProcessorDetail> typesDetail = new HashMap();
  /**
   * Available ImageProcessorTypes ("compile time" info) for GUI display purposes.
   */
  private final ArrayList<ImageProcessorDetail> orderedTypesDetail = new ArrayList();
  /**
   * Instantiated ImageProcessors hashed by type.
   */
  private final HashMap<ImageProcessorType, HashSet<ImageProcessor>> instantiatedProcessors = new HashMap();
  /**
   * Instantiated ImageProcessor clients hashed by ImageProcessor.
   */
  private final HashMap<ImageProcessor, HashSet<Object>> instantiatedProcessorClients = new HashMap();
  /**
   * Info needed to instantiate ImageProcessors of a certain "attribute" (runtime info).
   * (e.g., red(x) = name(X), where red is of type color (i.e., red isA color))
   * hashed by name in outermost map, and then predicate type -- if no type is
   * specified, name is used in both hash maps
   */
  private HashMap<String, HashMap<String, List<InstanceInfo>>> processorOptionInfo = new HashMap();
  /**
   * Info needed to instantiate ImageProcessors based on a ImageProcessorType. (runtime info)
   * NOTE: Only need to keep track of a single configFile option for each IPType in this
   * container bc the instance configuration isn't important when requested via IPTYpe. If
   * more detail/control is needed, the instance should be instantiated using its "attribute".
   */
  private HashMap<ImageProcessorType, String> configFiles = new HashMap();
  /**
   * Instantiated ImageProcessors for GUI.
   */
  private NonEDTListModel<ImageProcessor> orderedTypes = new NonEDTListModel<>();
  /**
   * Available advertised Predicates for GUI.
   */
  List<Term> advertisedPredicates = new ArrayList();
  private static Logger log = LoggerFactory.getLogger(AvailableImageProcessors.class);

  public AvailableImageProcessors(final int imgWidth, final int imgHeight, final boolean isStereo, final String configFile) {
    //parse config file
    if (configFile != null && !configFile.isEmpty()) {
      Term advertisement;
      String predicateType;
      String predicateName;
      int arity;
      ImageProcessorType type;
      String filename;
      String[] requirements;
      String[] dependencies;

      try {
        //get directory    
        File file = new File(configFile);
        String dir = file.getParent() + "/";
        //System.out.println("load processors config directory: " + dir);

        //get node
        Xml config = new Xml(configFile, "processors");
        List<Xml> manyProcessors = config.children("processor");
        for (Xml processor : manyProcessors) {

          //get ImageProcessor type
          try {
            type = ImageProcessorType.valueOf(processor.string("type").toUpperCase());
          } catch (Exception e) {
            log.error(String.format("\"%s\" does not name a valid ImageProcessorType. Ignoring requested ImageProcessor.",
                                    processor.string("type")));
            continue;
          }

          //check processor type's runtime requirements
          if (processor.containsAttribute("required")) {
            try {
              requirements = processor.string("required").split(",\\s*");
              boolean requirementsSatisfied = true;
              for (String requirement : requirements) {
                if (!Requirement.isRequirementSatisfied(requirement)) {
                  log.info(type + "'s runtime requirement (" + requirement + ") not satisfied. Skipping this processor type.");
                  requirementsSatisfied = false;
                  break;
                }
              }

              if (!requirementsSatisfied) {
                continue;
              }
            } catch (RuntimeException e) {
              log.error("Could not parse " + type + "'s runtime requirements. Skipping this processor type.");
              continue;
            }
          }

          //get ImageProcessor's image processor dependencies (if any)
          dependencies = null;
          if (processor.containsAttribute("depends")) {
            try {
              dependencies = processor.string("depends").split(",\\s*");
            } catch (RuntimeException e) {
              log.error("Could not parse " + type + "'s dependencies.");
            }
          }

          //get config filename (if any)
          filename = null;
          if (processor.containsAttribute("config")) {
            try {
              filename = processor.string("config");
            } catch (RuntimeException e) {
              log.error("Could not parse " + type + "'s config filename.");
            }
          }

          //add type to available options
          if (!typesDetail.containsKey(type)) {
            ImageProcessorDetail newProcDetail = new ImageProcessorDetail(type, imgWidth, imgHeight, isStereo);
            try {
              if (dependencies != null) {
                for (String dependency : dependencies) {
                  newProcDetail.addDependency(ImageProcessorType.valueOf(dependency.toUpperCase()));
                }
              }
            } catch (Exception e) {
              log.error(String.format("Problem parsing info for \"%s\" detector type. All such detector types will not be loaded.",
                                      type.toString()));
              continue;
            }
            typesDetail.put(type, newProcDetail);
            orderedTypesDetail.add(newProcDetail);
          } else {
            log.warn(String.format("\"%s\" ImageProcessor has already been loaded. Ignoring duplicate request.",
                                   type.toString()));
          }

          //parase ImageProcessor's config file to get info for each predicate option
          try {
            if (filename != null && !filename.isEmpty()) {
              filename = dir + filename;
              Xml ipConfig = new Xml(filename, "processor");
              predicateType = ipConfig.string("type");
              arity = ipConfig.numInt("arity");
              for (Xml predicate : ipConfig.children("predicate")) {

                predicateName = predicate.string("name");
                if (arity == 1) {
                  advertisement = new Predicate(predicateName, new Variable("X", PredicateHelper.varType));
                } else if (arity == 2) {
                  advertisement = new Predicate(predicateName, new Variable("X", PredicateHelper.varType), new Variable("Y", PredicateHelper.varType));
                } else {
                  log.error(String.format("Can't handle Predicates with arity: %d.", arity));
                  continue;
                }

                //testing
                //for (Term n : processorOptionInfo.keySet()) {
                //  if (name.equals(n)) {
                //    System.out.println("ImgProc: " + name.toString() + " ImgProc: " + n.toString() + " Equal");
                //  }
                //}
                //end testing
                //add ImageProcessor option
                HashMap<String, List<InstanceInfo>> descriptorMatches = processorOptionInfo.get(predicateName);
                if (descriptorMatches == null) {
                  descriptorMatches = new HashMap();
                  processorOptionInfo.put(predicateName, descriptorMatches);
                }
                List<InstanceInfo> processorOptions = descriptorMatches.get(predicateType);
                if (processorOptions == null) {
                  processorOptions = new ArrayList();
                  descriptorMatches.put(predicateType, processorOptions);
                  advertisedPredicates.add(advertisement);
                }
                processorOptions.add(new InstanceInfo(type, filename));

              }
              //add config file info for ImageProcessorType
              configFiles.put(type, filename);
            }
          } catch (Exception e) {
            log.error(String.format("Error parsing config file: %s.", filename), e);
          }

        }
      } catch (Exception e) {
        log.error(String.format("Error parsing config file: %s.", configFile), e);
      }
    }

  }

  /**
   * Add an image processor option. This is currently only used for runtime
   * instance learning.
   *
   * @param descriptor new description to add
   * @param type image processor type
   */
  public void addImageProcessorOption(Symbol descriptor, ImageProcessorType type) {
    // convert descriptor to vision form and do some checks
    Term advertisement;
    if (descriptor.isTerm()) {
      advertisement = (Term) descriptor;
    } else {
      List<Term> descriptors = PredicateHelper.convertToVisionForm(descriptor);
      advertisement = descriptors.get(0);
      if (descriptors.size() > 1) {
        log.error("[addImageProcessorOption] can't handle descriptor(s): " + descriptors
                + " attempting to use: " + advertisement);
      }
    }

    // add ImageProcessor option
    String predicateName = advertisement.getName();
    String predicateType = advertisement.getName();  // for now just using name as type since type isn't known
    HashMap<String, List<InstanceInfo>> descriptorMatches = processorOptionInfo.get(predicateName);
    if (descriptorMatches == null) {
      descriptorMatches = new HashMap();
      processorOptionInfo.put(predicateName, descriptorMatches);
    }
    List<InstanceInfo> processorOptions = descriptorMatches.get(predicateType);
    if (processorOptions == null) {
      processorOptions = new ArrayList();
      descriptorMatches.put(predicateType, processorOptions);
      advertisedPredicates.add(advertisement);
    }
    processorOptions.add(new InstanceInfo(type, configFiles.get(type)));
  }

  /**
   * Should only be used by the GUI!
   *
   * @return container of all instantiated ImageProcessors
   */
  public synchronized NonEDTListModel<ImageProcessor> getAll() {
    return orderedTypes;
  }

  /**
   * Get set of available predicates advertised to the rest of the system.
   * Should only be used by CameraConstrol GUI.
   *
   * @return
   */
  public synchronized List<Term> getOptions() {
    return new ArrayList(advertisedPredicates);
  }

  /**
   * Get set of available ImageProcessorTypes available to the system. Not all
   * of them may be advertised as Predicates (eg, SIFT).
   *
   * @return
   */
  public synchronized Set<ImageProcessorType> getTypeOptions() {
    return typesDetail.keySet();
  }

  /**
   * Get instance info for Predicate. If there are multiple options, the first
   * one is returned.
   *
   * @param term
   * @return
   */
  private InstanceInfo getInstanceInfo(final Term term) {
    //get options based on descriptor name
    HashMap<String, List<InstanceInfo>> descriptorMatches = processorOptionInfo.get(PredicateHelper.getRepresentativeString(term));
    if (descriptorMatches == null) {
      log.debug(String.format("[getInstanceInfo] No processors match the description: %s.", term.toString()));
      return null;
    }

    List<InstanceInfo> processorOptions = null;

    if (descriptorMatches.size() > 0) {
      processorOptions = descriptorMatches.values().iterator().next();

      //TODO: how to deal with this case? just use the first one for now
      // if (descriptorMatches.size() > 1)
      //if more than one (predicate type) option
    }

    if (processorOptions == null || processorOptions.isEmpty()) {
      log.debug(String.format("[getInstanceInfo] No processors match the description: %s.", term.toString()));
      return null;
    }

    if (processorOptions.size() > 1) {
      //TODO: how to deal with this case?
      log.info(String.format("[getInstanceInfo] More than one processor matches the "
              + "description: %s. Returning first found matching instance.", term.toString()));
    }

    return processorOptions.get(0);
  }

  /**
   * Is there an ImageProcessor that matches the specified predicate.
   *
   * @param term - Predicate description
   * @return - if one exists.
   */
  public boolean hasCapableProcessorType(final Term term) {
    if (getInstanceInfo(term) == null) {
      return false;
    }

    return true;
  }

  /**
   * Is there an ImageProcessor that matches the specified ImageProcessorType.
   *
   * @param type
   * @return - if type exists.
   */
  public boolean hasCapableProcessorType(final ImageProcessorType type) {
    if (typesDetail.containsKey(type)) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Get ImageProcessorType that matches the specified term.
   *
   * @param term - Predicate description
   * @return - ImageProcessorType that "matches" term. Null if no match.
   */
  public ImageProcessorType getCapableProcessorType(final Term term) {
    InstanceInfo instanceInfo = getInstanceInfo(term);

    if (instanceInfo == null) {
      return null;
    }

    return instanceInfo.type;
  }

  /**
   * Get ImageProcessor instance based on specified term. Returns null if
   * a match cannot be found.
   *
   * @param client
   * @param term
   * @return
   */
  public synchronized ImageProcessor getInstance(final Object client, final Term term) {
    InstanceInfo instanceInfo = getInstanceInfo(term);
    if (instanceInfo == null) {
      return null;
    }

    ImageProcessorType processorType = instanceInfo.type;
    String config = instanceInfo.config;
    return getInstance(client, processorType, term, config);
  }

  /**
   * Get ImageProcessor instance based on requested ImageProcessorType
   *
   * @param client
   * @param processorType
   * @return
   */
  public synchronized ImageProcessor getInstance(final Object client, final ImageProcessorType processorType) {
    return getInstance(client, processorType, null, configFiles.get(processorType));
  }

  /**
   * helper method
   *
   * @param client - object requesting instance
   * @param processorType - ImageProcessorType
   * @param configFile
   * @return
   */
  private synchronized ImageProcessor getInstance(final Object client, final ImageProcessorType processorType, final Term descriptor, final String configFile) {
    ImageProcessorDetail processorDetail = typesDetail.get(processorType);
    if (processorDetail == null) {
      log.error(String.format("[getInstance] No ProcessorDetail of type: %s", processorType.toString()));
      return null;
    }

    boolean singleInstance = false;

    //only single instances of these allowed
    if (processorType == ImageProcessorType.SIFT
            || processorType == ImageProcessorType.PLANE
            || processorType == ImageProcessorType.GLOBALFEATUREVALIDATOR) {
      singleInstance = true;
    }

    //get list of image processsors of specified type
    HashSet<ImageProcessor> processorSet = instantiatedProcessors.get(processorType);
    if (processorSet == null) {
      processorSet = new HashSet();
      instantiatedProcessors.put(processorType, processorSet);
    }

    //find existing, or instantiate new, image processor
    ImageProcessor processorInstance;
    if (processorSet.isEmpty() || !singleInstance) {   //create new instance
      processorInstance = new ImageProcessor(processorDetail, descriptor);
      processorSet.add(processorInstance);
      orderedTypes.add(processorInstance);

      //load config, if specified
      if (configFile != null && !configFile.isEmpty()) {
        //System.out.println("ip getInstance type: " + processorType + " config: " + processorOption.config);
        processorInstance.loadConfig(configFile);
      }
    } else {    //use existing instance
      processorInstance = processorSet.iterator().next();
    }

    addClient(client, processorInstance);
    return processorInstance;
  }

  //helper function to add getInstance caller to client list
  private void addClient(final Object client, final ImageProcessor processor) {
    HashSet<Object> clientList = instantiatedProcessorClients.get(processor);
    if (clientList == null) {
      clientList = new HashSet();
      instantiatedProcessorClients.put(processor, clientList);
    }

    if (clientList.contains(client)) {
      log.warn(String.format("[addClient] Calling object (%s) is already a client of the ImageProcessor: %s.",
                             client.toString(), processor.toString()));
      return;
    } else {
      //add client to ImageProcessor's client list
      clientList.add(client);
    }
  }

  /**
   * Client needs to let managing factory know when it's done using an
   * ImageProcessor instance, so that ImageProcessors can be properly managed
   * and garbage collected.
   */
  public synchronized void release(final Object client, final ImageProcessor processor) {
    HashSet<Object> clientList = instantiatedProcessorClients.get(processor);
    if (clientList == null || !clientList.contains(client)) {
      log.warn(String.format("[release] Calling object (%s) is not a client of the ImageProcessor: %s.",
                             client.toString(), processor.toString()));
      return;
    } else {
      clientList.remove(client);

      //if ImageProcessor doesn't have any clients, clean up references
      if (clientList.isEmpty()) {
        //remove it from all containers so it can be garbage collected
        instantiatedProcessorClients.remove(processor);
        instantiatedProcessors.get(processor.getType()).remove(processor);
        orderedTypes.remove(processor);

        //"delete" processor
        processor.terminate();

        //TODO: if the processor is the last one of its kind (i.e., ImageProcessorTpe),
        //instead of removing from containers, perhaps the processor
        //should be "cleaned up"and kept around for future use, instead
        //of having to instantiate a new one ??
      }
    }
  }

  /**
   * For AbstractListModel. To expose imageProcesstYpes to UI.
   *
   * @param arg0
   * @return
   */
  @Override
  public Object getElementAt(int arg0) {
    return orderedTypesDetail.get(arg0);
  }

  /**
   * For AbstractListModel. To expose imageProcesstYpes to UI.
   *
   * @return
   */
  @Override
  public int getSize() {
    return orderedTypesDetail.size();
  }

  /**
   * Helper class to hold the information needed to instantiate a particular
   * image processor option.
   */
  private class InstanceInfo {

    public final ImageProcessorType type;
    public String config;

    InstanceInfo(final ImageProcessorType processorType, final String configFile) {
      type = processorType;
      config = configFile;
    }
  }
}
