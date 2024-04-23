/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.db;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.db.performanceMeasures.PerformanceMeasures;
import edu.tufts.hrilab.action.db.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.stream.JsonWriter;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class PerformanceMeasuresDatabase {
  private final static Logger log = LoggerFactory.getLogger(PerformanceMeasuresDatabase.class);

  /**
   * Map of all action success and effect probabilities keyed by action name
   */
  private final Map<ActionDBEntry, PerformanceMeasures> performanceMeasuresMap = new HashMap<>();

  private final Map<Predicate, JsonElement> unmatchedPerformanceEntries = new HashMap<>();

  /**
   * Protected so that only the Database can instantiate.
   */
  protected PerformanceMeasuresDatabase() {
  }

  protected synchronized void createPerformanceMeasuresEntry(ActionDBEntry entry) {
    if (performanceMeasuresMap.containsKey(entry)) {
      log.warn("Trying to create performance measures tree for action which already exists: " + entry.getName());
      return;
    }

    // check if new entry has unmatched performance measures info (which is usually the case for primitive actions that
    // get registered after performance measures are loaded)
    PerformanceMeasures performanceMeasures = new PerformanceMeasures(entry);
    if (!unmatchedPerformanceEntries.isEmpty()) {
      // put signature options into map, so we can use filterEntries method below
      List<Pair<Predicate, ActionDBEntry>> map = new ArrayList<>();
      entry.getSignatureOptions(true).forEach(signature -> map.add(Pair.of(signature, entry)));

      for (Predicate signature : unmatchedPerformanceEntries.keySet()) {
        Symbol actor = signature.get(0);
        if (!Utilities.filterEntries(signature, map, actor).isEmpty()) {
          // matched with unmatchedPerformanceEntry -- parse json get the probability measures of the action
          performanceMeasures.populatePerformanceMeasures(unmatchedPerformanceEntries.get(signature).getAsJsonObject());
          break;
        }
      }
    }

    // add new dbEntry to performanceMeasures
    performanceMeasuresMap.put(entry, performanceMeasures);
  }

  /**
   * Get the performance measures for an action db entry.
   *
   * @param adbe action used to get performance measures
   */
  public synchronized PerformanceMeasures getPerformanceMeasures(ActionDBEntry adbe) {
    return performanceMeasuresMap.get(adbe);
  }

  @TRADEService
  public synchronized void savePerformanceMeasures(String path, String filename, Symbol agent) {
    String[] fileInfo = filename.split("\\.");
    if (!fileInfo[fileInfo.length - 1].equals("json")) {
      filename = filename + ".json";
    }

    String resourcePath = path + File.separator + filename;
    BufferedWriter bufferedWriter = null;
    JsonWriter jsonWriter = null;
    try {
      bufferedWriter = new BufferedWriter(new FileWriter(resourcePath));
      jsonWriter = new JsonWriter(bufferedWriter);
      jsonWriter.beginObject();
      for (Map.Entry<ActionDBEntry, PerformanceMeasures> entry : performanceMeasuresMap.entrySet()) {
        if (agent == null || entry.getKey().getAgents().contains(agent)) {
          entry.getValue().appendPerformanceMeasuresToJsonWriter(jsonWriter);
        }
      }
      jsonWriter.endObject();
    } catch (Exception e) {
      log.error("Error writing performance models to file.", e);
    } finally {
      try {
        if (jsonWriter != null) {
          jsonWriter.close();
        } else if (bufferedWriter != null) {
          bufferedWriter.close();
        }
      } catch (IOException e) {
        log.warn("No action performance measures file: " + filename);
      }
    }
  }

  @TRADEService
  public void savePerformanceMeasures(String path, String filename) {
    savePerformanceMeasures(path, filename, null);
  }

  /**
   * load the performance models from a file.
   *
   *  FIXME: need better way to load performance models. The current implementation assumes single db entry per action
   *         However, there will be different db entries for different agents.
   *
   * @param fileDir  file containing performance models
   * @param filename file containing performance models
   */
  public void loadPerformanceMeasures(String fileDir, String filename) {
    String filepath = Resources.createFilepath(fileDir, filename);
    BufferedReader reader = new BufferedReader((new InputStreamReader(
            PerformanceMeasures.class.getResourceAsStream(filepath))));
    String jsonString = reader.lines().collect(Collectors.joining()); // get string of json
    JsonParser parser = new JsonParser();
    JsonObject rootObj = parser.parse(jsonString).getAsJsonObject(); //parse json string
    for (Map.Entry<String, JsonElement> action : rootObj.entrySet()) {
      parseJsonAction(action); // parse each action
    }
  }

  private void parseJsonAction(Map.Entry<String, JsonElement> action) {
    Predicate actionSignature = Factory.createPredicate(action.getKey());
    List<ActionDBEntry> dbEntries = Database.getActionDB().getActionsBySignature(actionSignature);
    //List<ActionDBEntry> actionDBEntries = singleton.actionDB.get(action.getKey()); // get dbentry for action
    if (dbEntries != null && !dbEntries.isEmpty()) {
      //looping through all dbentries. need to
      // FIXME: need to ensure updating correct performance measures
      //        if there are multiple entries with the same name and number of arguments
      //        then how does the system know which is the correct one?
      for (ActionDBEntry entry : dbEntries) {
        PerformanceMeasures performanceMeasures = getPerformanceMeasures(entry);
        // parse json get the probability measures of the action: success, effects
        performanceMeasures.populatePerformanceMeasures(action.getValue().getAsJsonObject());
      }
    } else {
      unmatchedPerformanceEntries.put(actionSignature, action.getValue());
    }
  }

}
