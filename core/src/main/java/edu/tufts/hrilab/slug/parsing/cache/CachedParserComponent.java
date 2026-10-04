/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.cache;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.notification.NotificationType;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.slug.cache.CacheDatabase;
import edu.tufts.hrilab.slug.common.Utterance;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class CachedParserComponent extends DiarcComponent {
  private String name = null;
  private String[] loadPaths = null;
  private boolean persist = false;
  private CacheDatabase database;
  private String language = "English";
  private boolean addedActionListeners = false;

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("name").hasArg().argName("string").desc("Name of cached dictionary to store").build());
    options.add(Option.builder("load").hasArgs().argName("file").desc("Name of cached dictionary to load, can have multiple").build());
    options.add(Option.builder("persist").argName("boolean").desc("Whether to persist cache in a named file").build());
    options.add(Option.builder("language").argName("string").desc("Initial language to set in stored/retrieved utterances").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("name")) {
      name = cmdLine.getOptionValue("name");
    }
    if (cmdLine.hasOption("load")) {
      loadPaths = cmdLine.getOptionValues("load");
    }
    if (cmdLine.hasOption("persist")) {
      persist = true;
    }
    if (cmdLine.hasOption("language")) {
      language = cmdLine.getOptionValue("language");
    }
  }

  @Override
  protected void init () {
    database = new CacheDatabase(name, loadPaths, persist, false);
  }

  /**
   * Parses the incoming Utterance by retrieving its details from the database.
   *
   * @param incoming The incoming Utterance to parse.
   * @return The parsed Utterance with updated semantics and bindings if found in the database; otherwise, returns the incoming Utterance unchanged.
   */
  @TRADEService
  @Action
  public Utterance parseUtterance(Utterance incoming) {
    Utterance outgoing = database.get(incoming,language);
    if (outgoing != null) {
      incoming.setSemantics(outgoing.getSemantics());
      incoming.setIndirectSemantics(outgoing.getIndirectSemantics());
      incoming.setBindings(outgoing.getBindings());
    }
    return incoming;
  }

  /**
   * Adds an Utterance entry to the cache database and logs the action.
   *
   * @param entry The Utterance object to add to the cache.
   */
  @TRADEService
  public void addEntryToCache(Utterance entry) {
    // TODO: Should have a general method to determine what parses are not cacheable
    if (entry != null && entry.getSemantics() != null) {
      String name = entry.getSemantics().getName();
      // Do not cache focused context transfer actions
      // TODO: or allow this but find a way to make sure SocialLLM state is still appropriately updated with any relevant context before transferring
      // TODO: Generally define a "focused context" action rather than hardcode
      if (name.equals("learnMission") || name.equals("learnTaskSequence") || name.equals("startSchedulingTask") || name.equals("modifyExistingTaskStatus") || name.equals("startLearningNewTask")) {
        log.debug("[addEntryToCache] not adding entry for focused template context target");
        return;
      }
      // TODO: nothing around taught actions should be cached OR everything associated with a taught action should be deleted upon forgetting it
      if (name.equals("forgetTaughtAction")) {
        //getActionSteps
        log.debug("[addEntryToCache] not adding entry pointing at taught action which is being deleted");
        return;
      }

      // Do no cache SocialLLM "text response" parses - generateResponseFromString || updateDialogueHistoryFromString
      if (name.equals("generateResponseFromString") || name.equals("updateDialogueHistoryFromString")) {
        log.debug("[addEntryToCache] not adding entry for text response target");
        return;
      }
      //TODO: Keep an eye on validation question response parses - how much can context change the base yes/no distinction of a single statement?
    }
    database.add(entry, language);
  }

  //TODO: should eventually remove this and use utterance language, see CacheDatabase
  public void setLanguage(String language) {
    this.language = language;
  }

  public void removeDeletedActionCacheEntries(ActionDBEntry entry) {
    database.removeRelatedEntries(entry.getSignature(true));
  }
}
