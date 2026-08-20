/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.cache;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
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
    //TODO: need to generalize this and make sure the cache cannot store hits for focused template actions. Or things
    //  need to be updated such that LLM state is still appropriately updated (with any relevant context included) in
    //  the case of external hits
    if (entry != null && entry.getSemantics() != null) {
      String name = entry.getSemantics().getName();
      if (name.equals("learnMission") || name.equals("learnTaskSequence") || name.equals("scheduleTask")) {
        log.debug("[addEntryToCache] not adding entry for focused template context target");
        return;
      }
    }
    database.add(entry, language);
  }

  //TODO: should eventually remove this and use utterance language, see CacheDatabase
  public void setLanguage(String language) {
    this.language = language;
  }
}
