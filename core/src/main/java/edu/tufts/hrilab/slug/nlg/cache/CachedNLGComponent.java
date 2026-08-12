/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.nlg.cache;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.slug.cache.CacheDatabase;
import edu.tufts.hrilab.slug.common.Utterance;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class CachedNLGComponent extends DiarcComponent {
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
        database = new CacheDatabase(name, loadPaths, persist, true);
    }

    @TRADEService
    @Action
    public Utterance convertSemanticsToText(Utterance incoming) {
        return database.get(incoming,language);
    }

    /**
     * Adds an Utterance entry to the cache database and logs the action.
     *
     * @param entry The Utterance object to add to the cache.
     */
    @TRADEService
    public void addEntryToCache(Utterance entry) {
        database.add(entry,language);
    }

    //TODO: should eventually remove this and use utterance language, see CacheDatabase
    public void setLanguage(String language) {
      this.language = language;
    }
}
