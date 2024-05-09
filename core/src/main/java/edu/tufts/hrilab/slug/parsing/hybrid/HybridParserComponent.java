/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.hybrid;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.NLUInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.parsing.cache.CachedParserComponent;
import edu.tufts.hrilab.slug.parsing.llm.LLMParserComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class HybridParserComponent extends DiarcComponent implements NLUInterface {
  private NLUInterface llmParser;
  private NLUInterface tldlParser;
  private CachedParserComponent cacheParser;

  private String endpoint = "http://localhost:8000/parse/";
  private String tldl = "templatedict.dict";
  private String cacheName = null;
  private String[] cacheLoads = null;
  private String cachePersist = null;
  private String llm = null;
  private String prompt = null;

  private boolean confirmation = true;

  private ExecutorService executor = Executors.newCachedThreadPool();

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("tldl").hasArgs().required().argName("file").desc("load tldl dictionary from file").build());
    options.add(Option.builder("llm").hasArg().argName("string").desc("Name of TRADE Service to call in LLMParserComponent.parseUtterance()").build());
    options.add(Option.builder("prompt").hasArg().argName("string").desc("Prompt to use with LLM parser").build());
    options.add(Option.builder("endpoint").hasArg().argName("string").desc("Endpoint for LLMParserComponent").build());
    options.add(Option.builder("cacheLoad").hasArgs().argName("file").desc("Load cached database from file, can have multiple").build());
    options.add(Option.builder("cacheName").hasArg().argName("string").desc("Name of cache").build());
    options.add(Option.builder("cachePersist").hasArg().argName("file").desc("Persist cache in on users computer").build());
    options.add(Option.builder("noConfirmation").argName("boolean").desc("Ask human to confirm LLM parser response for cache").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("tldl")) {
      tldl = cmdLine.getOptionValue("tldl");
    }
    if (cmdLine.hasOption("llm")) {
      llm = cmdLine.getOptionValue("llm");
    }
    if (cmdLine.hasOption("prompt")) {
      prompt = cmdLine.getOptionValue("prompt");
    }
    if (cmdLine.hasOption("endpoint")) {
      endpoint = cmdLine.getOptionValue("endpoint");
    }
    if (cmdLine.hasOption("cacheName")) {
      cacheName = cmdLine.getOptionValue("cacheName");
    }
    if (cmdLine.hasOption("cacheLoad")) {
      cacheLoads = cmdLine.getOptionValues("cacheLoad");
    }
    if (cmdLine.hasOption("cachePersist")) {
      cachePersist = cmdLine.getOptionValue("cachePersist");
    }
    if (cmdLine.hasOption("noConfirmation")) {
      confirmation = false;
    }
  }

  @Override
  public void init() {
    String llmArgs = "-endpoint " + endpoint;
    String tldlArgs = "-dict " + tldl;
    String cacheArgs = "";
    if (llm != null) {
      llmArgs += " -service " + llm;
    }
    if (prompt != null) {
      llmArgs += " -prompt " + prompt;
    }
    if (cacheName != null) {
      cacheArgs += " -name " + cacheName;
    }
    if (cacheLoads != null) {
      for (String cacheLoad : cacheLoads) {
        cacheArgs += " -load " + cacheLoad;
      }
    }
    if (cachePersist != null) {
      cacheArgs += " -persist " + cachePersist;
    }
    llmParser = createInstance(LLMParserComponent.class, llmArgs, false);
    tldlParser = createInstance(TLDLParserComponent.class, tldlArgs, false);
    cacheParser = createInstance(CachedParserComponent.class, cacheArgs, false);
  }

  /**
   * Parses the incoming Utterance by delegating to multiple parsers asynchronously and returning the result.
   * Checks the cache first and returns if semantics are present.
   * Parses using TLDL second and returns if semantics are present and parse is not UNKNOWN.
   * Tries the LLMParser last and, if valid uses it and caches it.
   *
   * @param incoming The incoming Utterance to parse.
   * @return The parsed Utterance.
   */
  @Override
  public Utterance parseUtterance(Utterance incoming) {
    Future<Utterance> llmFuture = executor.submit(() -> llmParser.parseUtterance(incoming));
    Future<Utterance> tldlFuture = executor.submit(() -> tldlParser.parseUtterance(incoming));
    Future<Utterance> cachedFuture = executor.submit(() -> cacheParser.parseUtterance(incoming));

    Utterance output = incoming;
    Utterance cachedOutput = null;
    Utterance tldlOutput = null;
    Utterance llmOutput = null;

    //Check cache first
    try {
      cachedOutput = cachedFuture.get();
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error getting cached parser results.", e);
    }

    if (cachedOutput != null && cachedOutput.getSemantics() != null) {
      log.debug("Using cached parser response");
      return cachedOutput;
    }

    //Try TLDLParser, return if not null or UNKNOWN
    try {
      tldlOutput = tldlFuture.get();
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error getting tldl parser results.", e);
    }

    if (tldlOutput != null && tldlOutput.getSemantics() != null && tldlOutput.getType() != UtteranceType.UNKNOWN) {
      log.debug("Using tldl parser response");
      cacheParse(tldlOutput);
      return tldlOutput;
    } else if (tldlOutput != null && tldlOutput.getSemantics() != null && tldlOutput.getType() == UtteranceType.UNKNOWN) {
      //Use TLDL UNKNOWN parse in case LLM parse fails
      output = tldlOutput;
    }

    //Try LLMParser last
    try {
      llmOutput = llmFuture.get();
    } catch (InterruptedException | ExecutionException e) {
      log.error("Error getting llm parser results.", e);
    }

    if (llmOutput != null) {
      log.debug("Using llm parser response");
      output = llmOutput;
      if (confirmation) {
        output.setNeedsValidation(true);
      }
    }

    return output;
  }

  @TRADEService
  @Action
  public void cacheParse(Utterance utterance) {
    cacheParser.addEntryToCache(utterance);
  }
}
