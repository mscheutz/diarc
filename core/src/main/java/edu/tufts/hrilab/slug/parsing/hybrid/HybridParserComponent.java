/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.hybrid;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.interfaces.NLUInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.parsing.cache.CachedParserComponent;
import edu.tufts.hrilab.slug.parsing.llm.LLMParserComponent;
import edu.tufts.hrilab.slug.parsing.patternMatching.PatternMatchingParser;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.fol.util.Utilities;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.Pair;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.HashMap;

public class HybridParserComponent extends DiarcComponent implements NLUInterface {

  private boolean useLLM = false;
  private NLUInterface llmParser;
  private boolean useTLDL = false;
  private NLUInterface tldlParser;
  private boolean usePMP = false;
  private NLUInterface patternMatchingParser;
  private boolean useCache = false;
  private CachedParserComponent cacheParser;

  private String endpoint = "http://localhost:8000/parse/";
  private List<String> tldlDictionaries = new ArrayList<>();
  private  HashMap<Symbol, Symbol> addresseeMap = new HashMap<>();
  private boolean tldlUpdateAddressee = true;
  private String cacheName = null;
  private String[] cacheLoads = null;
  private String cachePersist = null;
  private boolean cacheTldl = false;
  private String llm = null;
  private String prompt = null;
  private Symbol unknownListener = null;

  private boolean confirmation = true;

  private ExecutorService executor = Executors.newCachedThreadPool();

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("tldl").hasArgs().required().argName("file").desc("load tldl dictionary from file(s)").build());
    options.add(Option.builder("llm").argName("boolean").desc("Use the LLM parser").build());
    options.add(Option.builder("llmService").hasArg().argName("string").desc("Name of TRADE Service to call in LLMParserComponent.parseUtterance()").build());
    options.add(Option.builder("llmPrompt").hasArg().argName("string").desc("Prompt to use with LLM parser").build());
    options.add(Option.builder("llmEndpoint").hasArg().argName("string").desc("Endpoint for LLMParserComponent").build());
    options.add(Option.builder("cacheLoad").hasArgs().argName("file").desc("Load cached database from file, can have multiple").build());
    options.add(Option.builder("cacheName").hasArg().argName("string").desc("Name of cache").build());
    options.add(Option.builder("cachePersist").hasArg().argName("file").desc("Persist cache in on users computer").build());
    options.add(Option.builder("cacheTldl").desc("Cache TLDL parses").build());
    options.add(Option.builder("noConfirmation").argName("boolean").desc("Ask human to confirm LLM parser response for cache").build());
    options.add(Option.builder("patternMatching").desc("use pattern matching parser").build());
    options.add(Option.builder("tldlNoUpdateAddressee").desc("TLDL: do not update addressee after direct address").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("tldl")) {
      useTLDL = true;
      Arrays.stream(cmdLine.getOptionValues("tldl"))
              .filter(dict -> !dict.isEmpty())
              .forEach(dict -> tldlDictionaries.add("-dict " + dict));
    }
    if (cmdLine.hasOption("tldlNoUpdateAddressee")) {
      tldlUpdateAddressee = false;
    }
    if (cmdLine.hasOption("llm")) {
      useLLM = true;
      log.debug("Using LLM parser");
    }
    if (cmdLine.hasOption("llmService")) {
      llm = cmdLine.getOptionValue("llmService");
      log.debug("Using LLM parser via TRADE Service " + llm);
    }
    if (cmdLine.hasOption("llmPrompt")) {
      prompt = cmdLine.getOptionValue("llmPrompt");
      log.debug("Using LLM parser with prompt " + prompt);
    }
    if (cmdLine.hasOption("llmEndpoint")) {
      endpoint = cmdLine.getOptionValue("llmEndpoint");
      log.debug("Using LLM parser at endpoint " + endpoint);
    }
    if (cmdLine.hasOption("cacheName")) {
      useCache = true;
      cacheName = cmdLine.getOptionValue("cacheName");
      log.debug("Using cache, name " + cacheName);
    }
    if (cmdLine.hasOption("cacheLoad")) {
      cacheLoads = cmdLine.getOptionValues("cacheLoad");
      log.debug("Loading pre-existing cache" + ( cacheLoads.length == 1 ? "" : "s" ) + ": " + String.join(", ", cacheLoads) );
    }
    if (cmdLine.hasOption("cachePersist")) {
      cachePersist = cmdLine.getOptionValue("cachePersist");
      log.debug("Persist cache as a local file in the user $HOME/.diarc/ directory");
    }
    if (cmdLine.hasOption("cacheTldl")) {
      cacheTldl = true;
      log.debug("Cache TLDL parses");
    }
    if (cmdLine.hasOption("noConfirmation")) {
      confirmation = false;
      log.debug("Do not require confirmation from LLM parser before storing in cache");
    }
    if (cmdLine.hasOption("patternMatching")) {
      usePMP = true;
      log.debug("Using the pattern matching parser");
    }
  }

  @Override
  public void init() {
    // init LLMParserComponent
    if (useLLM) {
      String llmArgs = "-endpoint " + endpoint;
      if (llm != null) {
        llmArgs += " -service " + llm;
      }
      if (prompt != null) {
        llmArgs += " -prompt " + prompt;
      }
      llmParser = createInstance(LLMParserComponent.class, llmArgs, false);
    }

    // init TLDLParserComponent
    if (useTLDL) {
      String tldlArgs = String.join(" ", tldlDictionaries);
      if (!tldlUpdateAddressee) {
        tldlArgs += " -noUpdateAddressee";
      }
      tldlParser = createInstance(TLDLParserComponent.class, tldlArgs, false);
    }

    // init PatternMatchingParser
    if (usePMP) {
      patternMatchingParser = createInstance(PatternMatchingParser.class, "", false);
    }

    // init CachedParserComponent
    if (useCache) {
      String cacheArgs = "";
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
      cacheParser = createInstance(CachedParserComponent.class, cacheArgs, false);
    }
    unknownListener = Factory.createSymbol("unknown");
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

    Future<Utterance> llmFuture = null;
    Future<Utterance> tldlFuture = null;
    Future<Utterance> cachedFuture = null;
    Future<Utterance> pmpFuture = null;

    if (useLLM) {
      llmFuture = executor.submit(() -> llmParser.parseUtterance(incoming));
    }
    if (useTLDL) {
      tldlFuture = executor.submit(() -> tldlParser.parseUtterance(incoming));
    }
    if (useCache) {
      cachedFuture = executor.submit(() -> cacheParser.parseUtterance(incoming));
    }
    if (usePMP) {
      pmpFuture = executor.submit(() -> patternMatchingParser.parseUtterance(incoming));
    }

    Utterance output = incoming;
    Utterance cachedOutput = null;
    Utterance pmpOutput = null;
    Utterance tldlOutput = null;
    Utterance llmOutput = null;

    //Check cache first
    if (cachedFuture != null) {
      try {
        cachedOutput = cachedFuture.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[parseUtterance] Error getting cached parser results.", e);
      }

      if (cachedOutput != null && cachedOutput.getSemantics() != null) {
        log.debug("[parseUtterance] Using cached parser response");
        if (Utilities.equalsIgnoreType(cachedOutput.getAddressee(), unknownListener)) {
          if (addresseeMap.containsKey(incoming.getSpeaker())) {
            cachedOutput.setListener(addresseeMap.get(incoming.getSpeaker()));
          } else {
            cachedOutput.setListener(Factory.createSymbol("unknown"));
          }
        }
        return cachedOutput;
      }
    }

    //Try patternmatching parser
    if (pmpFuture != null) {
      try {
        pmpOutput = pmpFuture.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[parseUtterance] Error getting pattern matching parser results.", e);
      }

      //TODO:brad: what should this return in the no match case?
      if (pmpOutput != null) {
        return pmpOutput;
      }
    }

    //Try TLDLParser, return if not null or UNKNOWN
    if (tldlFuture != null) {
      try {
        tldlOutput = tldlFuture.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[parseUtterance] Error getting tldl parser results.", e);
      }

      if (tldlOutput != null && tldlOutput.getSemantics() != null) {
        preserveAddressee(tldlOutput);
        if (tldlOutput.getType() != UtteranceType.UNKNOWN) {
          log.debug("[parseUtterance] Using tldl parser response");
          if (useCache && cacheTldl) {
            cacheParse(tldlOutput);
          }
          return applyAddressee(incoming, tldlOutput);
        } else {
          //Use TLDL UNKNOWN parse in case LLM parse fails
          output = tldlOutput;
        }
      }
    }

    //Try LLMParser last
    if (llmFuture != null) {
      try {
        llmOutput = llmFuture.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[parseUtterance] Error getting llm parser results.", e);
      }

      if (llmOutput != null && llmOutput.getSemantics() != null) {
        log.debug("[parseUtterance] Using llm parser response");
        preserveAddressee(llmOutput);
        output = llmOutput;
        if (confirmation) {
          output.setNeedsValidation(true);
        }
      }
    }

    return applyAddressee(incoming, output);
  }

  /**
   * On parsed utterances with directly-addressed listeners, store a map of the speaker->listener
   * so that parsed utterances with "unknown" listeners can be inferred from the map.
   *
   * @param parsedUtterance
   */
  private void preserveAddressee (Utterance parsedUtterance) {
    Symbol semantics = parsedUtterance.getSemantics();

    if (semantics != null && semantics.isTerm()) {
      if (!Utilities.equalsIgnoreType(parsedUtterance.getAddressee(), unknownListener)) {
        addresseeMap.put(parsedUtterance.getSpeaker(), parsedUtterance.getAddressee());
        log.debug("[preserveAddressee] Mapping unknown utterances from speaker " + parsedUtterance.getSpeaker().toString() + " to listener " + parsedUtterance.getAddressee().toString());
      }
    }
  }

  /**
   * In the case that an incoming utterance has an "unknown" listener and there is an existing
   * key in the speaker->listener addresseeMap, apply the inferred listener to the parsed
   * utterance.
   *
   * @param incomingUtterance
   * @param parsedUtterance
   * @return
   */
  private Utterance applyAddressee (Utterance incomingUtterance, Utterance parsedUtterance) {
    if (!Utilities.equalsIgnoreType(incomingUtterance.getAddressee(), unknownListener)) {
      return parsedUtterance;
    }
    Symbol addressee = Factory.createSymbol("unknown");
    if (addresseeMap.containsKey(parsedUtterance.getSpeaker())) {
      addressee = addresseeMap.get(parsedUtterance.getSpeaker());
      log.debug("[applyAddressee] Updated utterance from speaker " + parsedUtterance.getSpeaker().toString() + " to address " + addressee.toString());
    }
    parsedUtterance.setListener(addressee);
    Symbol semantics = parsedUtterance.getSemantics();
    if (semantics != null && semantics.isTerm()) {
      List<Symbol> args = ((Term) semantics).getArgs();
      args.set(0, addressee);
      parsedUtterance.setSemantics(Factory.createPredicate(semantics.getName(), args));
    }
    return parsedUtterance;
  }

  @TRADEService
  @Action
  public void cacheParse(Utterance utterance) {
    cacheParser.addEntryToCache(utterance);
  }

  @TRADEService
  @Action
  public void injectDictionaryEntry(String morpheme, String type, String semantics, String cognitiveStatus) {
    if (useTLDL) {
      ((TLDLParserComponent) tldlParser).injectDictionaryEntry(morpheme, type, semantics, cognitiveStatus);
    } else {
      log.warn("[injectDictionaryEntry] trying to update TLDL parser dictionary when not using TLDL parser");
    }
  }

  @TRADEService
  public boolean activatePattern(String patternName) {
    if (usePMP) {
      return ((PatternMatchingParser) patternMatchingParser).activatePattern(patternName);
    } else {
      log.warn("[activatePattern] trying to activate parsing pattern when not using pattern matching parser");
      return false;
    }
  }

}
