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
  private HashMap<Symbol, Symbol> addresseeMap = new HashMap<>();
  private boolean updateAddressee = true;
  private String cacheName = null;
  private String[] cacheLoads = null;
  private boolean cachePersist = false;
  private boolean cacheTldl = false;
  private String llm = null;
  private String prompt = null;
  private Symbol unknownListener = Factory.createSymbol("unknown");
  private Class matcherclass = PatternMatchingParser.class;

  // we have four parsers for now
  private String[] sequence;

  private boolean confirmation = true;

  private ExecutorService executor = Executors.newCachedThreadPool();

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("tldl").hasArgs().argName("file").desc("load tldl dictionary from file(s)").build());
    options.add(Option.builder("llm").argName("boolean").desc("Use the LLM parser").build());
    options.add(Option.builder("cache").argName("boolean").desc("Use the cache").build());
    options.add(Option.builder("cacheLoad").hasArgs().argName("file").desc("Load cached database from file, can have multiple").build());
    options.add(Option.builder("cacheName").hasArg().argName("string").desc("Name of cache").build());
    options.add(Option.builder("cachePersist").argName("file").desc("Persist cache in on users computer").build());
    options.add(Option.builder("cacheTldl").desc("Cache TLDL parses").build());
    options.add(Option.builder("noConfirmation").argName("boolean").desc("Ask human to confirm LLM parser response for cache").build());
    options.add(Option.builder("patternMatching").argName("boolean").desc("use pattern matching parser").build());
    options.add(Option.builder("matcher").hasArg().argName("string").desc("Optional name of Pattern Matcher Component to be used").build());
    options.add(Option.builder("noUpdateAddressee").desc("do not update addressee after direct address (TLDL and PM)").build());
    options.add(Option.builder("parsingSequence").hasArgs().argName("option1 option2 ...").desc("Sequence parsers (default cache, tldl, pattern, llm)").build());
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
    if (cmdLine.hasOption("noUpdateAddressee")) {
      updateAddressee = false;
    }
    if (cmdLine.hasOption("llm")) {
      useLLM = true;
      log.debug("Using LLM parser");
    }
    if (cmdLine.hasOption("cache")) {
      useCache = true;
      log.debug("Using cache");
    }
    if (useCache && cmdLine.hasOption("cacheName")) {
      cacheName = cmdLine.getOptionValue("cacheName");
      log.debug("Naming cache " + cacheName);
    }
    if (useCache && cmdLine.hasOption("cacheLoad")) {
      cacheLoads = cmdLine.getOptionValues("cacheLoad");
      log.debug("Loading pre-existing cache" + (cacheLoads.length == 1 ? "" : "s") + ": " + String.join(", ", cacheLoads));
    }
    if (useCache && cmdLine.hasOption("cachePersist")) {
      cachePersist = true;
      log.debug("Persist cache as a local file $HOME/.diarc/" + (cacheName != null ? cacheName : "diarc-parser-cache") + ".sqlite");
    }
    if (useCache && cmdLine.hasOption("cacheTldl")) {
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
    if (usePMP && cmdLine.hasOption("matcher")) {
      try {
        matcherclass = Class.forName("edu.tufts.hrilab.slug.parsing.patternMatching." + cmdLine.getOptionValue("matcher"));
        log.info("Using pattern matcher " + matcherclass);
      } catch (ClassNotFoundException cnfe) {
        log.error("Could not find pattern matcher {}. Using default.", cmdLine.getOptionValue("matcher"));
      }
    }

    // generate the sequence information
    if (cmdLine.hasOption("parsingSequence")) {
      sequence = cmdLine.getOptionValues("parsingSequence");
    } else {
      sequence = new String[]{"cache", "tldl", "pattern", "llm"};
    }
    log.info("Using sequence: {}", Arrays.toString(sequence));
  }

  @Override
  public void init() {
    // init LLMParserComponent
    if (useLLM) {
      llmParser = createInstance(LLMParserComponent.class, "", false);
    }

    // init TLDLParserComponent
    if (useTLDL) {
      String tldlArgs = String.join(" ", tldlDictionaries);
      if (!updateAddressee) {
        tldlArgs += " -noUpdateAddressee";
      }
      tldlParser = createInstance(TLDLParserComponent.class, tldlArgs, false);
    }

    // init PatternMatchingParser
    if (usePMP) {
      String pmArgs = "";
      if (!updateAddressee) {
        pmArgs += " -noUpdateAddressee";
      }
      patternMatchingParser = (NLUInterface) createInstance(matcherclass, pmArgs, false);
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
      if (cachePersist) {
        cacheArgs += " -persist";
      }
      cacheParser = createInstance(CachedParserComponent.class, cacheArgs, false);
    }
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
      llmFuture = executor.submit(() -> llmParser.parseUtterance(new Utterance.Builder(incoming).build()));
    }
    if (useTLDL) {
      tldlFuture = executor.submit(() -> tldlParser.parseUtterance(new Utterance.Builder(incoming).build()));
    }
    if (useCache) {
      cachedFuture = executor.submit(() -> cacheParser.parseUtterance(new Utterance.Builder(incoming).build()));
    }
    if (usePMP) {
      pmpFuture = executor.submit(() -> patternMatchingParser.parseUtterance(new Utterance.Builder(incoming).build()));
    }

    Utterance output = incoming;
    Utterance cachedOutput = null;
    Utterance pmpOutput = null;
    Utterance tldlOutput = null;
    Utterance llmOutput = null;

    // 
    for (int i = 0; i < sequence.length; i++) {
      switch (sequence[i]) {
        case "cache":
          //Check cache first
          if (cachedFuture != null) {
            try {
              cachedOutput = cachedFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[parseUtterance] Error getting cached parser results.", e);
            }

            if (cachedOutput != null && cachedOutput.getSemantics() != null) {
              output = applyAddressee(incoming, cachedOutput);
              log.debug("[parseUtterance] Using cached parser response: {}", output);
              return output;
            }
          }
          break;
        case "pattern":
          //Try patternmatching parser
          if (pmpFuture != null) {
            try {
              pmpOutput = pmpFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[parseUtterance] Error getting pattern matching parser results.", e);
            }

            //TODO:brad: what should this return in the no match case?
            if (pmpOutput != null && pmpOutput.getSemantics() != null) {
              log.debug("[parseUtterance] Using pattern matcher parser response: {}", pmpOutput);
              return pmpOutput;
            }
          }
          break;
        case "tldl":
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
                if (useCache && cacheTldl) {
                  cacheParse(tldlOutput);
                }
                output = applyAddressee(incoming, tldlOutput);
                log.debug("[parseUtterance] Using tldl parser response: {}", output);
                return output;
              } else {
                //Use TLDL UNKNOWN parse in case LLM parse fails
                output = tldlOutput;
              }
            }
          }
          break;
        case "llm":
          //Try LLMParser last
          if (llmFuture != null) {
            try {
              llmOutput = llmFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[parseUtterance] Error getting llm parser results.", e);
            }

            if (llmOutput != null && llmOutput.getSemantics() != null) {
              preserveAddressee(llmOutput);
              output = llmOutput;
              if (confirmation) {
                output.setNeedsValidation(true);
              } else if (useCache) {
                cacheParse(output);
              }
              log.debug("[parseUtterance] Using llm parser response: {}", output);
              return output;
            }
          }
        default:
          break;
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
  private void preserveAddressee(Utterance parsedUtterance) {
    Symbol semantics = parsedUtterance.getSemantics();
    Symbol addressee = null;
    if (semantics != null && semantics.isTerm()) {
      log.debug("[preserveAddressee] addressee = " + parsedUtterance.getAddressee().toString());
      //addressee = parsedUtterance.getAddressee();
      //until TLDLParserComponent consistently sets listener check first argument in semantics
      addressee = ((Term) semantics).getArgs().get(0);
      log.debug("[preserveAddressee] semantics first arg " + addressee.toString());
      if (!Utilities.equalsIgnoreType(addressee, unknownListener)) {
        addresseeMap.put(parsedUtterance.getSpeaker(), addressee);
        log.debug("[preserveAddressee] Mapping unknown utterances from speaker " + parsedUtterance.getSpeaker().toString() + " to listener " + addressee.toString());
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
  private Utterance applyAddressee(Utterance incomingUtterance, Utterance parsedUtterance) {
    if (!Utilities.equalsIgnoreType(incomingUtterance.getAddressee(), unknownListener)) {
      return parsedUtterance;
    }
    Symbol addressee = Factory.createSymbol("unknown");
    if (addresseeMap.containsKey(parsedUtterance.getSpeaker())) {
      addressee = addresseeMap.get(parsedUtterance.getSpeaker());
      log.debug("[applyAddressee] Updated utterance from speaker " + parsedUtterance.getSpeaker().toString() + " to address " + addressee.toString());
    }
    parsedUtterance.setAddressee(addressee);
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
    log.debug("[cacheParse] Caching parsed utterance");
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

  @Override
  protected void shutdownComponent() {
    super.shutdownComponent();
    if (useTLDL) {
      try {
        ((TLDLParserComponent) tldlParser).deregisterDictionary();
      } catch (Exception e) {
        log.error("[TLDLParserComponent] exception deregistering dictionary", e);
      }
    }
    executor.shutdownNow();
  }
}
