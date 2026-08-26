/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.hybrid;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.parsing.cache.CachedParserComponent;
import edu.tufts.hrilab.slug.parsing.llm.LLMParserComponent;
import edu.tufts.hrilab.slug.parsing.patternMatching.PatternMatchingParser;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.fol.util.Utilities;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.HashMap;

public class HybridParserComponent extends DiarcComponent {

  private boolean useLLM = false;
  private LLMParserComponent llmParser;
  private boolean useTLDL = false;
  private TLDLParserComponent tldlParser;
  private boolean usePMP = false;
  private PatternMatchingParser patternMatchingParser;
  private boolean useCache = false;
  private CachedParserComponent cacheParser;

  private String endpoint = "http://localhost:8000/parse/";
  private List<String> tldlDictionaries = new ArrayList<>();
  private  HashMap<Symbol, Symbol> addresseeMap = new HashMap<>();
  private boolean tldlUpdateAddressee = true;
  private String cacheName = null;
  private String[] cacheLoads = null;
  private boolean cachePersist = false;
  private boolean cacheTldl = false;
  protected boolean cacheLLM = true;
  private String llm = null;
  private String prompt = null;
  private Symbol unknownListener = null;
  private boolean confirmation = true;
  // Intended sequences:
  // "Social LLM Service Provider": Cache -> TLDL -> PMP -> LLM if first input (or really whatever the user desires). But if in an extended interaction, override LLM -> rest.
  // "Basic LLM Service Provider": Always Cache -> TLDL -> PMP -> LLM
  private String[] sequence;
  private boolean restrictedMode = false;
  private String[] cachedSequence = null;

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
    options.add(Option.builder("noCacheLLM").desc("Do not cache LLM parses directly in this component when confirmation is turned off").build());
    options.add(Option.builder("noConfirmation").argName("boolean").desc("Ask human to confirm LLM parser response for cache").build());
    options.add(Option.builder("patternMatching").desc("use pattern matching parser").build());
    options.add(Option.builder("tldlNoUpdateAddressee").desc("TLDL: do not update addressee after direct address").build());
    options.add(Option.builder("sequence").hasArgs().argName("option1 option2 ...").desc("Sequence parsers (default cache, tldl, pattern, llm)").build());
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
      log.debug("Loading pre-existing cache" + ( cacheLoads.length == 1 ? "" : "s" ) + ": " + String.join(", ", cacheLoads) );
    }
    if (useCache && cmdLine.hasOption("cachePersist")) {
      cachePersist = true;
      log.debug("Persist cache as a local file $HOME/.diarc/" + (cacheName != null ? cacheName : "diarc-parser-cache") + ".sqlite");
    }
    if (useCache && cmdLine.hasOption("cacheTldl")) {
      cacheTldl = true;
      log.debug("Cache TLDL parses");
    }
    if (useCache && cmdLine.hasOption("noCacheLLM")) {
      cacheLLM = false;
      log.debug("Not caching LLM parses here (may still occur in nlu.asl)");
    }
    if (cmdLine.hasOption("noConfirmation")) {
      confirmation = false;
      log.debug("Do not require confirmation from LLM parser before storing in cache");
    }
    if (cmdLine.hasOption("patternMatching")) {
      usePMP = true;
      log.debug("Using the pattern matching parser");
    }
    if (cmdLine.hasOption("sequence")) {
      sequence = cmdLine.getOptionValues("sequence");
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
      if (cachePersist) {
        cacheArgs += " -persist";
      }
      cacheParser = createInstance(CachedParserComponent.class, cacheArgs, false);
    }
    unknownListener = Factory.createSymbol("unknown");
  }

  // TODO: This new API currently assumes streaming TTS and filler phrases are only associated with the LLMParser/ServiceProvider (if a social subclass), could be generalized
  //    to any component(s) if that functionality is implemented. Also could generally change details about what components are triggered/checked synchronously vs async.
  // Just calling parseUtterance generally, no associated filler/llm streaming response. Prevent streaming output.
  @TRADEService
  @Action
  public Utterance parseUtterance(Utterance incoming) {
    Utterance output = parseUtterance(incoming, false, false);
    return output == null ? incoming : output;
  }

  // TODO: Depending on desired implementation, this method can trigger the LLM component to start running asynchronously or just only run the non-llm components first. (Or make that behavior configurable)
  //  (The callStreaming argument is only actually necessary if the LLM request is made asynchronously from this method rather than synchronously from getLLMParseResult)
  // Call parseUtterance for any valid non-LLM output. This should be used when the result is possibly tied to speech output and streaming TTS or filler utterances are associated.
  //  Need to run for non-llm result first to know if streaming TTS or filler phrases should be produced.
  @TRADEService
  public Utterance parseUtteranceForNonLLMResult(Utterance incoming, boolean callStreaming) {
    return parseUtterance(incoming, true, callStreaming);
  }

  // Associated with parseUtteranceForNonLLMResult - Get Trailing LLM result after checking non-llm success
  @TRADEService
  public Utterance getLLMParseResult(Utterance incoming, boolean callStreaming) {
    log.info("[getLLMParseResult] in method");

    // If starting the LLM asynchronously
    //*TODO: Would need to store futures and index on utteranceID to retrieve here(and probably move unblock/cancelTTSOutputForUID to this component and rename)

    // If only running the LLM synchronously
    Utterance llmInputUtterance = new Utterance.Builder(incoming).build();
    if (confirmation) {
      llmInputUtterance.setNeedsValidation(true);
    }
    Utterance output = llmParser.parseUtterance(llmInputUtterance, callStreaming);

    //Copying missed control flow from below sequence handling, revisit
    if (output != null) {
      if (output.getType() == UtteranceType.UNKNOWN) {
        //TODO: revisit, using a hack to allow for LLM to return error responses as strings. A better way is
        //  to just return semantics but populate the cache or make sure SimpleNLG can handle them properly
        //Use LLM UNKNOWN parse in case all other parsers fail
        output.setType(UtteranceType.INSTRUCT);
      } else {
        if (!confirmation && cacheLLM && !output.getWordsAsString().isEmpty()) {
          cacheParse(output);
        }

        try {
          TRADE.getAvailableService(new TRADEServiceConstraints().name("usedLLMParse").argTypes(Symbol.class, Symbol.class)).call(void.class, incoming.getSpeaker(), incoming.getAddressee());
        } catch (TRADEException ex) {
          log.error("Error calling usedLLMParse service", ex);
        }
      }
    }
    return output;
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
  private Utterance parseUtterance(Utterance incoming, boolean returnNonLLM, boolean callStreamingMethod) {
    Future<Utterance> llmFuture = null;
    Future<Utterance> tldlFuture = null;
    Future<Utterance> cachedFuture = null;
    Future<Utterance> pmpFuture = null;

    String[] parseSequence = sequence;
    boolean inExtendedParseLoop = false;

    // If LLM spawned async
    //if (useLLM) {
    // If LLM synchronously only after other components fail
    if (useLLM) {
      try {
         inExtendedParseLoop = TRADE.getAvailableService(new TRADEServiceConstraints().name("inExtendedParseLoop").argTypes(Symbol.class, Symbol.class)).call(boolean.class, incoming.getSpeaker(), incoming.getAddressee());
      } catch (TRADEException ex) {
        log.error("Error calling inExtendedParseLoop service", ex);
      }

      if (!returnNonLLM) {
        Utterance llmInputUtterance = new Utterance.Builder(incoming).build();
        if (confirmation) {
          llmInputUtterance.setNeedsValidation(true);
        }
        llmFuture = executor.submit(() -> llmParser.parseUtterance(llmInputUtterance, callStreamingMethod));
      }
    }

    // If in an extended Social LLM parse loop, temporarily ignore other parsers
    if (!inExtendedParseLoop) {
      if (useTLDL) {
        tldlFuture = executor.submit(() -> tldlParser.parseUtterance(new Utterance.Builder(incoming).build()));
      }
      if (useCache) {
        cachedFuture = executor.submit(() -> cacheParser.parseUtterance(new Utterance.Builder(incoming).build()));
      }
      if (usePMP) {
        pmpFuture = executor.submit(() -> patternMatchingParser.parseUtterance(new Utterance.Builder(incoming).build()));
      }
    }

    //Utterance output = incoming;
    Utterance output = null;
    Utterance cachedOutput = null;
    Utterance pmpOutput = null;
    Utterance tldlOutput = null;
    Utterance llmOutput = null;

    for (int i = 0; i < parseSequence.length; i++) {
      switch (parseSequence[i]) {
        case "cache":
          if (cachedFuture != null) {
            try {
              cachedOutput = cachedFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[parseUtterance] Error getting cached parser results.", e);
            }

            if (cachedOutput != null && cachedOutput.getSemantics() != null) {
              output = applyAddressee(incoming, cachedOutput);
              log.info("[parseUtterance] Using cached parser response: {}", output);
              return output;
            }
          }
          break;
        case "pattern":
          if (pmpFuture != null) {
            try {
              pmpOutput = pmpFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[parseUtterance] Error getting pattern matching parser results.", e);
            }

            //TODO:brad: what should this return in the no match case?
            if (pmpOutput != null && pmpOutput.getSemantics() != null) {
              log.info("[parseUtterance] Using pattern matcher parser response: {}", pmpOutput);
              return pmpOutput;
            }
          }
          break;
        case "tldl":
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
                log.info("[parseUtterance] Using tldl parser response: {}", output);
                return output;
              } else {
                if (output == null && !useLLM) {
                  //Use TLDL UNKNOWN parse in case LLM parse fails
                  output = tldlOutput;
                }
              }
            }
          }
          break;
        case "llm":
          if (llmFuture != null && !returnNonLLM) {
            try {
              llmOutput = llmFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[parseUtterance] Error getting llm parser results.", e);
            }

            if (llmOutput != null && llmOutput.getSemantics() != null) {
              if (llmOutput.getType() != UtteranceType.UNKNOWN) {
                log.info("[parseUtterance] Using llm parser response");
                preserveAddressee(llmOutput);
                output = llmOutput;
                //TODO: inExtendedParseLoop may be incorrect to use here - may have to revert to explicitly checking Chat context message length again
                if (!inExtendedParseLoop && !confirmation && useCache && cacheLLM) {
                  cacheParse(output);
                }
                // Same as above comment - in a crunch, redo
                try {
                  TRADE.getAvailableService(new TRADEServiceConstraints().name("usedLLMParse").argTypes(Symbol.class, Symbol.class)).call(void.class, incoming.getSpeaker(), incoming.getAddressee());
                } catch (TRADEException ex) {
                  log.error("Error calling usedLLMParse service", ex);
                }
                return output;
              } else {
                if (output == null) {
                  //TODO: revisit, using a hack to allow for LLM to return error responses as strings. A better way is
                  //  to just return semantics but populate the cache or make sure SimpleNLG can handle them properly
                  llmOutput.setType(UtteranceType.INSTRUCT);
                  //Use LLM UNKNOWN parse in case all other parsers fail
                  output = llmOutput;
                }
              }
            }
          }
          break;
        default:
          break;
      }
    }
    if (output == null) {
      return null;
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
    Symbol addressee = null;
    if (semantics != null && semantics.isTerm() && semantics.hasArgs()) {
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
  private Utterance applyAddressee (Utterance incomingUtterance, Utterance parsedUtterance) {
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
    if (useCache) {
      cacheParser.addEntryToCache(utterance);
    } else {
      log.warn("[cacheParse] attempting to add utterance to cache but parser cache is disabled");
    }
  }

  @TRADEService
  @Action
  public void cacheLLMParse(Utterance utterance) {
    if (cacheLLM) {
      if (useCache) {
        log.debug("[cacheLLMParse] Caching parsed utterance");
        cacheParser.addEntryToCache(utterance);
      } else {
        log.warn("[cacheLLMParse] attempting to add utterance to cache but parser cache is disabled");
      }
    }
  }

  @TRADEService
  @Action
  public void injectDictionaryEntry(String morpheme, String type, String semantics, String cognitiveStatus) {
    if (useTLDL) {
      ((TLDLParserComponent) tldlParser).injectDictionaryEntry(morpheme, type, semantics, cognitiveStatus);
    } else {
      log.debug("[injectDictionaryEntry] trying to update TLDL parser dictionary when not using TLDL parser");
    }
  }

  @Override
  protected void shutdownComponent() {
    super.shutdownComponent();
    if (useTLDL) {
      try {
        ((TLDLParserComponent)tldlParser).deregisterDictionary();
      } catch (Exception e) {
        log.error("[TLDLParserComponent] exception deregistering dictionary", e);
      }
    }
    executor.shutdownNow();
  }

  public CachedParserComponent getCacheParser() {
    return cacheParser;
  }

  @TRADEService
  public void setHybridParserRestrictedMode(boolean enabled) {
    if (!useLLM) {
      log.warn("[setHybridParserRestrictedMode] Cannot toggle restricted mode without being configured to use the LLM");
      return;
    }

    if (enabled) {
      if (restrictedMode) {
        log.warn("[setHybridParserRestrictedMode] already enabled");
        return;
      }
      cachedSequence = sequence;
      // Put LLM at the back of the order
      String[] restrictedSequence = new String[sequence.length];
      restrictedSequence[sequence.length-1] = "llm";
      int offset = 0;
      for (int i=0;i<sequence.length;i++) {
        if (sequence[i].equals("llm")) {
          offset = 1;
          continue;
        }
        restrictedSequence[i-offset] = sequence[i];
      }
      sequence = restrictedSequence;
      restrictedMode = true;
    } else {
      if (!restrictedMode) {
        log.warn("[setHybridParserRestrictedMode] already disabled");
        return;
      } else if (cachedSequence == null) {
        log.error("[setHybridParserRestrictedMode] cannot find original cached sequence to reset to");
        return;
      }
      sequence = cachedSequence;
      cachedSequence = null;
      restrictedMode = false;
    }
  }

  @TRADEService
  public boolean getHybridParserRestrictedMode() {
    return restrictedMode;
  }
}
