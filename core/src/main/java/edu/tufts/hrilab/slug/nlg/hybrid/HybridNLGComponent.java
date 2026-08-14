/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.nlg.hybrid;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.nlg.cache.CachedNLGComponent;
import edu.tufts.hrilab.slug.nlg.llm.LLMNLGComponent;
import edu.tufts.hrilab.slug.nlg.templated.TemplatedNLG;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class HybridNLGComponent extends DiarcComponent {

  private boolean useLLM = false;
  private LLMNLGComponent llmGenerator;
  private boolean useSimpleNLG = false;
  private SimpleNLGComponent simpleNLG;
  private boolean useCache = false;
  private CachedNLGComponent cacheNLG;
  private boolean useTemplatedNLG = false;
  private TemplatedNLG templatedNLG;

  private String cacheName = null;
  private String[] cacheLoads = null;
  private boolean cachePersist = false;
  private boolean cacheSimpleNLG = false;
  // Adding configurable sequence to NLG as well. Mainly because "Social LLM" domains generally want LLM first.
  private String[] sequence;
  private boolean restrictedMode = false;
  private String[] cachedSequence = null;

  private ExecutorService executor = Executors.newCachedThreadPool();

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("simpleNLG").argName("boolean").desc("Use SimpleNLG as generation option").build());
    options.add(Option.builder("llm").argName("boolean").desc("Use the LLM generator").build());
    options.add(Option.builder("cache").argName("boolean").desc("Use the cache").build());
    options.add(Option.builder("cacheLoad").hasArgs().argName("file").desc("Load cached database from file, can have multiple").build());
    options.add(Option.builder("cacheName").hasArg().argName("string").desc("Name of cache").build());
    options.add(Option.builder("cachePersist").argName("file").desc("Persist cache in on users computer").build());
    options.add(Option.builder("cacheSimpleNLG").desc("Cache SimpleNLG responses").build());
    options.add(Option.builder("templated").argName("boolean").desc("Use the TemplatedNLG generator").build());
    options.add(Option.builder("sequence").hasArgs().argName("option1 option2 ...").desc("Sequence parsers (default cache, templated, simple, llm)").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("simpleNLG")) {
      useSimpleNLG = true;
    }
    if (cmdLine.hasOption("llm")) {
      useLLM = true;
      log.debug("Using LLM NLG");
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
      log.debug("Persist cache as a local file $HOME/.diarc/" + (cacheName != null ? cacheName : "diarc-parser-cache-reverse") + ".sqlite");
    }
    if (useCache && cmdLine.hasOption("cacheSimpleNLG")) {
      cacheSimpleNLG = true;
      log.debug("Cache SimpleNLG results");
    }
    if (cmdLine.hasOption("templated")) {
      useTemplatedNLG = true;
      log.debug("Using TemplatedNLG");
    }
    if (cmdLine.hasOption("sequence")) {
      sequence = cmdLine.getOptionValues("sequence");
    } else {
      sequence = new String[]{"cache", "templated", "simple", "llm"};
    }
    log.info("Using sequence: {}", Arrays.toString(sequence));
  }

  @Override
  public void init() {
    // init LLMNLGComponent
    if (useLLM) {
      llmGenerator = createInstance(LLMNLGComponent.class, "", false);
    }

    // init SimpleNLGComponent
    if (useSimpleNLG) {
      simpleNLG = createInstance(SimpleNLGComponent.class, "", false);
    }

    // init CachedNLGComponent
    if (useCache) {
      String cacheArgs = "reverse";
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
      cacheNLG = createInstance(CachedNLGComponent.class, cacheArgs, false);
    }

    //init TemplatedNLGComponent
    if (useTemplatedNLG) {
      templatedNLG = TemplatedNLG.defaults();
    }
  }

  // TODO: This new API currently assumes streaming TTS and filler phrases are only associated with the LLMParser (if a social subclass), could be generalized
  //    to any component(s) if that functionality is implemented. Also could generally change details about what components are triggered/checked synchronously vs async.
  // General convertSemanticsToText target - Either not tied to any produced speech output, or no streaming TTS and filler phrases are being utilized.
  @TRADEService
  @Action
  public Utterance convertSemanticsToText(Utterance incoming) {
    return convertSemanticsToText(incoming, false, false);
  }

  // TODO: Depending on desired implementation, this method can trigger the LLM component to start running asynchronously or just only run the non-llm components first. (Or make that behavior configurable)
  //  (The callStreaming argument is only actually necessary if the LLM request is made asynchronously from this method rather than synchronously from getLLMNLGResult)
  // Call convertSemanticsToText for any valid non-LLM output. This should be used when the result is tied to speech output and streaming TTS or filler utterances are associated.
  //  Need to run for non-llm result first to know if streaming TTS or filler phrases should be produced.
  @TRADEService
  public Utterance convertSemanticsToTextForNonLLMResult(Utterance incoming, boolean callStreaming) {
    return convertSemanticsToText(incoming, true, callStreaming);
  }

  // Associated with convertSemanticsToTextForNonLLMResult - Get Trailing LLM result after checking non-llm success
  @TRADEService
  public Utterance getLLMNLGResult(Utterance incoming, boolean callStreaming) {
    log.debug("[getLLMNLGResult] in method");

    // If starting the LLM asynchronously
    //*TODO: Would need to store futures and index on utteranceID to retrieve here(and probably move unblock/cancelTTSOutputForUID to this component and adjust/rename)

    // If only running the LLM synchronously
    Utterance output = llmGenerator.convertSemanticsToText(new Utterance.Builder(incoming).build(), callStreaming);

    if (output != null && useCache && !output.getWordsAsString().isEmpty()) {
      cacheNLGOutput(output);
    }
    return output;
  }


  private Utterance convertSemanticsToText(Utterance incoming, boolean returnNonLLM, boolean callStreamingMethod) {
    long t0 = System.nanoTime();
    log.debug("[lat] nlg_entry");
    log.debug("[convertSemanticsToText] hit with utterance: " + incoming);

    Future<Utterance> llmFuture = null;
    Future<Utterance> simpleNLGFuture = null;
    Future<Utterance> cachedFuture = null;
    Future<Utterance> templatedFuture = null;

    // If LLM spawned async
    //if (useLLM) {
    // If LLM synchronously only after other components fail
    if (useLLM && !returnNonLLM) {
      llmFuture = executor.submit(() -> llmGenerator.convertSemanticsToText(new Utterance.Builder(incoming).build(), callStreamingMethod));
    }
    if (useSimpleNLG) {
      simpleNLGFuture = executor.submit(() -> simpleNLG.convertSemanticsToText(new Utterance.Builder(incoming).build()));
    }
    if (useCache) {
      cachedFuture = executor.submit(() -> cacheNLG.convertSemanticsToText(new Utterance.Builder(incoming).build()));
    }
    if (useTemplatedNLG) {
      templatedFuture = executor.submit(() -> templatedNLG.convertSemanticsToText(new Utterance.Builder(incoming).build()));
    }

    Utterance output = null;
    Utterance cachedOutput = null;
    Utterance simpleNLGOutput = null;
    Utterance llmOutput = null;
    Utterance templatedOutput = null;

    for (int i = 0; i < sequence.length; i++) {
      switch (sequence[i]) {
        case "cache":
          if (cachedFuture != null) {
            try {
              cachedOutput = cachedFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[convertSemanticsToText] Error getting cached NLG results.", e);
            }

            if (cachedOutput != null && !cachedOutput.getWordsAsString().isEmpty()) {
              log.debug("[convertSemanticsToText] Using cached NLG response: " + cachedOutput);
              log.debug("[convertSemanticsToText] Using cached NLG words: " + cachedOutput.getWordsAsString());
              log.debug("[lat] nlg_done dur={}ms path=cache", (System.nanoTime() - t0) / 1_000_000);
              return cachedOutput;
            }
          }
          break;
        case "llm":
          if (!returnNonLLM && llmFuture != null) {
            try {
              llmOutput = llmFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[convertSemanticsToText] Error getting llm NLG results.", e);
            }

            if (llmOutput != null && !llmOutput.getWordsAsString().isEmpty()) {
              log.debug("[convertSemanticsToText] Using llm NLG response");
              output = llmOutput;
              if (useCache) {
                cacheNLGOutput(output);
              }
              log.debug("[lat] nlg_done dur={}ms path=llm", (System.nanoTime() - t0) / 1_000_000);
              return output;
            }
          }
          break;
        case "simple":
          if (simpleNLGFuture != null) {
            try {
              simpleNLGOutput = simpleNLGFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[convertSemanticsToText] Error getting SimpleNLG results.", e);
            }

            if (simpleNLGOutput != null && !simpleNLGOutput.getWordsAsString().isEmpty()) {
              log.debug("[convertSemanticsToText] Using SimpleNLG response");
              if (useCache && cacheSimpleNLG) {
                cacheNLGOutput(simpleNLGOutput);
              }
              log.debug("[lat] nlg_done dur={}ms path=simple", (System.nanoTime() - t0) / 1_000_000);
              return simpleNLGOutput;
            }
          }
          break;
        case "templated":
          if (templatedFuture != null) {
            try {
              templatedOutput = templatedFuture.get();
            } catch (InterruptedException | ExecutionException e) {
              log.error("[convertSemanticsToText] Error getting templated results.", e);
            }
          }

          if (templatedOutput != null && !templatedOutput.getWordsAsString().isEmpty()) {
            log.debug("[convertSemanticsToText] Using templated response");
            log.debug("[lat] nlg_done dur={}ms path=templated", (System.nanoTime() - t0) / 1_000_000);
            return templatedOutput;
          }
          break;
        default:
          break;
      }
    }

    log.debug("[lat] nlg_done dur={}ms path=fallthrough", (System.nanoTime() - t0) / 1_000_000);
    return output;
  }

  @TRADEService
  @Action
  public void cacheNLGOutput(Utterance utterance) {
    if (useCache) {
      log.debug("[cacheNLGOutput] Caching parsed utterance");
      cacheNLG.addEntryToCache(utterance);
    } else {
      log.debug("[cacheNLGOutput] Not caching parsed utterance");
    }
  }

  @Override
  protected void shutdownComponent() {
    super.shutdownComponent();
    executor.shutdownNow();
  }

  public CachedNLGComponent getCacheNLG() {
    return cacheNLG;
  }

  // Push LLM back in the sequence to prevent further error messages until reconnected
  @TRADEService
  public void setHybridNLGRestrictedMode(boolean enabled) {
    if (!useLLM) {
      log.warn("[setHybridNLGRestrictedMode] Cannot toggle restricted mode without being configured to use the LLM");
      return;
    }

    if (enabled) {
      if (restrictedMode) {
        log.warn("[setHybridNLGRestrictedMode] already enabled");
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
        log.warn("[setHybridNLGRestrictedMode] already disabled");
        return;
      } else if (cachedSequence == null) {
        log.error("[setHybridNLGRestrictedMode] cannot find original cached sequence to reset to");
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
