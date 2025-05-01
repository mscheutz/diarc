package edu.tufts.hrilab.slug.nlg.hybrid;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.NLGInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.nlg.cache.CachedNLGComponent;
import edu.tufts.hrilab.slug.nlg.llm.LLMNLGComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class HybridNLGComponent extends DiarcComponent implements NLGInterface {

    private boolean useLLM = false;
    private NLGInterface llmGenerator;
    private boolean useSimpleNLG = false;
    private NLGInterface simpleNLG;
    private boolean useCache = false;
    private CachedNLGComponent cacheNLG;

    private String cacheName = null;
    private String[] cacheLoads = null;
    private boolean cachePersist = false;
    private boolean cacheSimpleNLG = false;

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
        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        if (cmdLine.hasOption("simpleNLG")) {
            useSimpleNLG = true;
        }
        if (cmdLine.hasOption("llm")) {
            useLLM = true;
            log.debug("Using LLM NLGr");
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
            log.debug("Persist cache as a local file $HOME/.diarc/" + (cacheName != null ? cacheName : "diarc-parser-cache-reverse") + ".sqlite");
        }
        if (useCache && cmdLine.hasOption("cacheSimpleNLG")) {
            cacheSimpleNLG = true;
            log.debug("Cache SimpleNLG results");
        }
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
          StringBuilder cacheArgs = new StringBuilder("reverse");
        if (cacheName != null) {
            cacheArgs.append(" -name ").append(cacheName);
        }
        if (cacheLoads != null) {
            for (String cacheLoad : cacheLoads) {
                cacheArgs.append(" -load ").append(cacheLoad);
            }
        }
        if (cachePersist) {
            cacheArgs.append(" -persist ");
        }
            cacheNLG = createInstance(CachedNLGComponent.class, cacheArgs.toString(), false);
        }
    }

    @Override
    public Utterance convertSemanticsToText(Utterance incoming) {
        log.debug("[convertSemanticsToText] hit with utterance: " + incoming);

        Future<Utterance> llmFuture = null;
        Future<Utterance> simpleNLGFuture = null;
        Future<Utterance> cachedFuture = null;

        if (useLLM) {
            llmFuture = executor.submit(() -> llmGenerator.convertSemanticsToText(new Utterance.Builder(incoming).build()));
        }
        if (useSimpleNLG) {
            simpleNLGFuture = executor.submit(() -> simpleNLG.convertSemanticsToText(new Utterance.Builder(incoming).build()));
        }
        if (useCache) {
            cachedFuture = executor.submit(() -> cacheNLG.convertSemanticsToText(new Utterance.Builder(incoming).build()));
        }

        Utterance output = null;
        Utterance cachedOutput = null;
        Utterance simpleNLGOutput = null;
        Utterance llmOutput = null;

        //Check cache first
        if (cachedFuture != null) {
        try {
                cachedOutput = cachedFuture.get();
        } catch (InterruptedException | ExecutionException e) {
                log.error("[convertSemanticsToText] Error getting cached NLG results.", e);
        }

            if (cachedOutput != null && !cachedOutput.getWordsAsString().isEmpty()) {
                log.debug("[convertSemanticsToText] Using cached NLG response: " + cachedOutput);
                log.debug("[convertSemanticsToText] Using cached NLG words: " + cachedOutput.getWordsAsString());
                return cachedOutput;
            }
        }

        //Try LLMParser, return if not empty
        if (llmFuture != null) {
        try {
                llmOutput = llmFuture.get();
            } catch (InterruptedException | ExecutionException e) {
                log.error("[convertSemanticsToText] Error getting llm NLG results.", e);
                }

            if (llmOutput != null && !llmOutput.getWordsAsString().isEmpty()){
                log.debug("[convertSemanticsToText] Using llm NLG response");
                output = llmOutput;
                if (useCache) {
                    cacheNLGOutput(output);
            }
                return output;
            }
        }

        //Try SimpleNLG last
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
                return simpleNLGOutput;
            }
        }

        return output;
    }

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
}
