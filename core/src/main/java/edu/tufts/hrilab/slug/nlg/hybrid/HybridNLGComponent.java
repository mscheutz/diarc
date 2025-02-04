package edu.tufts.hrilab.slug.nlg.hybrid;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.NLGInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.nlg.cache.CachedNLGComponent;
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
    final protected Logger log = LoggerFactory.getLogger(this.getClass());

    private ExecutorService executor = Executors.newCachedThreadPool();

    //TODO: add parseArgs/additionalUsageInfo
    //  (and have convertSemanticsToText use configuration)
    private boolean useSimpleNLG = true;
    private NLGInterface simpleNLGComponent;
    private boolean useCache = true;
    private CachedNLGComponent cachedNLGComponent;
    private String modifierService = "spaceStationModifyNLG";
    private String cacheName = null;
    private String[] cacheLoads = null;
    private boolean cachePersist = false;
    //TODO: LLM NLG?

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("cacheLoad").hasArgs().argName("file").desc("Load cached database from file, can have multiple").build());
        options.add(Option.builder("cacheName").hasArg().argName("string").desc("Name of cache").build());
        options.add(Option.builder("cachePersist").argName("file").desc("Persist cache in on users computer").build());
        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        if (cmdLine.hasOption("cacheName")) {
            cacheName = cmdLine.getOptionValue("cacheName");
            log.debug("Naming cache " + cacheName);
        }
        if (cmdLine.hasOption("cacheLoad")) {
            cacheLoads = cmdLine.getOptionValues("cacheLoad");
            log.debug("Loading pre-existing cache" + (cacheLoads.length == 1 ? "" : "s") + ": " + String.join(", ", cacheLoads));
        }
        if (cmdLine.hasOption("cachePersist")) {
            cachePersist = true;
            log.debug("Persist cache as a local file $HOME/.diarc/" + (cacheName != null ? cacheName : "diarc-parser-cache") + ".sqlite");
        }
    }

    @Override
    public void init() {
        simpleNLGComponent = createInstance(SimpleNLGComponent.class, "-modifierService " + modifierService);

        StringBuilder cacheArgs = new StringBuilder();
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
        cachedNLGComponent = createInstance(CachedNLGComponent.class, cacheArgs.toString());
    }

    @Override
    public Utterance convertSemanticsToText(Utterance u) {

        Future<Utterance> cachedFuture = executor.submit(() -> cachedNLGComponent.convertSemanticsToText(u));
        Future<Utterance> genFuture = executor.submit(() -> simpleNLGComponent.convertSemanticsToText(u));

        Utterance cacheOutput = null;
        try {
            cacheOutput = cachedFuture.get();
        } catch (InterruptedException | ExecutionException e) {
            log.error("[convertSemanticsToText] error getting cached database result", e);
        }

        if (cacheOutput != null && !cacheOutput.getWordsAsString().isEmpty()) {
            log.debug("[convertSemanticsToText] using cached response: " + u.getWordsAsString());
            return cacheOutput;
        }

        try {
            Utterance genOutput = genFuture.get();
            if (genOutput != null) {
                String genWords = genOutput.getWordsAsString();
                if (!genWords.isEmpty()) {
                    cachedNLGComponent.addEntryToCache(genOutput);
                    log.debug("[convertSemanticsToText] using generated response: " + genWords);
                    return genOutput;
                }
            }
        } catch (InterruptedException | ExecutionException e) {
            log.error("[convertSemanticsToText] error getting generated result", e);
        }

        return u;
    }
}
