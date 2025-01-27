package edu.tufts.hrilab.slug.nlg.hybrid;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.interfaces.NLGInterface;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.nlg.cache.CachedNLGComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

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
    //TODO: LLM NLG?

    @Override
    public void init() {
        simpleNLGComponent = createInstance(SimpleNLGComponent.class, "-modifierService " + modifierService);
        cachedNLGComponent = createInstance(CachedNLGComponent.class, "-persist");
    }

    @Override
    public Utterance convertSemanticsToText(Utterance u) {

        Future<Utterance> cachedFuture = null;
        Future<Utterance> genFuture = null;

        genFuture = executor.submit(() -> simpleNLGComponent.convertSemanticsToText(u));
        cachedFuture = executor.submit(() -> cachedNLGComponent.convertSemanticsToText(u));

        Utterance cacheOutput = null;
        Utterance genOutput = null;

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
            genOutput = genFuture.get();
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
