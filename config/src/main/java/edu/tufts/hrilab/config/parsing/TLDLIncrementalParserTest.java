package edu.tufts.hrilab.config.parsing;

import java.util.Arrays;
import java.util.Collection;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.llm.LLMComponent;

public class TLDLIncrementalParserTest extends DiarcConfiguration {
    private TLDLParserComponent component;
    private LLMComponent LLMcomponent;
    private PythonWrapper wrapper;
    private PythonWrapper wrapper_planner;
    private final Queue<String> recognizedTextQueue = new ConcurrentLinkedQueue<>();
    private final AtomicBoolean isProcessing = new AtomicBoolean(false);

    private static final Set<String> FILLER_WORDS = new HashSet<>(Arrays.asList(
            "huh", "uh", "okay", "um", "wait", "actually"
    ));

    @Override
    public void runConfiguration() {
        log.info("Running DIARC Configuration for Incremental Parsing Test");

        // Initialize TLDL Parser Component
        component = new TLDLParserComponent();
        component.addDictionary("incrementalTesting.dict");

        // Initialize LLM Component
        //DiarcComponent.createInstance(edu.tufts.hrilab.llm.LLMComponent.class, "");
        LLMcomponent = new LLMComponent();


        // Start Python ASR Service
        initializePythonWrapper();

        // Start a background thread for ASR recognition
        Thread asrThread = new Thread(this::fetchAsrText);
        asrThread.setDaemon(true); // Allows program to exit cleanly
        asrThread.start();

        // Start processing queue in a separate thread
        new Thread(this::processQueueContinuously).start();
    }

    /**
     * Fetch ASR recognized text and add it to the queue for processing.
     * Runs in a separate background thread.
     */
    private void fetchAsrText() {
        while (!Thread.currentThread().isInterrupted()) { // Allows graceful stopping
            log.info("🔊 START SPEAKING NOW"); // Prompt user

            // Sleep briefly to prevent excessive polling
            try {
                Thread.sleep(8000);
            } catch (InterruptedException e) {
                log.error("ASR listening thread interrupted", e);
                Thread.currentThread().interrupt(); // Allow graceful exit
            }

            try {
                String recognizedText = TRADE.getAvailableService(new TRADEServiceConstraints().name("startRecognition")).call(String.class);

                log.info("Raw TRADE response: {}", recognizedText);

                if (recognizedText.equals("No speech recognized") || recognizedText.isEmpty()) {
                    log.warn("No recognized text received from ASR.");
                    continue;
                }
                else
                {
                  log.info("Received ASR text from Python: '{}'", recognizedText);
                  // Add recognized text to queue for processing
                  recognizedTextQueue.add(recognizedText);
                }

            } catch (TRADEException e) {
                log.error("Error retrieving recognized text from ASR", e);
                log.warn("Resetting ASR system to allow retry...");
                resetASRSystem();
            }

            // Sleep briefly to prevent excessive polling
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                log.error("ASR listening thread interrupted", e);
                Thread.currentThread().interrupt(); // Allow graceful exit
            }
        }
    }

    /**
     * Process phrases from the queue dynamically.
     * If no new input is received within 5 seconds, perform a final parse.
     */
    private void processQueueContinuously() {
        StringBuilder incrementalPhrase = new StringBuilder();
        long lastInputTime = System.currentTimeMillis();

        while (!Thread.currentThread().isInterrupted()) { // Allows graceful stopping
            if (!recognizedTextQueue.isEmpty()) {
                isProcessing.set(true);
                String recognizedText = recognizedTextQueue.poll(); // Fetch next phrase
                lastInputTime = System.currentTimeMillis();// Update last input time

                // ✅ Check if the spoken phrase is "reset"
                if (recognizedText.equalsIgnoreCase("reset")) {
                    log.info("🔄 Reset command detected! Clearing queue and resetting parser.");

                    recognizedTextQueue.clear(); // ✅ Clear all queued phrases
                    component.resetParser(); // ✅ Reset the parser component

                    incrementalPhrase.setLength(0); // ✅ Clear incremental phrase tracking

                    log.info("✅ Queue and parser have been reset.");
                    continue; // ✅ Skip further processing for this iteration
                }

                if (recognizedText.equalsIgnoreCase("exit") || recognizedText.equalsIgnoreCase("stop")) {
                    log.info("🔴 Exit command detected! Terminating program.");

                    recognizedTextQueue.clear(); // Clear all queued phrases
                    component.resetParser(); // Reset the parser component
                    incrementalPhrase.setLength(0); // Clear incremental phrase tracking

                    log.info("✅ Cleanup complete. Exiting...");

                    // Gracefully stop the program
                    System.exit(0); // Immediately terminates the Java process
                }

                if (recognizedText != null) {
                    //log.info("Processing new phrase: {}", recognizedText);

                    // ✅ Process only the new phrase (not everything before)
                    processIncrementalParsing(recognizedText);
                }
            } else {
                long timeSinceLastInput = System.currentTimeMillis() - lastInputTime;

                if (isProcessing.get() && timeSinceLastInput > 9000) { // If silent for 5+ sec, finalize parse
                    log.info("No new input detected. Performing final parse.");
                    //component.resetChart();
                    Utterance finalUtterance = createUtterance(incrementalPhrase.toString().trim());
                    Utterance finalResult = component.parseUtterance(finalUtterance);
                    log.info("Final result after reset: {}", finalResult);
                    incrementalPhrase.setLength(0); // Clear accumulated phrase
                }

                isProcessing.set(false);
                try {
                    Thread.sleep(50); // Wait before checking queue again
                } catch (InterruptedException e) {
                    log.error("Processing thread interrupted", e);
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    /**
     * Start the Python ASR service.
     */
    private void initializePythonWrapper() {
        String file = "diarc.asr.asr_service"; // Python module path
        wrapper = new PythonWrapper(file, true); // Start Python as a TRADE spoke
        wrapper.start();

        // String file_planner = "diarc.mcp_planner.experiment0";
        // String file_planner = "diarc.mcp_planner.experiment1";
        // String file_planner = "diarc.mcp_planner.experiment2";
        //String file_planner = "diarc.mcp_planner.experiment3";
       // wrapper_planner = new PythonWrapper(file_planner, true);
        //wrapper_planner.start();

        try {
            Thread.sleep(8000); // Allow time for PyTRADE to initialize
        } catch (InterruptedException e) {
            log.error("PythonWrapper initialization interrupted", e);
        }
    }

    private void resetASRSystem() {
        recognizedTextQueue.clear();  // ✅ Clear any pending recognized text
        component.resetParser();       // ✅ Reset parser component
        log.info("✅ ASR system reset. Please try speaking again.");
    }

    /**
     * Process a phrase incrementally.
     */
    private void processIncrementalParsing(String newPhrase) {
        log.info("Incrementally parsing new phrase: {}", newPhrase);

        // ✅ Filter out filler words
        String cleanedPhrase = Arrays.stream(newPhrase.split("\\s+")).filter(word -> !FILLER_WORDS.contains(word.toLowerCase())).collect(Collectors.joining(" "));

        // ✅ Only process the most recent phrase
        Utterance partialUtterance = createUtterance(cleanedPhrase);
        Utterance result = component.parsePartialUtterance(partialUtterance);
        String parsedResult = (result != null) ? result.toString() : "PARSE_FAILED";
        log.warn("Parsing result is null, using default: {}", parsedResult);

        // ✅ Send parse result to Python via TRADE
        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("logParse")).call(Void.class, parsedResult);
        } catch (TRADEException e) {
            log.error("Error sending parsed result to Python", e);
        }

        // ✅ Map parsed result to an action via Python
        String action = "NONE";
        try {
            action = TRADE.getAvailableService(new TRADEServiceConstraints().name("mapParsedTextToAction")).call(String.class, parsedResult);
        } catch (TRADEException e) {
            log.error("Error mapping parsed result to action", e);
        }

        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("send_to_planner")).call(String.class, action);
        } catch (TRADEException e) {
            log.error("Error sending instructions to planner", e);
        }
    }

    /**
     * Create an utterance object for parsing.
     */
    private Utterance createUtterance(String phrase) {
        return new Utterance.Builder()
                .setWords(Arrays.asList(phrase.split(" ")))
                .setSpeaker(Factory.createSymbol("speaker"))
                .setAddressee(Factory.createSymbol("listener"))
                .build();
    }
}
