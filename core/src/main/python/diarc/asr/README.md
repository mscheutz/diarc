
# TLDL Incremental Parser Test Overview

This document explains how the `TLDLIncrementalParserTest` class works for incremental parsing and interacting with the Python ASR (Automatic Speech Recognition) service using Vosk and TRADE.

## Key Components

- **TLDLParserComponent**: The core parsing component responsible for parsing the recognized speech input incrementally. It uses a chart-based approach to handle token-based parsing.

- **Python ASR Service**: A Python service using the Vosk model for speech recognition. The ASR service listens for speech, processes it, and sends the recognized text back to the Java component for incremental parsing.

- **TRADE**: A framework for managing interactions between Java and Python services. It allows for the mapping of recognized speech to actions via a defined action map.

## Initialization

The test class initializes the following components:
1. **TLDLParserComponent**: This component is responsible for parsing the recognized speech and updating the parse tree incrementally.
2. **Python ASR Service**: The Python service is started using the `PythonWrapper` to communicate with the speech recognition system.
3. **Background Threads**: Two background threads are started: one for fetching recognized text from the ASR service and the other for processing the recognized text in real-time.

```java
@Override
public void runConfiguration() {
    log.info("Running DIARC Configuration for Incremental Parsing Test");

    // Initialize TLDL Parser Component
    component = new TLDLParserComponent();
    component.addDictionary("incrementalTesting.dict");

    // Start Python ASR Service
    initializePythonWrapper();

    // Start a background thread for ASR recognition
    Thread asrThread = new Thread(this::fetchAsrText);
    asrThread.setDaemon(true); // Allows program to exit cleanly
    asrThread.start();

    // Start processing queue in a separate thread
    new Thread(this::processQueueContinuously).start();
}
```

## ASR Recognition

The `fetchAsrText` method listens for speech, recognizes the text using Vosk, and adds the recognized text to a queue for processing.

```java
private void fetchAsrText() {
    while (!Thread.currentThread().isInterrupted()) { // Allows graceful stopping
        log.info("🔊 START SPEAKING NOW"); // Prompt user

        try {
            String recognizedText = TRADE.getAvailableService(new TRADEServiceConstraints().name("startRecognition")).call(String.class);
            log.info("Raw TRADE response: {}", recognizedText);
            
            if (recognizedText != null && !recognizedText.isEmpty()) {
                log.info("Received ASR text from Python: '{}'", recognizedText);
                recognizedTextQueue.add(recognizedText);
            }
        } catch (TRADEException e) {
            log.error("Error retrieving recognized text from ASR", e);
            resetASRSystem();
        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            log.error("ASR listening thread interrupted", e);
            Thread.currentThread().interrupt(); // Allow graceful exit
        }
    }
}
```

## Processing Recognized Text

Once the recognized text is available in the queue, the `processQueueContinuously` method processes it incrementally. It performs the following:

- Cleans and filters the text by removing filler words.
- Passes the cleaned text to the `TLDLParserComponent` for parsing.
- Maps the parsed result to an action using a Python service.
- Handles "reset" and "exit" commands to reset the parser and terminate the program.

```java
private void processQueueContinuously() {
    StringBuilder incrementalPhrase = new StringBuilder();
    long lastInputTime = System.currentTimeMillis();

    while (!Thread.currentThread().isInterrupted()) {
        if (!recognizedTextQueue.isEmpty()) {
            String recognizedText = recognizedTextQueue.poll();
            lastInputTime = System.currentTimeMillis();

            if (recognizedText.equalsIgnoreCase("reset")) {
                log.info("🔄 Reset command detected! Clearing queue and resetting parser.");
                recognizedTextQueue.clear();
                component.resetChart();
                incrementalPhrase.setLength(0); 
                continue;
            }

            if (recognizedText.equalsIgnoreCase("exit") || recognizedText.equalsIgnoreCase("stop")) {
                log.info("🔴 Exit command detected! Terminating program.");
                recognizedTextQueue.clear();
                component.resetChart();
                incrementalPhrase.setLength(0); 
                System.exit(0);
            }

            if (recognizedText != null) {
                log.info("Processing new phrase: {}", recognizedText);
                processIncrementalParsing(recognizedText);
            }
        } else {
            long timeSinceLastInput = System.currentTimeMillis() - lastInputTime;
            if (timeSinceLastInput > 9000) {
                log.info("No new input detected. Performing final parse.");
                Utterance finalUtterance = createUtterance(incrementalPhrase.toString().trim());
                Utterance finalResult = component.parseUtterance(finalUtterance);
                log.info("Final result after reset: {}", finalResult);
                incrementalPhrase.setLength(0);
            }
            try {
                Thread.sleep(3000); // Wait before checking queue again
            } catch (InterruptedException e) {
                log.error("Processing thread interrupted", e);
                Thread.currentThread().interrupt();
            }
        }
    }
}
```

## Incremental Parsing Process

The `processIncrementalParsing` method processes each phrase by removing filler words and parsing the cleaned phrase. The result is then sent to Python for action mapping.

```java
private void processIncrementalParsing(String newPhrase) {
    log.info("Incrementally parsing new phrase: {}", newPhrase);

    String cleanedPhrase = Arrays.stream(newPhrase.split("\s+")).filter(word -> !FILLER_WORDS.contains(word.toLowerCase())).collect(Collectors.joining(" "));
    Utterance partialUtterance = createUtterance(cleanedPhrase);
    Utterance result = component.parseUtterance(partialUtterance);

    String parsedResult = result.toString();

    try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("logParse")).call(Void.class, parsedResult);
    } catch (TRADEException e) {
        log.error("Error sending parsed result to Python", e);
    }

    try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("mapParsedTextToAction")).call(String.class, parsedResult);
    } catch (TRADEException e) {
        log.error("Error mapping parsed result to action", e);
    }
}
```

## Example Usage

```bash
diarc USER$ ./gradlew launch -Pmain=edu.tufts.hrilab.config.parsing.TLDLIncrementalParserTest
```

Example output after speech recognition and parsing:

```
INFO  TLDLIncrementalParserTest -- 🔊 START SPEAKING NOW
Recognized Text (Python): grab the mug
INFO  TLDLIncrementalParserTest -- Processing new phrase: grab the mug
INFO  TLDLIncrementalParserTest -- Incrementally parsing new phrase: grab the mug
INFO  TLDLIncrementalParserTest -- [Python] Received Parse Result: INSTRUCT(speaker,listener,graspObject(listener,mug),{})
INFO  TLDLIncrementalParserTest -- [Python] Mapped action: GRASP(MUG_DEFAULT)
INFO  TLDLIncrementalParserTest -- Raw TRADE response: on the table
INFO  TLDLIncrementalParserTest -- Received ASR text from Python: 'on the table'
INFO  TLDLIncrementalParserTest -- 🔊 START SPEAKING NOW
Recognized Text (Python): on the table
INFO  TLDLIncrementalParserTest -- Processing new phrase: on the table
INFO  TLDLIncrementalParserTest -- Incrementally parsing new phrase: on the table
INFO  TLDLIncrementalParserTest -- [Python] Received Parse Result: INSTRUCT(speaker,listener,graspObject(listener,mug),{on(mug,table)})
INFO  TLDLIncrementalParserTest -- [Python] Mapped action: GRASP(MUG_ON_TABLE)
```

## Conclusion

The `TLDLIncrementalParserTest` class effectively combines incremental speech recognition and parsing. It processes speech in real-time, interacts with the Python ASR service, and maps parsed text to appropriate actions based on predefined patterns.
