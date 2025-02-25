
# TLDL Incremental Parsing Overview

This document provides an overview of how the `TLDLParserComponent` class implements incremental parsing using a dynamic chart. The incremental parsing process is designed to process input one token at a time, updating a parse tree as more tokens are added.

## Key Components

- **Address History:** Keeps track of the most recent speaker-addressee pairs for context-based parsing. This is important for managing address semantics and resolving ambiguities.

- **Chart:** A 2D array (or "chart") that stores possible parse nodes for every span of words in an utterance. This chart allows the parser to expand incrementally as new words are added.

- **Terminal Categories:** Different types of utterances (e.g., `STATEMENT`, `QUESTION`, `INSTRUCT`) are tracked through the `terminalCategories` map. These are used to identify the type of utterance as it's parsed.

## Initialization

Upon receiving an input utterance, the chart is initialized (if it hasn't been set already) to match the number of tokens in the utterance. If the chart has already been initialized, it is expanded to accommodate new tokens.

```java
private void initializeChart(int initialSize) {
    currentChartSize = initialSize;
    chart = new ArrayList[initialSize][initialSize];
}
```

## Chart Expansion

The chart is designed to grow incrementally as new words are processed. When new tokens are added to the utterance, the chart is expanded by increasing its size and copying over the previous chart's entries.

```java
private void expandChart(int newWords) {
    int newSize = currentChartSize + newWords;
    List<Node>[][] newChart = new ArrayList[newSize][newSize];

    // Copy existing entries into the new chart
    for (int i = 0; i < currentChartSize; i++) {
        for (int j = i; j < currentChartSize; j++) {
            newChart[i][j] = chart[i][j];
        }
    }

    // Initialize the new rows and columns for the expanded chart
    for (int i = currentChartSize; i < newSize; i++) {
        for (int j = 0; j < newSize; j++) {
            newChart[i][j] = new ArrayList<>();
        }
    }

    chart = newChart;
    currentChartSize = newSize;
}
```

## Incremental Parsing Process

The `parseIncrementally` method handles the parsing of each token in the utterance. For each token, it updates the chart with new nodes representing the syntactic structure of the sentence.

```java
public Utterance parseIncrementally(Utterance input) {
    List<String> words = input.getWords();

    // Initialize or expand the chart as needed
    if (!isChartInitialized) {
        initializeChart(words.size());
        isChartInitialized = true;
    } else {
        expandChart(words.size());
    }

    log.debug("Chart size after expansion: {}", currentChartSize);

    // Use the incremental chart parsing logic
    doChartParsingWithDP(input);

    // Build a partially parsed Utterance based on the current state of the chart
    Utterance partialUtterance = buildPartialUtterance(input);

    log.debug("Partial Utterance after parsing: {}", partialUtterance);

    return partialUtterance;
}
```

## Tokenization and Parsing

Tokens are generated from the input utterance using the dictionary's `tokenize` method. Each token is parsed by generating nodes that represent syntactic structures.

```java
public void doChartParsingWithDP(Utterance input) {
    List<Pair<String, List<Entry>>> tokens = dictionary.tokenize(input.getWords());
    log.debug("[doChartParsingWithDP] Tokenized input: {}", tokens);

    int startOffset = currentChartSize - tokens.size();
    int tokenCount = tokens.size();

    // Initialize chart cells for the new tokens
    for (int i = 0; i < tokenCount; i++) {
        int chartIndex = startOffset + i;
        List<Entry> entries = tokens.get(i).getRight();

        if (entries.isEmpty()) {
            log.warn("[doChartParsingWithDP] Unknown token: {}", tokens.get(i).getLeft());
            continue;
        }

        Node node = new Node(entries);
        if (chart[chartIndex][chartIndex] == null) {
            chart[chartIndex][chartIndex] = new ArrayList<>();
        }

        // **Check for duplicates before adding**
        if (!chart[chartIndex][chartIndex].contains(node)) {
            chart[chartIndex][chartIndex].add(node);
        } else {
            log.debug("[doChartParsingWithDP] Duplicate node detected, skipping: {}", node.getLabel());
        }
    }
}
```

## Handling Multiple Parses

Once the chart is updated, the method `buildPartialUtterance` retrieves the final parse(s) from the chart and constructs an utterance based on the best parse.

```java
public Utterance buildPartialUtterance(Utterance input) {
    List<Node> finalParses = chart[0][currentChartSize - 1];
    if (finalParses == null || finalParses.isEmpty()) {
        log.warn("[buildPartialUtterance] No valid parses found.");
        return input;
    }

    // Iterate through all parses to create debug utterances
    List<Utterance> debugUtterances = new ArrayList<>();

    Symbol speaker = input.getSpeaker();
    Symbol addressee = input.getAddressee();

    Utterance endOfChartUtterance = null;

    // Only proceed if finalParses is non-empty
    if (!finalParses.isEmpty()) {
        for (int i = 0; i < finalParses.size(); i++) {
            Node currentParse = finalParses.get(i);
            Utterance debugUtterance = createUtterance(currentParse, speaker, addressee);
            debugUtterances.add(debugUtterance);
        }

        Node bestParse = finalParses.get(0);
        endOfChartUtterance = createUtterance(bestParse, speaker, addressee);
    }

    return endOfChartUtterance;
}
```

## Conclusion

The `TLDLParserComponent` uses an efficient chart parsing mechanism to incrementally parse input utterances. This allows for real-time updates and flexibility in parsing. The system processes tokens step by step, updating its internal state, and handles ambiguities by storing multiple potential parses in the chart until a valid parse emerges.

