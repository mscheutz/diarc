package edu.tufts.hrilab.slug.parsing.tldl.chart;

import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.parsing.tldl.*;
import edu.tufts.hrilab.slug.parsing.tldl.parse.*;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ChartParser {
  private static final Logger log = org.slf4j.LoggerFactory.getLogger(ChartParser.class);
  private List<Node>[][] chart;
  private boolean isChartInitialized = false;
  private int currentChartSize = 0;
  private List<String> unknownTokens = new ArrayList<>();

  public void resetChart() {
    chart = null;
    isChartInitialized = false;
    currentChartSize = 0;
    unknownTokens.clear();
  }

  private void initializeChart(int initialSize) {
    currentChartSize = initialSize;
    chart = new ArrayList[initialSize][initialSize];
  }

  // Expand the chart for incremental parsing
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

  // Incremental parsing using the persistent chart
  public List<Node> parseIncrementally(Utterance input, Dictionary dictionary) {
    List<String> words = input.getWords();

    // where in the chart to start adding tokens. 0 = new utterance, N = incrementally starting at new incoming words
    int previousChartSize = 0;

    // Initialize or expand the chart as needed
    if (!isChartInitialized) {
      initializeChart(words.size());
      isChartInitialized = true;
    } else {
      previousChartSize = currentChartSize;
      expandChart(words.size());
    }

    log.debug("Chart size after expansion: {}", currentChartSize);

    // Use the incremental chart parsing logic
    doChartParsingWithDP(input, dictionary, previousChartSize);

    // Retrieve final parses from the chart
    List<Node> finalParses = chart[0][currentChartSize - 1];
    return finalParses;
  }

  /**
   * Executes dynamic programming-based chart parsing on the provided input utterance.
   * This method tokenizes the input, initializes and updates the chart,
   * and combines spans using available parsing rules and syntax.
   *
   * @param input the utterance to be parsed, containing words for tokenization and chart creation
   */
  public void doChartParsingWithDP(Utterance input, Dictionary dictionary, int previousChartSize) {
    // TODO: EAK this tokenize method is a bit odd. It returns multi-word dictionary entries
    //       but ignores corresponding single word entries. Doesn't this ignore potentially valid parses?
    List<Pair<String, List<Entry>>> tokens = dictionary.tokenize(input.getWords());
    log.debug("[doChartParsingWithDP] Tokenized input: {}", tokens);

    // Initialize chart cells for the new tokens
    int startTokenIndex = 0;
    for (int i = 0; i < tokens.size(); i++) {
      // get chart index and span for current token
      Pair<String, List<Entry>> token = tokens.get(i);

      List<Entry> entries = token.getRight();
      if (entries.isEmpty()) {
        log.warn("[doChartParsingWithDP] Unknown token: {}", token.getLeft());
        unknownTokens.add(token.getLeft());
        continue;
      }

      // get chart index and span for current token
      List<String> tokenWords = Arrays.stream(token.getLeft().split(" ")).toList();
      int utteranceIndex = getUtterancIndex(input.getWords(), tokenWords, startTokenIndex);
      int chartIndex_1 = utteranceIndex + previousChartSize;
      int chartSpan = tokenWords.size();
      int chartIndex_2 = chartIndex_1 + chartSpan - 1;
      startTokenIndex = utteranceIndex + chartSpan;

      Node node = new Node(entries);
      if (chart[chartIndex_1][chartIndex_2] == null) {
        chart[chartIndex_1][chartIndex_2] = new ArrayList<>();
      }

      // **Check for duplicates before adding**
      if (!chart[chartIndex_1][chartIndex_2].contains(node)) {
        chart[chartIndex_1][chartIndex_2].add(node);
      } else {
        log.debug("[doChartParsingWithDP] Duplicate node detected, skipping: {}", node.getLabel());
      }
    }

    // Combine spans across the full chart (including old and new nodes)
    for (int span = 2; span <= currentChartSize; span++) {
      for (int start = 0; start <= currentChartSize - span; start++) {
        int end = start + span - 1;

        for (int split = start; split < end; split++) {
          List<Node> leftNodes = chart[start][split];
          if (leftNodes == null) {
            continue;
          }

          for (Node left : leftNodes) {
            // if Node has rule that has literal syntax argType, it can accept anything on the right side
            // create new Node with all words on right side and add to the chart
            if (left.getRules().stream().anyMatch(r -> r.getSyntacticRule().getArgType() != null && r.getSyntacticRule().getArgType().isLiteral())) {
              addNewLiteralNode(input.getWordsAsString(), left, split + 1);
            }

            List<Node> rightNodes = chart[split + 1][end];
            if (rightNodes != null) {

              for (Node right : rightNodes) {
                Node combinedNode = left.combine(right);
                if (combinedNode != null) {
                  if (chart[start][end] == null) {
                    chart[start][end] = new ArrayList<>();
                  }

                  // Check for duplicates before adding
                  if (!chart[start][end].contains(combinedNode)) {
                    chart[start][end].add(combinedNode);
                  } else {
                    log.debug("[doChartParsingWithDP] Duplicate combined node detected, skipping: {}", combinedNode.getLabel());
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  /**
   * Retrieves the starting index of a substring (sequence of words) within a given list of strings,
   * if it exists. Compares words in a case-insensitive manner. If the substring is not found,
   * returns -1.
   *
   * @param utterance The list of strings representing the main text to search in.
   * @param words     The list of strings representing the sequence of words to find.
   * @return The starting index in the utterance where the sequence begins, or -1 if no match is found.
   */
  private int getUtterancIndex(List<String> utterance, List<String> words, int startIndex) {
    for (int i = startIndex; i <= utterance.size() - words.size(); i++) {
      if (utterance.get(i).equalsIgnoreCase(words.get(0))) {
        // found first word match --> check the rest of the phrase
        for (int j = 1; j < words.size(); j++) {
          if (!utterance.get(i + j).equalsIgnoreCase(words.get(j))) {
            break;
          }
        }

        // if didn't break from for-loop above, a match is found
        // NOTE: this is always the first match. Should we find ALL matches?
        return i;
      }
    }

    log.error("[getChartIndex] No match found for word: {}", words);
    return -1;
  }

  /**
   * Adds a new literal node to the parsing chart based on the provided utterance and left node.
   *
   * @param utteranceWordsAsString the full input sentence as a string
   * @param left                   the left Node containing a rule with a literal syntax argument type
   * @param chartIndex             the index in the chart where the new node should be added
   */
  private void addNewLiteralNode(String utteranceWordsAsString, Node left, int chartIndex) {
    // if Node has rule that has literal syntax argType, it can accept anything on the right side
    // create new Node with all words on right side and add to the chart
    log.debug("[doChartParsingWithDP] Literal syntax detected, creating new LIT node");
    String literalTokens = utteranceWordsAsString.substring(utteranceWordsAsString.toLowerCase().indexOf(left.getLabel().toLowerCase()) + left.getLabel().length()).trim().toLowerCase();

    // find literal syntax rule
    NodeEntry rule = left.getRules().stream().filter(r -> r.getSyntacticRule().getArgType() != null && r.getSyntacticRule().getArgType().isLiteral()).findFirst().orElse(null);

    Entry entry = generateLiteralEntry(literalTokens.toString(), rule.getSyntacticRule());
    Node literalNode = new Node(Collections.singletonList(entry));

    if (chart[chartIndex][currentChartSize - 1] == null) {
      chart[chartIndex][currentChartSize - 1] = new ArrayList<>();
    }
    chart[chartIndex][currentChartSize - 1].add(literalNode);
  }

  /**
   * Generates a literal entry based on the provided morpheme and syntactic rule.
   * This method creates an entry by utilizing the input morpheme,
   * updating the syntactic rule type, and generating a default literal semantic rule.
   *
   * @param morpheme the input string representing the morpheme for the entry
   * @param syntax   the syntactic rule associated with the morpheme
   * @return a new Entry object with the provided morpheme, updated syntactic rule,
   * default literal semantic rule, and an empty string for additional information
   */
  private Entry generateLiteralEntry(String morpheme, SyntacticRule syntax) {
    log.debug("generating wild card entry for: " + morpheme);
    //For now the convention is to have the syntatic type before the star
    String argtype = syntax.getArgType().getType();
    //String newType = argtype.substring(0,argtype.indexOf("*"));
    String newType = argtype;

    //TODO:brad: it might be nice to be able to provide semantic type info here some how at some point
    return new Entry(morpheme, new SyntacticRule(newType), SemanticRule.generateLiteralSemanticRule(morpheme, ""), "");
  }

  private void addNodeToChart(Node newNode, int position, Utterance input) {
    log.debug("[addNodeToChart] Adding new node at position {}: {}", position, newNode);

    // Initialize the chart cell for the single token
    if (chart[position][position] == null) {
      chart[position][position] = new ArrayList<>();
    }
    chart[position][position].add(newNode);

    // Expand the chart dynamically by combining the new node with existing nodes
    for (int length = 1; length <= position + 1; length++) {
      for (int start = position - length + 1; start >= 0; start--) {
        int end = start + length - 1;

        if (end != position) {
          continue; // Skip ranges that don't include the new node
        }

        // Iterate over possible split points
        for (int split = start; split < end; split++) {
          List<Node> leftNodes = chart[start][split];
          List<Node> rightNodes = chart[split + 1][end];

          if (leftNodes != null && rightNodes != null) {
            for (Node left : leftNodes) {
              for (Node right : rightNodes) {
                // Attempt to combine nodes
                Node combinedNode = left.combine(right);

                // Check if there are valid combinations
                if (combinedNode != null) {
                  //masterNodeEntry.add(validCombinations);

                  // Need to catch node combination errors
                  log.debug("[addNodeToChart] Combining nodes: {} and {}", left, right);
                  // Potential spot to create utterance with node and send down the pipeline
                  // Make a decision to reset chart to continue parsing
                  Node combinedNodeCopy = null;
                  try {
                    combinedNodeCopy = combinedNode.clone();
                  } catch (CloneNotSupportedException e) {
                    log.error("Could not clone combined Node.", e);
                    continue;
                  }

                  // Add the combined node to the chart
                  if (chart[start][end] == null) {
                    chart[start][end] = new ArrayList<>();
                  }
                  chart[start][end].add(combinedNode);
                  log.debug("[addNodeToChart] Combined node added to chart: {}", combinedNode);
                } else {
                  log.debug("[addNodeToChart] No valid combinations found for nodes: {} and {}", left, right);
                }
              }
            }
          }
        }
      }
    }
  }

  // TODO: delete this?
  public void doChartParsingIncrementally(Utterance input, Dictionary dictionary) {
    List<Utterance> completeUtteranceNodes = new ArrayList<>();
    List<Pair<String, List<Entry>>> tokens = dictionary.tokenize(input.getWords());
    log.debug("[doChartParsingIncrementally] Tokenized input: {}", tokens);

    // Initialize the chart for the first token
    int tokenCount = tokens.size();
    chart = new ArrayList[tokenCount][tokenCount];

    // Process each token incrementally
    for (int position = 0; position < tokens.size(); position++) {
      String token = tokens.get(position).getLeft();
      List<Entry> entries = tokens.get(position).getRight();

      if (entries.isEmpty()) {
        log.warn("[doChartParsingIncrementally] Unknown token: {}", token);
        continue;
      }

      // Create a new node for the token and add it incrementally to the chart
      Node newNode = new Node(entries);
      log.debug("[doChartParsingIncrementally] Created new node: {}", newNode);

      addNodeToChart(newNode, position, input);

    }

    // Retrieve final parses
    List<Node> finalParses = chart[0][tokenCount - 1];
    if (finalParses == null || finalParses.isEmpty()) {
      log.warn("[doChartParsingIncrementally] No valid parses found.");
    } else {
      log.info("[doChartParsingIncrementally] Found {} complete parses.", finalParses.size());
    }
  }

  /**
   * Retrieves all valid parses from a specific cell in the chart.
   *
   * @param start The start index of the chart cell.
   * @param end   The end index of the chart cell.
   * @return List of Utterance objects representing possible parses.
   */
  public List<Node> getParsesFromChartCell(int start, int end) {
    List<Node> parses = new ArrayList<>();

    // Ensure the chart is initialized and the cell contains parses
    if (chart == null || start < 0 || end >= chart.length || chart[start][end] == null) {
      log.warn("[getParsesFromChartCell] Invalid chart cell ({}, {}).", start, end);
      return parses;
    }

    log.debug("[getParsesFromChartCell] Retrieving parses from cell ({}, {}).", start, end);
    chart[start][end].forEach(parse -> parses.add(parse));
    log.debug("[getParsesFromChartCell] Found {} parses.", parses.size());
    return parses;
  }

  public boolean hasUnkownTokens() {
    return !unknownTokens.isEmpty();
  }

  public String getFirstUnknownTokens() {
    if (unknownTokens.isEmpty()) {
      return "";
    }

    return unknownTokens.get(0);
  }

}
