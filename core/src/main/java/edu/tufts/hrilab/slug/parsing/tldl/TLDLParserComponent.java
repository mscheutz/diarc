/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.slug.parsing.tldl;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.interfaces.NLUInterface;
import edu.tufts.hrilab.slug.common.*;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.*;

import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import ai.thinkingrobots.trade.*;

public class TLDLParserComponent extends DiarcComponent implements NLUInterface {
  final protected Logger log = LoggerFactory.getLogger(this.getClass());

  /**
   * (speaker->most recent addressee)
   */
  private Map<Symbol, Symbol> addressHistory;
  /**
   * The addressee for the utterance being parsed. This is cleared and updated for every utterance.
   */
  private Symbol currentAddressee;
  /**
   * If true, use the addressHistory to keep track of most recent (speaker,addressee) pairs, in order to
   * track addressee across utterances that don't contain explicit direct address. This is true by default.
   */
  private boolean useAddressHistory = true;
  /**
   * Factory for creating unique variables.
   * NOTE: currently a singleton used across all speakers. Need instance per speaker to handle incremental parsing for more than one speaker
   */
  private VariableFactory variableFactory = VariableFactory.getInstance();

  static Dictionary dictionary;
  /**
   * List of .dict files currently in use by dictionary, including resource config path.
   */
  private List<String> dictionaryFiles;
  private String resourceConfigPath = "config/edu/tufts/hrilab/slug/parsing/tldl";
  //the set of CCG types which denote the end of an utterance, and their corresponding slug.common.UtteranceType
  private Map<String, UtteranceType> terminalCategories;

  //used for testing
  private List<Utterance> lastUtt;

  //used for testing
  public List<Node> nodeCollection = new LinkedList<>();

  //used for testing
  // Completed nodes (can inlude fragments and phrases) and full sentences
  public Map<Node, Utterance> completedNodes = new HashMap<>();

  //chart
  public List<Node>[][] chart;
  public boolean isChartInitialized = false;
  public int currentChartSize = 0;

  // ADDED FOR GUI
  private boolean usingGUI = false;
  private int guiWidth = 1265;
  private int guiHeight = 645;
  private String corpusFile;

  private List<Pair<String, String>> homophones = new ArrayList<>();

  public TLDLParserComponent() {
    super();
    addressHistory = new HashMap<>();
    dictionary = new Dictionary();
    try {
      TRADE.registerAllServices(dictionary, this.getMyGroups());
    } catch (TRADEException e) {
      log.error("[TLDLParserComponent] exception registering dictionary", e);
    }
    dictionaryFiles = new ArrayList<>();

    terminalCategories = new HashMap<>();
    terminalCategories.put("S", UtteranceType.STATEMENT);
    terminalCategories.put("Q", UtteranceType.QUESTION);
    terminalCategories.put("C", UtteranceType.INSTRUCT);
    terminalCategories.put("G", UtteranceType.GREETING);
    terminalCategories.put("ACK", UtteranceType.ACK);
    //TODO:brad:added this for question answering, we might want to consider a more generalized framework?
    terminalCategories.put("VAL", UtteranceType.REPLY);
  }

  // Reset the parsing chart
  public void resetChart() {
    chart = null;
    isChartInitialized = false;
    currentChartSize = 0;
    variableFactory.reset();
  }

  // Initialize the chart for parsing
  private void initializeChart(int initialSize) {
    currentChartSize = initialSize;
    chart = new ArrayList[initialSize][initialSize];
  }

  public boolean isChartInitialized() {
    return isChartInitialized;
  }

  public int getCurrentChartSize() {
    return currentChartSize;
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

  public Utterance buildPartialUtterance(Utterance input) {
    // Retrieve final parses from the chart
    List<Node> finalParses = chart[0][currentChartSize - 1];

    Symbol speaker = input.getSpeaker();
    Symbol addressee = input.getAddressee();

    Utterance endOfChartUtterance = null;

    // Only proceed if finalParses is non-empty
    if (finalParses != null && !finalParses.isEmpty()) {
      // The first parse in finalParses is considered "best"
      // but remember to check again to avoid IndexOutOfBounds
      Node bestParse = finalParses.get(0);
      endOfChartUtterance = createUtterance(bestParse, speaker, addressee);
    }

    if (endOfChartUtterance == null) {
      // Handle the case where finalParses is empty
      log.warn("[buildPartialUtterance] No valid parses found.");
//      endOfChartUtterance = generateErrorSemantics(speaker, addressee, input.getWordsAsString().toLowerCase());
//      endOfChartUtterance.setWords(input.getWords());
      return input;
    }

    // perform reasoning on multiple parses in debugUtterances here if needed
//    List<Utterance> candidateParses = getParsesFromChartCell(0, currentChartSize - 1, speaker, addressee);
//    List<Utterance> validParses = pruneParsesWithBelief(candidateParses);

// Finally, return only the best parse's utterance
    return endOfChartUtterance;

  }

  @Override
  public Utterance parseUtterance(Utterance input) {
    if (input == null || input.getWords() == null || input.getWords().isEmpty()) {
      log.warn("[parseUtterance] Null or empty input utterance.");
      return input;
    }

    // Check for end-of-utterance signal and reset if necessary
    // Example end-of-utterance condition or reasoning step//
    resetChart();

    // clear addressee from previous utterance
    currentAddressee = null;

    // Perform incremental parsing
    Utterance output = parseIncrementally(input);
//    Utterance output = parseUtteranceHelper(input);

    // use addressee history when there's no parse so the correct address generates the failure explanation
    if (output.getSemantics() == null && useAddressHistory && addressHistory.containsKey(input.getSpeaker())) {
      output = new Utterance.Builder(output).setAddressee(addressHistory.get(input.getSpeaker())).build();
    }
    log.info("[parseUtterance] Words: {} Result: {}", input.getWordsAsString(), output);
    return output;
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
  private int getChartIndex(List<String> utterance, List<String> words, int startIndex) {
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
   * Executes dynamic programming-based chart parsing on the provided input utterance.
   * This method tokenizes the input, initializes and updates the chart,
   * and combines spans using available parsing rules and syntax.
   *
   * @param input the utterance to be parsed, containing words for tokenization and chart creation
   */
  public void doChartParsingWithDP(Utterance input) {
    // TODO: EAK this tokenize method is a bit odd. It returns multi-word dictionary entries
    //       but ignores corresponding single word entries. Doesn't this ignore potentially valid parses?
    List<Pair<String, List<Entry>>> tokens = dictionary.tokenize(input.getWords());
    log.debug("[doChartParsingWithDP] Tokenized input: {}", tokens);

    // Initialize chart cells for the new tokens
    int startIndex = 0;
    for (int i = 0; i < tokens.size(); i++) {
      // get chart index and span for current token
      Pair<String, List<Entry>> token = tokens.get(i);

      List<Entry> entries = token.getRight();
      if (entries.isEmpty()) {
        log.warn("[doChartParsingWithDP] Unknown token: {}", token.getLeft());
        continue;
      }

      // get chart index and span for current token
      List<String> tokenWords = Arrays.stream(token.getLeft().split(" ")).toList();
      int chartIndex_1 = getChartIndex(input.getWords(), tokenWords, startIndex);
      int chartSpan = tokenWords.size();
      int chartIndex_2 = chartIndex_1 + chartSpan - 1;
      startIndex = chartIndex_2 + 1;

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
            if (left.rules.stream().anyMatch(r -> r.syntax.argType != null && r.syntax.argType.isLiteral)) {
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
    String literalTokens = utteranceWordsAsString.substring(utteranceWordsAsString.toLowerCase().indexOf(left.label.toLowerCase()) + left.label.length()).trim().toLowerCase();

    // find literal syntax rule
    NodeEntry rule = left.rules.stream().filter(r -> r.syntax.argType != null && r.syntax.argType.isLiteral).findFirst().orElse(null);

    Entry entry = generateLiteralEntry(literalTokens.toString(), rule.syntax);
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
    String argtype = syntax.argType.type;
    //String newType = argtype.substring(0,argtype.indexOf("*"));
    String newType = argtype;

    //TODO:brad: it might be nice to be able to provide semantic type info here some how at some point
    return new Entry(morpheme, new SyntacticRule(newType), SemanticRule.generateLiteralSemanticRule(morpheme, ""), "");
  }

  @Override
  protected void shutdownComponent() {
    deregisterDictionary();
  }

  public void deregisterDictionary() {
    try {
      TRADE.deregister(dictionary);
    } catch (TRADEException e) {
      log.error("[TLDLParserComponent] exception deregistering dictionary", e);
    }
  }

  @Override
  protected void init() {
    // load dictionary
    if (!dictionaryFiles.isEmpty()) {
      for (String dictionaryFile : dictionaryFiles) {
        dictionary.loadFile(dictionaryFile);
      }
    }

    // add homophones specified on command line
    for (Pair<String, String> homophone : homophones) {
      addHomophone(homophone.getLeft(), homophone.getRight());
    }
  }

  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("dict").hasArgs().argName("file").desc("load dictionary from file").build());
    options.add(Option.builder("homophone").hasArgs().numberOfArgs(2).argName("base homophone").desc("add base parse rule for homophone").build());
    options.add(Option.builder("noUpdateAddressee").desc("do not update addressee after direct address").build());

    return options;
  }

  /**
   * Prunes a list of candidate parses using Prolog-based belief reasoning.
   *
   * @param candidateParses List of possible Utterance interpretations.
   * @return Filtered list of valid Utterances after belief reasoning.
   */
  public List<Utterance> pruneParsesWithBelief(List<Utterance> candidateParses) {
    List<Utterance> filteredParses = new ArrayList<>();

    if (candidateParses.isEmpty()) {
      log.warn("[pruneParsesWithBelief] No candidate parses to evaluate.");
      return filteredParses;
    }

    log.debug("[pruneParsesWithBelief] Evaluating {} candidate parses.", candidateParses.size());

    // Query each parse individually
    for (Utterance parse : candidateParses) {
      Predicate query = generateQueryFromParse(parse);
      if (query == null) {
        log.warn("[pruneParsesWithBelief] Skipping null query for parse: {}", parse);
        continue;
      }

      List<Map<Variable, Symbol>> beliefResults = new ArrayList<>();

      // try {
      //   // Query the belief system with a single parse (using Predicate)
      //   beliefResults = TRADE.getAvailableService(
      //           new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)
      //   ).call(List.class, query);
      // } catch (TRADEException e) {
      //   log.warn("Could not query beliefs for parse: {}", parse, e);
      //   continue;
      // }

      // Apply reasoning to determine if the parse should be kept
      if (isValidParse(parse, beliefResults)) {
        filteredParses.add(parse);
      } else {
        log.debug("[pruneParsesWithBelief] Pruning parse: {}", parse);
      }
    }

    log.info("[pruneParsesWithBelief] Returning {} filtered parses.", filteredParses.size());
    return filteredParses;
  }

  /**
   * Converts an Utterance into a Prolog query Predicate.
   *
   * @param parse The Utterance object representing a parse.
   * @return A Predicate representing the Prolog query.
   */
  private Predicate generateQueryFromParse(Utterance parse) {
    if (parse == null || parse.getSemantics() == null) {
      return null;
    }
    // Construct a Prolog-style query as a Predicate
    return Factory.createPredicate("query_parse", "X", parse.getSemantics().toString());
  }

  /**
   * Determines if a parse is valid based on Prolog belief reasoning.
   *
   * @param parse         The candidate Utterance.
   * @param beliefResults The corresponding Prolog query results.
   * @return True if the parse is valid, False otherwise.
   */
  private boolean isValidParse(Utterance parse, List<Map<Variable, Symbol>> beliefResults) {
    if (beliefResults == null || beliefResults.isEmpty()) {
      return false; // No supporting belief means it's likely invalid.
    }

    // Example: Apply a custom filtering rule based on belief results.
    for (Map<Variable, Symbol> binding : beliefResults) {
      Symbol beliefAmount = binding.get(Factory.createVariable("beliefAmount"));
      Symbol worldAmount = binding.get(Factory.createVariable("worldAmount"));

      if (beliefAmount != null && worldAmount != null) {
        // Compute the belief difference
        Symbol difference = Factory.createSymbol(String.valueOf(
                Integer.parseInt(beliefAmount.getName()) - Integer.parseInt(worldAmount.getName())
        ));

        // Example: Keep parses where belief and world state are consistent
        if (Integer.parseInt(difference.getName()) >= 0) {
          return true;
        }
      }
    }

    return false;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("dict")) {
      Arrays.asList(cmdLine.getOptionValues("dict"))
              .stream().filter(dict -> !dict.isEmpty())
              .forEach(dict -> addDictionary(dict));
    }
    if (cmdLine.hasOption("noUpdateAddressee")) {
      useAddressHistory = false;
    }
    if (cmdLine.hasOption("homophone")) {
      String[] cliHomophones = cmdLine.getOptionValues("homophone");
      for (int i = 0; i < cliHomophones.length; i = i + 2) {
        this.homophones.add(Pair.of(cliHomophones[i], cliHomophones[i + 1]));
      }
    }
  }

  //replaces current dictionary with one found at path
  public void addDictionary(String newDict) {
    String newDictWithPath = Resources.createFilepath(resourceConfigPath, newDict);
    dictionaryFiles.add(newDictWithPath);
    log.info("adding dictionary: " + newDictWithPath);
    dictionary.loadFile(newDictWithPath);
  }

  /**
   * Replaces current Dictionary with new empty dictionary. Also replaces dictionaryFiles with an empty list/
   */
  public void clearDictionary() {
    dictionary = new Dictionary();
    try {
      TRADE.registerAllServices(dictionary, this.getMyGroups());
    } catch (TRADEException e) {
      log.error("[TLDLParserComponent] exception registering dictionary", e);
    }
    dictionaryFiles = new ArrayList<>();
  }

  // ADDED FOR GUI
  public List<String> getDictionaryFiles() {
    return dictionaryFiles;
  }

  //used to update word learning component, from 2019 ICRA paper
  @TRADEService
  public Pair<List<String>, List<String>> getLex() {
    return dictionary.getLex();
  }

  /**
   * Updates currentAddressee and addressee history (if enabled) based on utterance semantics.
   * Also removes direct address wrappers from generated semantics.
   *
   * @param semantics current utterance semantics
   * @param speaker   utterance speaker
   * @param addressee utterance addressee
   * @param type      utterance type
   * @return updated semantics with direct address wrapper removed
   */
  private Symbol updateDirectAddress(Symbol semantics, Symbol speaker, Symbol addressee, UtteranceType type) {
    Symbol returnSemantics = semantics;

    // try to extract direct address from semantics
    if (semantics.isTerm()) {
      if (semantics.getName().equals("directAddress")) {
        currentAddressee = ((Term) semantics).getArgs().get(0);
        returnSemantics = ((Term) semantics).get(1);
      } else if (type.equals(UtteranceType.INSTRUCT)
              && !((Term) semantics).getArgs().isEmpty()
              && !((Term) semantics).getArgs().get(0).getName().equals("?ADDRESSEE")) {
        currentAddressee = ((Term) semantics).getArgs().get(0);
      }
    }

    // if no direct address found in semantics, first try to use the addressee history, otherwise finally use the addressee from incoming Utterance
    if (currentAddressee == null) {
      if (useAddressHistory && addressHistory.containsKey(speaker)) {
        currentAddressee = addressHistory.get(speaker);
      } else {
        currentAddressee = addressee;
      }
    } else {
      if (useAddressHistory) {
        addressHistory.put(speaker, currentAddressee);
      }
    }
    return returnSemantics;
  }

  /**
   * replaces all instances of target in base with replacement
   */
  private Symbol updateSpeakerNamesHelper(Symbol base, Symbol target, Symbol replacement) {
    if (base.isTerm()) {
      List<Symbol> baseArgs = ((Term) base).getArgs();
      List<Symbol> newArgs = new ArrayList<>();
      for (Symbol arg : baseArgs) {
        newArgs.add(updateSpeakerNamesHelper(arg, target, replacement));
      }
      String baseName = base.getName();
      if (!base.getType().isEmpty()) {
        baseName += ":" + base.getType();
      }
      return Factory.createPredicate(baseName, newArgs);
    } else {
      if (base.getName().equals(target.getName())) {// && base.getType().equals(target.getType())){
        return replacement;
      } else {
        return base;
      }
    }
  }

  private Symbol updateSpeakerNames(Symbol semantics, Symbol addressee, Symbol speaker) {
    Symbol groundedSemantics = updateSpeakerNamesHelper(semantics, new Variable("?ADDRESSEE"), addressee);
    return updateSpeakerNamesHelper(groundedSemantics, new Variable("?INTERACTOR"), speaker);
  }

  //sends error message which indicates that the utterance wasn't parsed
  private Utterance generateErrorSemantics(Symbol speaker, Symbol addressee, String reason) {
    log.debug("sending error message");

    Utterance.Builder utterance = new Utterance.Builder();
    utterance.setSpeaker(speaker);
    if (useAddressHistory && addressHistory.containsKey(speaker)) {
      utterance.setAddressee(addressHistory.get(speaker));
    } else {
      utterance.setAddressee(addressee);
    }

    Predicate failureSemantics;
    if (reason.isEmpty()) {
      failureSemantics = Factory.createPredicate("not(understand(" + addressee + ",that))");
    } else {
      failureSemantics = Factory.createPredicate("error(" + addressee + ",doNotKnowWhat(" + addressee + "," + reason + ",means))");
    }
    utterance.setSemantics(failureSemantics);

    log.debug("utterance generated: " + utterance);
    return utterance.build();
  }

  /**
   * Constructs an Utterance object based on the provided finalNode's rules and given speaker
   * and addressee symbols. The method processes semantic and syntactic information from the node's
   * rules to generate a structured Utterance.
   *
   * @param finalNode The Node object containing the rules from which the utterance is generated.
   * @param speaker   The Symbol representing the speaker of the utterance.
   * @param addressee The Symbol representing the addressee of the utterance.
   * @return An Utterance object constructed based on the rules, semantics, and other properties of the node.
   * Returns null if no valid Utterance could be created from the node's rules.
   */
  public Utterance createUtterance(Node finalNode, Symbol speaker, Symbol addressee) {
    for (NodeEntry nodeRule : finalNode.rules) {

      UtteranceType utteranceType = terminalCategories.get(nodeRule.syntax.type);
      if (utteranceType == null) {
        log.warn("Skipping nodeRule with unknown terminal category: {}", nodeRule.syntax.type);
        continue;
      }

      Pair<List<Symbol>, Map<Variable, Symbol>> diarcSemantics = nodeRule.semantics.getDIARCSemantics();
      List<Symbol> fullSemantics = diarcSemantics.getLeft();
      log.debug("full semantics: " + fullSemantics);

      Symbol mainSemantics = fullSemantics.remove(0);
      mainSemantics = updateDirectAddress(mainSemantics, speaker, addressee, utteranceType);
      mainSemantics = updateSpeakerNames(mainSemantics, currentAddressee, speaker);
      List<Term> supplementalSemanticsTest = new ArrayList<>();
      for (Symbol semTest : fullSemantics) {
        if (semTest.isTerm()) {
          //TODO:brad: will we ever have ?ADDRESSEE or ?INTERACTOR in supplemental semantics?
          supplementalSemanticsTest.add((Term) updateSpeakerNames(semTest, currentAddressee, speaker));
        } else {
          log.warn("badly formatted supplemental semantics: " + semTest + " . Still added though");
          supplementalSemanticsTest.add((Term) semTest);
        }
      }
      log.debug("Bindings: " + diarcSemantics);

      Utterance utteranceTest = new Utterance.Builder()
              .setSpeaker(speaker)
              .setAddressee(currentAddressee)
              .setSemantics(mainSemantics)
              .setSupplementalSemantics(supplementalSemanticsTest)
              .setTierAssignments(diarcSemantics.getRight())
              .setIsInputUtterance(true)
              .setUtteranceType(utteranceType)
              .setWords(Arrays.asList(nodeRule.morpheme.split(" ")))
              .build();

      return utteranceTest;

    }
    return null;
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

                  // Create full utterance with completed node
                  Symbol sp = addressHistory.keySet().iterator().next();
                  Utterance nodeUtterance = createUtterance(combinedNodeCopy, input.getSpeaker(), input.getAddressee());
                  completedNodes.put(combinedNodeCopy, nodeUtterance);

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

  public void doChartParsingIncrementally(Utterance input) {
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
  public List<Utterance> getParsesFromChartCell(int start, int end, Symbol speaker, Symbol addressee) {
    List<Utterance> parses = new ArrayList<>();

    // Ensure the chart is initialized and the cell contains parses
    if (chart == null || start < 0 || end >= chart.length || chart[start][end] == null) {
      log.warn("[getParsesFromChartCell] Invalid chart cell ({}, {}).", start, end);
      return parses;
    }

    log.debug("[getParsesFromChartCell] Retrieving parses from cell ({}, {}).", start, end);

    for (Node node : chart[start][end]) {
      try {
        // Convert the node to an utterance
        Utterance utterance = createUtterance(node, speaker, addressee);
        if (utterance != null) {
          parses.add(utterance);
        }
      } catch (Exception e) {
        log.error("[getParsesFromChartCell] Error converting node to utterance.", e);
      }
    }

    log.debug("[getParsesFromChartCell] Found {} parses.", parses.size());
    return parses;
  }

  /**
   * Used in parser test to toggle updateAddressHistory
   *
   * @param val new value
   */
  protected void setUseAddressHistory(boolean val) {
    useAddressHistory = val;
  }

  /**
   * Used by the component test
   *
   * @return dictionary so that things like generateLocationRules can be tested in the parser test
   */
  protected Dictionary getDictionary() {
    return dictionary;
  }

  @TRADEService
  @Action
  public void injectDictionaryEntry(String morpheme, String type, String semantics, String cognitiveStatus) {
    //TODO: probably don't want this here permanently
    // EAK: causes issues if FirebaseComponent isn't fully initialized yet
//    if (TRADE.isAvailable("writeDictionaryKey")) {
//      try {
//        TRADE.callThe("writeDictionaryKey", morpheme);
//      } catch (TRADEException e) {
//        log.error("[injectDictionaryEntry] unable to write dictionary keys to firebase", e);
//      }
//    }
    morpheme = morpheme.toLowerCase();
    log.debug("[injectDictionaryEntry] injecting entry for: " + morpheme + " type: " + type + " semantics: " + semantics + " cs: " + cognitiveStatus);
    Entry e = new Entry(morpheme, new SyntacticRule(type), new SemanticRule(semantics), cognitiveStatus);
    log.debug("[injectDictionaryEntry] entry: " + e);
    dictionary.addEntry(e);
  }

  //TODO:brad: move these to Dictionary
  //Function used in FirebaseConnectionComponent to list dictionary keys on webapp for
  //  homophone definitions
  @TRADEService
  @Action
  public Set<String> getDictionaryEntries() {
    return dictionary.getKeySet();
  }

  @TRADEService
  public void addHomophone(String base, String homophone) {
    //TODO:brad:make this toggleable form the commandline
    addHomophone(base, homophone, false);
  }

  //Given a base morpheme and homophone, create and add all duplicate entries with homophone as new morpheme
  @TRADEService
  public void addHomophone(String base, String homophone, boolean writeToFile) {
    log.info("[addHomophone] have base: " + base + " and homophone: " + homophone);
    List<Entry> entries = dictionary.lookUpEntries(base);
    for (Entry entry : entries) {
      injectDictionaryEntry(homophone, entry.getSyntax(), entry.getSemantics().getSemantics(), entry.getCognitiveStatus());
      if (writeToFile) {
        //TODO: make output file configurable (through webapp)
        File tempASLFile = new File("resources/config/edu/tufts/hrilab/slug/parsing/tldl/assemblyHomophones.dict");
        try {
          tempASLFile.getParentFile().mkdirs();
          tempASLFile.createNewFile();
          FileOutputStream outputStream = new FileOutputStream(tempASLFile, true);
          byte[] strToBytes = ("\n%" + base + "\n" + homophone + "; " + entry.getSyntax() + "; " + entry.getSemantics().getDefinition()).getBytes();
          outputStream.write(strToBytes);
          outputStream.close();
        } catch (IOException e) {
          log.error("Unable to to modify homophone file: ", e);
        }
      }
    }
  }

}
