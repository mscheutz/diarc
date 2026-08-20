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
   * (speaker -> current parse if it exists), only exists in cases of incremental input
   */
  private Map<Symbol, Parse> parses;

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
    parses = new HashMap<>();
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
    if (finalParses == null || finalParses.isEmpty()) {
      log.warn("[buildPartialUtterance] No valid parses found.");
      return input;
    }

    Symbol speaker = input.getSpeaker();
    Symbol addressee = input.getAddressee();

    Utterance endOfChartUtterance = null;

    // Only proceed if finalParses is non-empty
    if (!finalParses.isEmpty()) {
      // The first parse in finalParses is considered "best"
      // but remember to check again to avoid IndexOutOfBounds
      Node bestParse = finalParses.get(0);
      endOfChartUtterance = createUtterance(bestParse, speaker, addressee);
    } else {
      // Handle the case where finalParses is empty, if needed
      // e.g., set endOfChartUtterance = null or throw an exception
      // endOfChartUtterance = null;
    }

     // perform reasoning on multiple parses in debugUtterances here if needed
//    List<Utterance> candidateParses = getParsesFromChartCell(0, currentChartSize - 1, speaker, addressee);
//    List<Utterance> validParses = pruneParsesWithBelief(candidateParses);

// Finally, return only the best parse's utterance
    return endOfChartUtterance;

  }

  @Override
  public Utterance parseUtterance(Utterance input) {
    // Check for end-of-utterance signal and reset if necessary
    // Example end-of-utterance condition or reasoning step//
     resetChart();

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

    // Combine spans across the full chart (including old and new nodes)
    for (int span = 2; span <= currentChartSize; span++) {
      for (int start = 0; start <= currentChartSize - span; start++) {
        int end = start + span - 1;

        if (chart[start][end] == null) {
          chart[start][end] = new ArrayList<>();
        }

        for (int split = start; split < end; split++) {
          List<Node> leftNodes = chart[start][split];
          List<Node> rightNodes = chart[split + 1][end];

          if (leftNodes != null && rightNodes != null) {
            for (Node left : leftNodes) {
              for (Node right : rightNodes) {
                try {
                  List<NodeEntry> validCombinations = left.combineTest(right);
                  if (!validCombinations.isEmpty()) {
                    Node combinedNode = new Node(left, right, validCombinations);

                    // **Check for duplicates before adding**
                    if (!chart[start][end].contains(combinedNode)) {
                      chart[start][end].add(combinedNode);
                    } else {
                      log.debug("[doChartParsingWithDP] Duplicate combined node detected, skipping: {}", combinedNode.getLabel());
                    }
                  }
                } catch (Exception e) {
                  // Do nothing - silently catch the exception);
                }
              }
            }
          }
        }
      }
    }
  }

  @Override
  protected void shutdownComponent() {
    deregisterDictionary();
    parses.clear();
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
   * @param parse The candidate Utterance.
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
   * @param speaker utterance speaker
   * @param addressee utterance addressee
   * @param type utterance type
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

  /**
   * First step of Utterance Processing, updates address history and passes utterance to
   * addWords. This allows for parallelization across incoming utterances, while still preserving
   * incremental processing to some degree.
   * <p>
   * Tokenizes the text of the utterance and updates the parse with each token in order.
   *
   * @param input utterance to be parsed
   * @return parsed Utterance (or Utterance with failure semantics)
   */
  private Utterance parseUtteranceHelper(Utterance input) {
    // clear addressee from previous utterance
    currentAddressee = null;

    Symbol speaker = input.getSpeaker();
    Symbol addressee = input.getAddressee();
    if (addressee == null) {
      log.warn("addressee not set in incoming utterance. Setting to \"unknown\" unless direct address overrides this value.");
    }

    //Make sure there is a Parse object for the speaker of the current Utterance
    if (!parses.containsKey(speaker)) {
      parses.put(speaker, new Parse(terminalCategories.keySet()));
    }
    Parse currentParse = parses.get(speaker);

    List<Pair<String, List<Entry>>> tokens = dictionary.tokenize(input.getWords());

    Utterance output;
    for (ListIterator<Pair<String, List<Entry>>> tokensIterator = tokens.listIterator(); tokensIterator.hasNext(); ) {
      Pair<String, List<Entry>> nextToken = tokensIterator.next();
      String token = nextToken.getLeft();
      List<Entry> entries = nextToken.getRight();
      if (entries.isEmpty()) {
        log.warn("[parseUtterance] unknown token: " + token);
        log.debug("current interactor: " + speaker);
        parses.put(speaker, new Parse(terminalCategories.keySet()));
        // TODO: this should be handled in the NLU scripts

        output = generateErrorSemantics(speaker, addressee, token);
        output.setWords(input.getWords());
        return output;
      }
      currentParse = addEntriesForToken(entries, speaker);
      //this occurs if we're in not in learning mode and we get a morpheme that's not in the dict
      if (currentParse == null) {
        parses.put(speaker, new Parse(terminalCategories.keySet()));
        log.debug("parse tree reset");
        return input;
      }

      if (currentParse.isLiteral()) {
        StringBuilder newToken = new StringBuilder();
        while (tokensIterator.hasNext()) {
          newToken.append(tokensIterator.next().getLeft()).append(" ");
        }
        //convert all of the rest of they keys into a new token
        //TODO:brad check if this is empty
        if (!newToken.toString().isEmpty()) {
          Entry e = currentParse.generateLiteralEntry(newToken.toString());
          //add the new entry to the parse
          addWord(e, speaker);
          //we're done and the utterance is complete
          break;
        } else {
          log.warn("expecting literal args at the end of utterance, getting ignored: " + token);
        }
      }

      //TODO:brad: do we actually want this behavior?
      //utterance ends in the middle of tokens
      //if the utterance ends in the middle of tokens, we want to send it on the complete utterance,
      // but also keep the remainder of tokens
//      if (currentParse.verifyParseCompletion()) {
////        log.debug("Addition of \"" + token + "\" verifies the previous utterance was over");
//        Node previousParse = currentParse.popPreviousParse();
//        log.debug("previous parse: " + previousParse);
//        sendParse(previousParse, currentInteractor,currentListener);
//
//        //reset parse tree once utterance has been set, but re add the word we just added
//        parses.put(u.getSpeaker(), new Parse(terminalCategories.keySet()));
//        addEntriesForToken(entries, currentInteractor);
//      }
    }

    //Once we've heard all of the words in the utterance:
    //if the current parse is a terminal case
    if (currentParse.checkTerminal()) {
      //currentParse.pruneNonTerminal();
      log.debug("terminal condition found");
      output = createParsedUtterance(currentParse.getCurrentParse(), speaker, addressee);

      //reset parse tree once utterance has been sent
      //TODO: in the future we might want to keep track of the set of all previous parses somewhere
      parses.put(input.getSpeaker(), new Parse(terminalCategories.keySet()));
    }
    else {
      //otherwise check to see if we can make it a terminal case...
      log.debug("no valid parse and no more input in time.");
      log.debug("current parse: " + currentParse);
      //if we don't have parse rules for a word try to figure them out
      //TODO:brad: make a separate pipeline that results in in infer unknowns getting called.
      if (!currentParse.checkAllDefinitionsExist() && false) {
        log.debug("inferring new definitions ");
        dictionary.addEntries(currentParse.inferUnknowns());
        log.debug("sending inferred parse");
        output = createParsedUtterance(currentParse.getCurrentParse(), speaker, addressee);
      } else if (currentParse.checkComplete()) {
        //try to rebuild the tree because the current parsing algorithm is too greedy and might be wrong
        log.debug("parse might be fixable");
        currentParse.reviseParse();
        //now check if it's complete
        if (currentParse.checkTerminal()) {
          log.debug("sending revised parse");
          output = createParsedUtterance(currentParse.getCurrentParse(), speaker, addressee);
        } else {
          log.debug("it wasn't...");
          return input;
        }
      } else {
        //otherwise let the system no that we couldn't find a parse
        return input;
      }

      log.debug("currentInteractor " + speaker);
      parses.put(speaker, new Parse(terminalCategories.keySet()));
      log.debug("parse tree reset");
    }

    return output;
  }

  private Parse addEntriesForToken(List<Entry> entries, Symbol speaker) {
    Parse currentParse = parses.get(speaker);
    log.debug("\nadding: " + entries);
    if (!entries.isEmpty()) {
      log.debug("possible foundEntries size: " + entries.size());
      //initialize new Node in the Parse space
      Node found = new Node(entries);
      nodeCollection.add(found);
      //Try to combine the new Node with the rest of the current parse
      currentParse.addNode(found);
    }
    return currentParse;
  }

  //Update parse for given speaker with given entry
  private Parse addWord(Entry incoming, Symbol speaker) {
    Parse currentParse = parses.get(speaker);

    log.debug("\nadding entry: " + incoming);
    List<Entry> foundEntries = new ArrayList<>();
    foundEntries.add(incoming);

    //initialize new Node in the Parse space
    Node found = new Node(foundEntries);
    //Try to combine the new Node with the rest of the current parse
    currentParse.addNode(found);

    return currentParse;
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

  //Create an utterance
  public Utterance createUtterance(Node finalNode, Symbol speaker, Symbol addressee) {
    //NLPacket nlPacketTest = null;
    try {
      /// CREATE NEW UTTERANCE
      for (NodeEntry nodeRule : finalNode.rules) {
        Pair<List<Symbol>, Map<Variable, Symbol>> diarcSemantics = nodeRule.semantics.getDIARCSemantics();
        List<Symbol> fullSemantics = diarcSemantics.getLeft();
        log.debug("full semantics: " + fullSemantics);

        UtteranceType utteranceType = terminalCategories.get(nodeRule.syntax.type);
        if (utteranceType == null) {
          log.warn("Skipping nodeRule with unknown terminal category: {}", nodeRule.syntax.type);
          continue;
        }
        Symbol mainSemantics = fullSemantics.remove(0);
        mainSemantics = updateDirectAddress(mainSemantics, speaker, addressee, utteranceType);
        mainSemantics = updateSpeakerNames(mainSemantics, currentAddressee, speaker);
        List<Term> supplementalSemanticsTest = new ArrayList<>();
        for (Symbol semTest : fullSemantics) {
          if (semTest.isTerm()) {
            //TODO:brad: will we ever have ?ADDRESSEE or ?INTERACTOR in supplemental semantics?
            supplementalSemanticsTest.add((Term) updateSpeakerNames(semTest, addressHistory.get(Pair.of(speaker, addressee)), speaker));
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
    } catch (Exception e) {
      log.error("Error calling nodeRule");
    }
    return null;
  }

  //Collect utterances
  public List<Utterance> collectUtterances(Node finalNode, Utterance utterance, Symbol interactor, Symbol listener) {
    try {
      List<Utterance> utteranceList = new LinkedList<>();
      Node arg1 = finalNode;

      int ruleSize = finalNode.rules.size();
      for (int i = 0; i < ruleSize; i++) {
        List<Entry> final_rules = new ArrayList<>();
        Entry nodeRule = finalNode.rules.get(i);
        final_rules.add(nodeRule);
        Node combinedNode2 = new Node(final_rules);
        Utterance utterance2 = createUtterance(combinedNode2, interactor, listener);
        utteranceList.add(utterance2);
      }
      return utteranceList;

    } catch (Exception e) {
      log.error("Error calling collectUtterances");
    }
    return null;
  }

  //Pass along new utterance with updated semantics to another NLPComponent
  private Utterance createParsedUtterance(Node possibleSemantics, Symbol speaker, Symbol addressee) {
    log.debug("sending semantics");
    log.trace("number of parses found: " + possibleSemantics.rules.size());

    //iterat over possible parses at terminal Node
    for (NodeEntry rule : possibleSemantics.rules) {
      log.trace("complete? " + rule.semantics.isComplete());
      //if the semantics are complete, meaning there are no unbound lambda args
      if (rule.semantics.isComplete() && terminalCategories.get(rule.syntax.type) != null) {
        Pair<List<Symbol>, Map<Variable, Symbol>> DIARCSemantics = rule.semantics.getDIARCSemantics();
        List<Symbol> fullSemantics = DIARCSemantics.getLeft();
        log.debug("full semantics: " + fullSemantics);

        // start building utterance and set utterance type
        Utterance.Builder builder = new Utterance.Builder();
        UtteranceType type = terminalCategories.get(rule.syntax.type);
        builder.setUtteranceType(type);

        // set utterance semantics
        Symbol mainSemantics = fullSemantics.remove(0);
        mainSemantics = updateDirectAddress(mainSemantics, speaker, addressee, type);
        mainSemantics = updateSpeakerNames(mainSemantics, currentAddressee, speaker);
        builder.setSemantics(mainSemantics);

        // set utterance supplemental semantics
        for (Symbol sem : fullSemantics) {
          if (sem.isTerm()) {
            //TODO:brad: will we ever have ?ADDRESSEE or ?INTERACTOR in supplemental semantics?
            builder.addSupplementalSemantics((Term) updateSpeakerNames(sem, currentAddressee, speaker));
          } else {
            log.error("Badly formatted supplemental semantics: " + sem);
          }
        }
        // set remaining utterance info
        log.debug("Bindings: " + DIARCSemantics);
        builder.setSpeaker(speaker);
        builder.setAddressee(currentAddressee);
        builder.setTierAssignments(DIARCSemantics.getRight());
        builder.setIsInputUtterance(true);
        builder.setWords(Arrays.asList(rule.morpheme.split(" ")));

        Utterance utterance = builder.build();
        log.debug("utterance generated: " + utterance);
        log.debug("utterance words: " + utterance.getWordsAsString());

        // NOTE: if we want to support multiple parse options, we should process
        // the rest of the parse options and add them to the Utterance before returning

        // This is where we do chart parsing to collection multiple parses at the end of the utterance
        //TODO send partial semantics (e.g. "grab the mug") from "grab the mug on the table"
        //
        List<Utterance> final_parses = doChartParsing(utterance);
        doChartParsingIncrementally(utterance);
        doChartParsingWithDP(utterance);

        return final_parses.get(0);
      }
    }
    log.warn("tried to send incomplete semantics, nothing sent... " + possibleSemantics);
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
                try {
                  // Attempt to combine nodes
                  List<NodeEntry> validCombinations = left.combineTest(right);

                  // Check if there are valid combinations
                  if (!validCombinations.isEmpty()) {
                    //masterNodeEntry.add(validCombinations);

                    // Need to catch node combination errors
                    try {
                      log.debug("[addNodeToChart] Combining nodes: {} and {}", left, right);
                      Node combinedNode = new Node(left, right, validCombinations);
                      // Potential spot to create utterance with node and send down the pipeline
                      // Make a decision to reset chart to continue parsing
                      Node combinedNodeCopy = combinedNode.clone();

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
                    } catch (Exception e) {
                      log.error("[addNodeToChart] Error while creating combined node: {}", e.getMessage(), e);
                    }
                  } else {
                    log.debug("[addNodeToChart] No valid combinations found for nodes: {} and {}", left, right);
                  }
                } catch (Exception e) {
                  log.error("[addNodeToChart] Error during combineTest for nodes: {} and {}", left, right, e);
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
   * @param end The end index of the chart cell.
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

  private List<Utterance> doChartParsing(Utterance utterance) {
    log.debug("[doChartParsing] utterance: {}", utterance);

    LinkedList<Utterance> all_parses = new LinkedList();
    all_parses.add(utterance);

    //////////////////////////////////////////////////////////////////
    // TESTING CHART PARSING GIVEN WORDS, RULES, AND EXISTING METHODS
    // nodeCollection
    // TEST 1: TRY COMBINING
    ArrayList<List<NodeEntry>> masterNodeEntry = new ArrayList<>();
    List<Node> NodeCollectionCopy = new ArrayList<>();
    NodeCollectionCopy.addAll(nodeCollection);
    this.chart = new ArrayList[nodeCollection.size()][nodeCollection.size()];
    ArrayList<Node> allParses = new ArrayList<>();

    // initialize chart
    for (int i = 0; i < nodeCollection.size(); i++) {
      for (int j = 0; j < nodeCollection.size(); j++) {
        chart[i][j] = new ArrayList<>();
      }
    }

    // Populate the initial chart with nodes for individual words
    for (int i = 0; i < nodeCollection.size(); i++) {
      Node nodeWord = nodeCollection.get(i);
      List<NodeEntry> rules = nodeWord.rules;
      chart[i][i].add(nodeWord);
    }

    /////////////////////////////////////////////////////////////////////
    // Main parsing algorithm

    // Populate the chart based on the sentence and grammarRules
    for (int length = 1; length <= nodeCollection.size(); length++) {
      for (int start = 0; start <= nodeCollection.size() - length; start++) {
        int end = start + length - 1;

        // Iterate over possible split points
        for (int split = start; split < end; split++) {
          // Iterate over nodes in chart[start][split] and chart[split+1][end]
          for (Node left : chart[start][split]) {
            for (Node right : chart[split + 1][end]) {
              // Check for valid combinations
              //List<NodeEntry> validcombinationsTest = null;
              try {
                List<NodeEntry> validcombinationsTest = left.combineTest(right);

                if (!validcombinationsTest.isEmpty()) {
                  // Create a new combined node
                  Node combinedNode = new Node(left, right, validcombinationsTest);
                  chart[start][end].add(combinedNode);
                  allParses.add(combinedNode);

                }
              } catch (Exception e) {
                log.error("Error calling validcombinationsTest");
              }
            }
          }
        }
      }
    }

    /////////////////////////////////////////////////////////////////////

    for (Node n : nodeCollection) {
      System.out.println("NODE AND RULES");
      log.info(n.getLabel());
      int size = nodeCollection.size();

      for (int i = 0; i < size - 1; i++) {
        // Get the current node and its next node
        Node currentNode = n;
        Node nextNode = nodeCollection.get(i + 1);

        // Perform the operation on the current node and its next node
        NodeEntry combination = null;

        try {
          List<NodeEntry> validcombinationsTest = currentNode.combineTest(nextNode);
          if (!validcombinationsTest.isEmpty()) {
            masterNodeEntry.add(validcombinationsTest);
            try {
              log.info(currentNode.label);
              log.info(nextNode.label);
              Node combinedNode = new Node(currentNode, nextNode, validcombinationsTest);
              NodeCollectionCopy.add(combinedNode);
              log.info("COMBINED");
            } catch (Exception e) {
              log.error("Error calling validcombinationsTest");
            }
          }
          log.info("validcombinationsTest: {}", validcombinationsTest);
        } catch (Exception e) {
          log.error("Error calling validcombinationsTest");
        }
      }

      //if (combination != null) {
      //  log.trace("valid combinations size" + validCombinations.size());
      //}
      log.info("MADE IT OUT OF LOOP");

    }

    // grab --> (the mug)
    // ArrayList<NodeEntry> themuglist = masterNodeEntry.get(0);
    // Node n1 = new Node(this, partner, validCombinations);
    //ArrayList<NodeEntry> validcombinationsTest2 = NodeCollection.get(0).combineTest(themuglist.get(0));

    NodeEntry combination = null;

    List<Utterance> packetTest = new LinkedList<>();

    // get any parse from the chart (holds multiple parses)
    int endOfParse = nodeCollection.size() - 1;

    // collect all COMPLETED parses that have a single terminal symbol remaining "C"

    List<Node> completeParses = new LinkedList<>();

    List<Node>[] cellrow = this.chart[0];

    List<Node> lastList = cellrow[cellrow.length - 1]; // Accessing the last list in the array
    Node lastElement = lastList.get(lastList.size() - 1);

    for (Node endParse : lastList) {
      Utterance utTest = createUtterance(endParse, utterance.getSpeaker(), utterance.getAddressee());
      packetTest.add(utTest);

      //List<NodeEntry> rule = endParse.rules;

      //if (rule.get(0).semantics.isComplete() && terminalCategories.get(rule.get(0).syntax.type) != null) {
      //Pair<List<Symbol>, Map<Variable, Symbol>> DIARCSemantics = rule.get(0).semantics.getDIARCSemantics();
      //List<Symbol> fullSemantics = DIARCSemantics.getLeft();
      //log.debug("full semantics: " + fullSemantics);

      //add to final list of parses
      //packetTest.add(fullSemantics);
    }

    return packetTest;

    // END OF CHART PARSING
    //////////////////////////////////////////////////////////////////

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
