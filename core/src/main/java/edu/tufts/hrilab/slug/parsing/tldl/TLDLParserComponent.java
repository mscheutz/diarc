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
import edu.tufts.hrilab.slug.common.*;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.*;

import edu.tufts.hrilab.slug.parsing.tldl.chart.ChartParser;
import edu.tufts.hrilab.slug.parsing.tldl.parse.*;
import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import ai.thinkingrobots.trade.*;

public class TLDLParserComponent extends DiarcComponent {
  final protected Logger log = LoggerFactory.getLogger(this.getClass());

  /**
   * (speaker->most recent addressee)
   */
  private Map<Symbol, Symbol> addressHistory;
  /**
   * If true, use the addressHistory to keep track of most recent (speaker,addressee) pairs, in order to
   * track addressee across utterances that don't contain explicit direct address. This is true by default.
   */
  private boolean useAddressHistory = true;
  /**
   * A static dictionary used for storing and managing linguistic data
   * such as morphemes, semantics, and associated attributes for processing
   * and interpreting utterances within the natural language understanding component.
   */
  static Dictionary dictionary;
  /**
   * List of .dict files currently in use by dictionary, including resource config path.
   */
  private List<String> dictionaryFiles;
  /**
   * Default directory for dictionary files.
   */
  private String resourceConfigPath = "config/edu/tufts/hrilab/slug/parsing/tldl";
  /**
   * The set of CCG types which denote the end of an utterance, and their corresponding slug.common.UtteranceType
   */
  private Map<String, UtteranceType> terminalCategories;
  /**
   * Chart parser for parsing utterances.
   */
  private ChartParser chartParser = new ChartParser();
  /**
   * A list that stores pairs of homophones. Each pair is (base, homophone), where existing base parse rules
   * will be created for the homophone.
   */
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

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("dict").hasArgs().argName("file").desc("load dictionary from file").build());
    options.add(Option.builder("homophone").hasArgs().numberOfArgs(2).argName("base homophone").desc("add base parse rule for homophone").build());
    options.add(Option.builder("noUpdateAddressee").desc("do not update addressee after direct address").build());

    return options;
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

  /**
   * Main entry point for non-incremental parsing. The chart parser is reset before parsing.
   * @param input
   * @return
   */
  @Override
  public Utterance parseUtterance(Utterance input) {
    chartParser.resetChart();

    return parsePartialUtterance(input);
  }

  /**
   * Parses a (potentially partial) input Utterance and generates an Utterance containing
   * the parsed semantics, error semantics, or return the input as is (when there are no words to parse).
   *
   * This method does not reset the chart parser, and will add the incoming utterance to the previously
   * processed Utterance. Use the parseUtterance method for full, independent parses.
   *
   * NOTE: this is experimental. There is currently no mechansim to automaticlly detect the end of
   * an utternacee and reset the chart parser.
   *
   * @param input The input Utterance to be parsed. It should contain a list of words
   *              and other relevant metadata required for processing. If the input is
   *              null or contains no words, it will be returned as is.
   *
   * @return An updated Utterance constructed from the results of the parsing process.
   */
  public Utterance parsePartialUtterance(Utterance input) {
    if (input == null || input.getWords() == null || input.getWords().isEmpty()) {
      log.warn("[parseUtterance] Null or empty input utterance.");
      return input;
    }

    // Perform incremental parsing
    List<Node> finalParses = chartParser.parseIncrementally(input, dictionary);

    // Build output utterance from final parses
    Utterance output = buildPartialUtterance(input, finalParses);

    // use addressee history when there's no parse so the correct address generates the failure explanation
    if (output.getSemantics() == null && useAddressHistory && addressHistory.containsKey(input.getSpeaker())) {
      output = new Utterance.Builder(output).setAddressee(addressHistory.get(input.getSpeaker())).build();
    }
    log.info("[parseUtterance] Words: {} Result: {}", input.getWordsAsString(), output);
    return output;
  }

  /**
   * Temporary method to manually reset the chart parser if using the parsePartialUtterance method.
   */
  public void resetParser() {
    chartParser.resetChart();
  }

  /**
   * Constructs a partial Utterance by processing the input Utterance and a list of possible
   * parses. If a valid parse exists in the provided list, a new Utterance is created
   * based on the top-ranked parse. Otherwise, it handles the case of empty or invalid
   * parses by generating error semantics or returning the input as is.
   *
   * @param input The input Utterance to be processed.
   * @param finalParses A list of Node objects representing potential parses for the utterance.
   *                     This can be empty or null if no valid parses are produced.
   * @return A new Utterance built from the best parse in the list, or an Utterance with
   *         error semantics if no valid parses are available. Returns the original input
   *         Utterance if no modifications are required.
   */
  public Utterance buildPartialUtterance(Utterance input, List<Node> finalParses) {

    Symbol speaker = input.getSpeaker();
    Symbol addressee = input.getAddressee();

    // generate Utterances for all valid parses
    List<Utterance> utterances = new ArrayList<>();
    if (finalParses != null) {
      for (Node parse : finalParses) {
        utterances.addAll(createUtterances(parse, speaker, addressee));
      }
    }

    if (utterances.isEmpty()) {
      log.warn("[buildPartialUtterance] No valid parses found.");
      if (chartParser.hasUnkownTokens()) {
        Utterance errorSemantics = generateErrorSemantics(speaker, addressee, chartParser.getFirstUnknownTokens());
        errorSemantics.setWords(input.getWords());
        return errorSemantics;
      } else {
        // no specific unknown tokens, just couldn't parse
        return input;
      }
    }

    // TODO: perform reasoning on multiple parses here if needed
    // List<Utterance> candidateParses = getParsesFromChartCell(0, currentChartSize - 1, speaker, addressee);
    // List<Utterance> validParses = pruneParsesWithBelief(candidateParses);

    // Finally, return only the best parse's utterance
    Utterance finalUtterance = utterances.get(0);
    updateAddresseeHistory(finalUtterance.getSpeaker(), finalUtterance.getAddressee());
    return utterances.get(0);
  }

  //replaces current dictionary with one found at path
  public void addDictionary(String newDict) {
    String newDictWithPath = Resources.createFilepath(resourceConfigPath, newDict);
    dictionaryFiles.add(newDictWithPath);
    log.info("adding dictionary: " + newDictWithPath);
    dictionary.loadFile(newDictWithPath);
  }

  /**
   * Constructs an Utterance object based on the provided finalNode's rules and given speaker
   * and addressee symbols. The method processes semantic and syntactic information from the node's
   * rules to generate a structured Utterance.
   *
   * @param finalNode The Node object containing the rules from which the utterance is generated.
   * @param speaker   The Symbol representing the speaker of the input utterance.
   * @param addressee The Symbol representing the addressee of the input utterance (often unknown).
   * @return An Utterance object constructed based on the rules, semantics, and other properties of the node.
   * Returns null if no valid Utterance could be created from the node's rules.
   */
  public List<Utterance> createUtterances(Node finalNode, Symbol speaker, Symbol addressee) {
    List<Utterance> utterances = new ArrayList<>();
    for (NodeEntry nodeRule : finalNode.getRules()) {

      UtteranceType utteranceType = terminalCategories.get(nodeRule.getSyntacticRule().getType());
      if (utteranceType == null) {
        log.warn("Skipping nodeRule with unknown terminal category: {}", nodeRule.getSyntacticRule().getType());
        continue;
      }
      // reset variable names for each utterance
      VariableFactory.getInstance().reset();

      Pair<List<Symbol>, Map<Variable, Symbol>> diarcSemantics = nodeRule.getSemanticRule().getDIARCSemantics();
      List<Symbol> fullSemantics = diarcSemantics.getLeft();
      log.debug("full semantics: " + fullSemantics);

      Symbol mainSemantics = fullSemantics.remove(0);

      // get intended addressee based on semantics, direct address, and addressee history
      Symbol intendedAddressee = getAddressee(mainSemantics, speaker, addressee, utteranceType);

      // remove explicit directAddress semantics
      mainSemantics = stripDirectAddressSemantics(mainSemantics);

      // update semantics with intended addressee
      mainSemantics = updateSpeakerNames(mainSemantics, intendedAddressee, speaker);
      List<Term> supplementalSemanticsTest = new ArrayList<>();
      for (Symbol semTest : fullSemantics) {
        if (semTest.isTerm()) {
          //TODO:brad: will we ever have ?ADDRESSEE or ?INTERACTOR in supplemental semantics?
          supplementalSemanticsTest.add((Term) updateSpeakerNames(semTest, intendedAddressee, speaker));
        } else {
          log.warn("badly formatted supplemental semantics: " + semTest + " . Still added though");
          supplementalSemanticsTest.add((Term) semTest);
        }
      }
      log.debug("Bindings: " + diarcSemantics);

      // build utterance
      Utterance utterance = new Utterance.Builder()
              .setSpeaker(speaker)
              .setAddressee(intendedAddressee)
              .setSemantics(mainSemantics)
              .setSupplementalSemantics(supplementalSemanticsTest)
              .setTierAssignments(diarcSemantics.getRight())
              .setIsInputUtterance(true)
              .setUtteranceType(utteranceType)
              .setWords(Arrays.asList(nodeRule.getMorpheme().split(" ")))
              .build();

      utterances.add(utterance);
    }

    return utterances;
  }

  /**
   * Determines the addressee of an utterance based on the provided semantics, speaker,
   * addressee, and utterance type. It first attempts to extract a direct addressee
   * from the semantics. If no direct addressee can be determined, it uses the
   * address history (if enabled) or defaults to the addressee from the incoming
   * utterance.
   *
   * @param semantics The semantic representation of the utterance, which may contain
   *                  information about a direct addressee.
   * @param speaker   The Symbol representing the speaker of the utterance.
   * @param addressee The Symbol representing the original addressee of the utterance,
   *                  typically from incoming context.
   * @param type      The type of utterance (e.g., INSTRUCT), which may affect the determination
   *                  of the addressee.
   * @return A Symbol representing the determined addressee. This will be the direct
   *         addressee extracted from the semantics if available, the addressee based
   *         on history if enabled, or the incoming addressee if no other information
   *         is available.
   */
  private Symbol getAddressee(Symbol semantics, Symbol speaker, Symbol addressee, UtteranceType type) {
    Symbol returnAddressee = null;

    // try to extract direct address from semantics
    if (semantics.isTerm()) {
      if (semantics.getName().equals("directAddress")) {
        returnAddressee = ((Term) semantics).getArgs().get(0);
      } else if (type.equals(UtteranceType.INSTRUCT)
              && !((Term) semantics).getArgs().isEmpty()
              && !((Term) semantics).getArgs().get(0).getName().equals("?ADDRESSEE")) {
        returnAddressee = ((Term) semantics).getArgs().get(0);
      }
    }

    // if no direct address found in semantics, first try to use the addressee history, otherwise finally use the addressee from incoming Utterance
    if (returnAddressee == null) {
      if (useAddressHistory && addressHistory.containsKey(speaker)) {
        returnAddressee = addressHistory.get(speaker);
      } else {
        returnAddressee = addressee;
      }
    }

    return returnAddressee;
  }

  /**
   * Updates the addressee history by associating the given speaker with the given addressee,
   * if the address history functionality is enabled.
   *
   * @param speaker   The Symbol representing the speaker of the utterance.
   * @param addressee The Symbol representing the addressee of the utterance.
   */
  private void updateAddresseeHistory(Symbol speaker, Symbol addressee) {
    if (useAddressHistory) {
      addressHistory.put(speaker, addressee);
    }
  }

  /**
   * Removes the "directAddress" wrapper from the given semantic representation, if present.
   * If the input semantic contains "directAddress", the method extracts and returns the
   * nested semantics; otherwise, it returns the original semantic as is.
   *
   * @param semantics The semantic representation to be processed. It may or may not
   *                  contain a "directAddress" wrapper.
   * @return The processed semantic representation with the "directAddress" wrapper removed
   *         if it was present. Returns the original semantic representation if no such
   *         wrapper exists.
   */
  private Symbol stripDirectAddressSemantics(Symbol semantics) {
    Symbol returnSemantics = semantics;

    if (semantics.getName().equals("directAddress")) {
      returnSemantics = ((Term) semantics).get(1);
    }

    return returnSemantics;
  }

  /**
   * Updates the semantic representation by replacing placeholders for the addressee and
   * speaker with the corresponding actual symbols. The method ensures that the provided
   * semantics reflect the correct speaker and addressee information.
   *
   * @param semantics The initial semantic representation to be updated.
   * @param addressee The Symbol representing the addressee to replace in the semantics.
   * @param speaker   The Symbol representing the speaker to replace in the semantics.
   * @return A new Symbol containing the updated semantic representation with the specified
   *         speaker and addressee incorporated.
   */
  private Symbol updateSpeakerNames(Symbol semantics, Symbol addressee, Symbol speaker) {
    Symbol groundedSemantics = updateSpeakerNamesHelper(semantics, new Variable("?ADDRESSEE"), addressee);
    return updateSpeakerNamesHelper(groundedSemantics, new Variable("?INTERACTOR"), speaker);
  }

  /**
   * Recursively updates a semantic representation by replacing all occurrences of a target symbol
   * with a replacement symbol. This method traverses the structure of the given base symbol, which
   * may be either a term or a non-term symbol, and applies the replacement where applicable.
   *
   * @param base The Symbol to be processed and potentially updated. This can represent either a
   *             term with arguments or a non-term symbol.
   * @param target The Symbol to be replaced during the traversal. If any part of the `base` matches
   *               this target, it will be substituted with the replacement symbol.
   * @param replacement The Symbol to be substituted for occurrences of the target symbol within the
   *                    base.
   * @return A new Symbol containing the updated semantic representation with all instances of
   *         the target symbol replaced by the replacement symbol. If no replacement occurs, the
   *         original base symbol is returned.
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

  /**
   * Generates an Utterance object with error semantics when the input cannot be understood or
   * specific unknown words are encountered. The generated error message includes appropriate semantics
   * depending on whether the `unknownWords` parameter is empty or non-empty.
   *
   * @param speaker       The Symbol representing the speaker of the error utterance.
   * @param addressee     The Symbol representing the intended recipient or addressee of the error utterance.
   * @param unknownWords  A string containing the unknown words that could not be understood. If empty,
   *                      the generated semantics will not reference specific unknown words.
   * @return An Utterance object containing the error message along with the appropriate semantics and addressee.
   */
  private Utterance generateErrorSemantics(Symbol speaker, Symbol addressee, String unknownWords) {
    log.debug("sending error message");

    Utterance.Builder utterance = new Utterance.Builder(input);
    utterance.setSpeaker(speaker);
    if (useAddressHistory && addressHistory.containsKey(speaker)) {
      utterance.setAddressee(addressHistory.get(speaker));
    } else {
      utterance.setAddressee(addressee);
    }

    Predicate failureSemantics;
    if (unknownWords.isEmpty()) {
      failureSemantics = Factory.createPredicate("not(understand(" + addressee + ",that))");
    } else {
      failureSemantics = Factory.createPredicate("error(" + addressee + ",doNotKnowWhat(" + addressee + "," + unknownWords + ",means))");
    }
    utterance.setSemantics(failureSemantics);

    log.debug("utterance generated: " + utterance);
    return utterance.build();
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
    morpheme = morpheme.toLowerCase();
    log.debug("[injectDictionaryEntry] injecting entry for: " + morpheme + " type: " + type + " semantics: " + semantics + " cs: " + cognitiveStatus);
    Entry e = new Entry(morpheme, new SyntacticRule(type), new SemanticRule(semantics), cognitiveStatus);
    log.debug("[injectDictionaryEntry] entry: " + e);
    dictionary.addEntry(e);
  }

  @TRADEService
  public void addHomophone(String base, String homophone) {
    //TODO:brad:make this toggleable form the commandline
    addHomophone(base, homophone, false);
  }

  /**
   * Given a base morpheme and homophone, create and add all duplicate entries with homophone as new morpheme
   *
   * @param base
   * @param homophone
   * @param writeToFile
   */
  @TRADEService
  public void addHomophone(String base, String homophone, boolean writeToFile) {
    log.info("[addHomophone] have base: " + base + " and homophone: " + homophone);
    List<Entry> entries = dictionary.lookUpEntries(base);
    for (Entry entry : entries) {
      injectDictionaryEntry(homophone, entry.getSyntacticRule().getType(), entry.getSemanticRule().getSemantics(), entry.getCognitiveStatus());
      if (writeToFile) {
        //TODO: make output file configurable (through webapp)
        File tempASLFile = new File("resources/config/edu/tufts/hrilab/slug/parsing/tldl/assemblyHomophones.dict");
        try {
          tempASLFile.getParentFile().mkdirs();
          tempASLFile.createNewFile();
          FileOutputStream outputStream = new FileOutputStream(tempASLFile, true);
          byte[] strToBytes = ("\n%" + base + "\n" + homophone + "; " + entry.getSyntacticRule().getType() + "; " + entry.getSemanticRule().getDefinition()).getBytes();
          outputStream.write(strToBytes);
          outputStream.close();
        } catch (IOException e) {
          log.error("Unable to to modify homophone file: ", e);
        }
      }
    }
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

}
