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
   *  (speaker->most recent addressee) . this is protected so it can be used by the component test.
   */
  private Map<Pair<Symbol, Symbol>, Symbol> addressHistory;
  /**
   * TODO:temporary: whenever an utterance switch addressee, subsequent utterances are directed at that addressee until it is switched.
   */
  boolean updateAddressHistory = true;
  //(speaker -> current parse if it exists), only exists in cases of incremental input
  private Map<Symbol, Parse> parses;

  static Dictionary dictionary;
  /**
   * List of .dict files currently in use by dictionary, including resource config path.
   */
  private List<String> dictionaryFiles;
  private String resourceConfigPath = "config/edu/tufts/hrilab/slug/parsing/tldl";
  //the set of CCG types which denote the end of an utterance, and their corresponding slug.common.UtteranceType
  private Map<String, UtteranceType> terminalCategories;

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

  @Override
  protected void shutdownComponent() {
    try {
      TRADE.deregister(dictionary);
    } catch (TRADEException e) {
      log.error("[TLDLParserComponent] exception deregistering dictionary", e);
    }
    parses.clear();
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
      updateAddressHistory = false;
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
   * removes direct address wrappers from generated semantics, and updates, parser's current addressee field
   */
  private Symbol updateDirectAddress(Symbol semantics, Symbol interactor, Symbol listner, UtteranceType t) {

    if (semantics.isTerm()) {
      if (semantics.getName().equals("directAddress")) {
        addressHistory.put(Pair.of(interactor, listner), ((Term) semantics).getArgs().get(0));
        return ((Term) semantics).get(1);
      }
      if (t.equals(UtteranceType.INSTRUCT)
              && !((Term) semantics).getArgs().isEmpty()
              && !((Term) semantics).getArgs().get(0).getName().equals("?ADDRESSEE")
              && !((Term) semantics).getArgs().get(0).equals(addressHistory.get(Pair.of(interactor, listner)))
      ) {
        if(updateAddressHistory) {
          addressHistory.put(Pair.of(interactor, listner), ((Term) semantics).getArgs().get(0));
        }
      }
    }
    return semantics;
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

  private Symbol updateSpeakerNames(Symbol semantics, Symbol addressee, Symbol interactor) {
    Symbol groundedSemantics = updateSpeakerNamesHelper(semantics, new Variable("?ADDRESSEE"), addressee);
    return updateSpeakerNamesHelper(groundedSemantics, new Variable("?INTERACTOR"), interactor);
  }

  @Override
  public Utterance parseUtterance(Utterance input) {
    Utterance output = parseUtteranceHelper(input);
    if (output.getSemantics() == null) {
      // need this case to correctly set the addressee so it doesn't use default "self"
      Symbol addressee = addressHistory.get(Pair.of(input.getSpeaker(),input.getAddressee()));
      addressee = addressee == null ? input.getAddressee() : addressee;
      output = new Utterance.Builder(output).setListener(addressee).build();
    }
    log.info("[parseUtterance] Words: {} Result: {}", input.getWordsAsString(), output);
    return output;
  }

  /**
   * First step of Utterance Processing, updates address history and passes utterance to
   * addWords. This allows for parallelization across incoming utterances, while still preserving
   * incremental processing to some degree.
   *
   * Tokenizes the text of the utterance and updates the parse with each token in order.
   *
   * @param input utterance to be parsed
   * @return parsed Utterance (or Utterance with failure semantics)
   */
  private Utterance parseUtteranceHelper(Utterance input) {
    //TODO:not sure if we need to pass listeners on here...
    addressHistory.computeIfAbsent(Pair.of(input.getSpeaker(), input.getListeners().get(0)), k -> input.getListeners().get(0));
    //TODO:brad: what are the side effects of turning this off?
//    if (!addressHistory.containsKey(incoming.getUtterance().getSpeaker())) {
//      addressHistory.put(incoming.getUtterance().getSpeaker(), incoming.getUtterance().getListeners().get(0));
//    }

    //Make sure there is a Parse object for the speaker of the current Utterance
    Symbol currentInteractor = input.getSpeaker();
    Symbol currentListener = input.getListeners().get(0);
    if (!parses.containsKey(currentInteractor)) {
      parses.put(currentInteractor, new Parse(terminalCategories.keySet()));
    }
    Parse currentParse = parses.get(currentInteractor);

    List<Pair<String, List<Entry>>> tokens = dictionary.tokenize(input.getWords());

    Utterance output;
    for (ListIterator<Pair<String, List<Entry>>> tokensIterator = tokens.listIterator(); tokensIterator.hasNext(); ) {
      Pair<String, List<Entry>> nextToken = tokensIterator.next();
      String token = nextToken.getLeft();
      List<Entry> entries= nextToken.getRight();
      if (entries.isEmpty()){
        log.warn("[parseUtterance] unknown token: "+token);
        log.debug("current interactor: " + currentInteractor);
        parses.put(currentInteractor, new Parse(terminalCategories.keySet()));
        // TODO: this should be handled in the NLU scripts
        output = generateErrorSemantics(currentInteractor, currentListener, token);
        output.setWords(input.getWords());
        return output;
      }
      currentParse = addEntriesForToken(entries,currentInteractor);
      //this occurs if we're in not in learning mode and we get a morpheme that's not in the dict
      if (currentParse == null) {
        parses.put(currentInteractor, new Parse(terminalCategories.keySet()));
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
          addWord(e, currentInteractor);
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
      output = createParsedUtterance(currentParse.getCurrentParse(), currentInteractor, currentListener);

      //reset parse tree once utterance has been sent
      //TODO: in the future we might want to keep track of the set of all previous parses somewhere
      parses.put(input.getSpeaker(), new Parse(terminalCategories.keySet()));

    }
    //otherwise check to see if we can make it a terminal case...
    else {
      log.debug("no valid parse and no more input in time.");
      log.debug("current parse: " + currentParse);
      //if we don't have parse rules for a word try to figure them out
      //TODO:brad: make a separate pipeline that results in in infer unknowns getting called.
      if (!currentParse.checkAllDefinitionsExist() && false) {
        log.debug("inferring new definitions ");
        dictionary.addEntries(currentParse.inferUnknowns());
        log.debug("sending inferred parse");
        output = createParsedUtterance(currentParse.getCurrentParse(), currentInteractor, currentListener);
      } else if (currentParse.checkComplete()) {
        //try to rebuild the tree because the current parsing algorithm is too greedy and might be wrong
        log.debug("parse might be fixable");
        currentParse.reviseParse();
        //now check if it's complete
        if (currentParse.checkTerminal()) {
          log.debug("sending revised parse");
          output = createParsedUtterance(currentParse.getCurrentParse(), currentInteractor, currentListener);
        } else {
          log.debug("it wasn't...");
          return input;
        }
      } else {
        //otherwise let the system no that we couldn't find a parse
        return input;
      }
      log.debug("currentInteractor " + currentInteractor);
      parses.put(currentInteractor, new Parse(terminalCategories.keySet()));
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
  private Utterance generateErrorSemantics(Symbol interactor, Symbol listener, String reason) {
    log.debug("sending error message");
   //TODO:brad:update this to use the utterance builder
    Utterance utterance = new Utterance(
            interactor,
      addressHistory.get(Pair.of(interactor,listener)),
      new ArrayList<>(),
      UtteranceType.UNKNOWN,
            false
    );
    Predicate failureSemantics = Factory.createPredicate("not(understand("+listener+",that))");
    if (!reason.isEmpty()){
      failureSemantics = Factory.createPredicate("error("+listener+",doNotKnowWhat("+listener+","+reason+",means))");
    }
    utterance.setSemantics( failureSemantics);
    utterance.setIndirectSemantics(new ArrayList<>());
    log.debug("utterance generated: " + utterance);
    log.debug("utterance words: " + utterance.getWords());
     return utterance;
  }

  //Pass along new utterance with updated semantics to another NLPComponent
  private Utterance createParsedUtterance(Node possibleSemantics, Symbol interactor, Symbol listener) {
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
        mainSemantics = updateDirectAddress(mainSemantics, interactor, listener, type);
        mainSemantics = updateSpeakerNames(mainSemantics, addressHistory.get(Pair.of(interactor, listener)), interactor);
        builder.setSemantics(mainSemantics);

        // set utterance supplemental semantics
        for (Symbol sem : fullSemantics) {
          if (sem.isTerm()) {
            //TODO:brad: will we ever have ?ADDRESSEE or ?INTERACTOR in supplemental semantics?
            builder.addSupplementalSemantics((Term) updateSpeakerNames(sem, addressHistory.get(Pair.of(interactor, listener)), interactor));
          } else {
            log.error("Badly formatted supplemental semantics: " + sem);
          }
        }
        // set remaining utterance info
        log.debug("Bindings: " + DIARCSemantics);
        builder.setSpeaker(interactor);
        builder.addListener(addressHistory.get(Pair.of(interactor, listener)));
        builder.setTierAssignments(DIARCSemantics.getRight());
        builder.setIsInputUtterance(true);
        builder.setWords(Arrays.asList(rule.morpheme.split(" ")));


        Utterance utterance = builder.build();
        log.debug("utterance generated: " + utterance);
        log.debug("utterance words: " + utterance.getWordsAsString());

        // NOTE: if we want to support multiple parse options, we should process
        // the rest of the parse options and add them to the Utterance before returning
        return utterance;
      }
    }
    log.warn("tried to send incomplete semantics, nothing sent... " + possibleSemantics);
    return null;
  }

  /**
   * Used in parser test to toggle updateAddressHistory
   * @param val new value
   */
  protected void setUpdateAddressHistory(boolean val) {
    updateAddressHistory = val;
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
    morpheme= morpheme.toLowerCase();
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
