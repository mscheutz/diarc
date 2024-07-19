/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.tldl;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.StringUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.text.DecimalFormat;
import java.util.*;

public class Dictionary {

  //TODO: save updated dictionaries to file

  private static final Logger log = LoggerFactory.getLogger(Dictionary.class);

  //these are linked hashmaps to preserve the insertion order of rules aka the order of the rules in teh .dict file
  //this is not ideal, but it is necessary for the wildcard morpheme stuff to work
  private Map<String, List<Entry>> entries = Collections.synchronizedMap(new LinkedHashMap<>());
  private Map<String, List<TemplateEntry>> templateTypes = new LinkedHashMap<>();

  boolean acceptAll= false;

  //List of groupos of homophones that can be used interchangably
  private static List<List<String>> homophoneGroups;

  class TemplateEntry {
    String syntax;
    String semantics;
    String cogstatus;

    TemplateEntry(){
    }
  }

  public Dictionary() {
    populateHomophoneGroups();
  }

  public void setAcceptAll(boolean tv){
    acceptAll =tv;
  }

  //TODO:brad: do we want these to be two methods?
  private void insertEntry(String key, Entry value) {
    List<Entry> wrapper = new ArrayList<>();
    wrapper.add(value);
    insertEntries(key, wrapper);
  }

  private void insertEntries(String key, List<Entry> value) {
    if (entries.containsKey(key)) {
      entries.get(key).addAll(value);
    } else {
      entries.put(key, value);
    }
  }

  void loadFile(String filename) {
    InputStream in = getClass().getResourceAsStream(filename);
    try (BufferedReader br = new BufferedReader(new InputStreamReader(in))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (!line.startsWith("%") && !line.equals("")) { //comments

          //Create templates
          if (line.startsWith("START-TEMPLATES")) {
            readTemplate(br);
            continue;
          }
          readDefinition(line);
        }
      }
    } catch (Exception e) {
      log.error("exception loading dictionary file: "+filename,e);
    }

    log.debug("loaded: " + filename);
    log.debug(" entries in dictionary: " + entries.size());
  }

  /**
   * Adds line from .dict file to Dictonary instance. line format:
   * name ; syntax ; semantics ; cognitiveStatus
   * @param line line of Dictionary file to read
   */
  private void readDefinition(String line){
          String[] splitLine = line.split(";");
          for(int i=0; i<splitLine.length; i++){
            if(i==0){
              splitLine[i]=splitLine[i].trim();
            }
            else if(i==1){
              splitLine[i]=splitLine[i].replaceAll(" ", "");
            }
          }

    //template
          if (templateTypes.containsKey(splitLine[1].trim())) {
            insertEntries(splitLine[0].trim(), entriesFromTemplate(splitLine));
          } else if (splitLine.length == 3 || splitLine.length == 4) { //definition
            String[] syntaxes = splitLine[1].trim().split(",");
            String morpheme = splitLine[0].trim();
            String cogStat = "";
            if (splitLine.length == 4) {
              cogStat = splitLine[3].trim();
            }
            for (String s : syntaxes) {

               //Better way to handle dict entries with hardcoded args?
               //This technically works but isn't the clearest or best way to do this. generateHardcodedSemanticRule simply
               //   sets argsSet to false after calling the default string constructor. This causes any hardcoded semantic rule
               //   to be passed into Factory.createPredicate so that it is correctly understood as a predicate rather than
               //   an arbitrary string.
               //Any semantic rule with arguments will already have argsSet as false, so this is safe as things stand
               if (splitLine[2].contains("(") || splitLine[2].contains(")")) {
                 insertEntry(morpheme, new Entry(splitLine[0].trim(), new SyntacticRule(s.trim()), SemanticRule.generateHardcodedSemanticRule(splitLine[2].trim()), cogStat));
               } else {
                 insertEntry(morpheme, new Entry(splitLine[0].trim(), new SyntacticRule(s.trim()), new SemanticRule(splitLine[2].trim()), cogStat));
               }
            }
          }
          else {
            log.trace("split line len " + splitLine.length);
            log.error("Invalid dictionary line: " + line + " Not added.");
          }

  }

  private void readTemplate(BufferedReader br) {
    String line;
    try {
      line = br.readLine();
      while (!(line.equals("END-TEMPLATES"))) {
        log.debug("[readTemplate] curr line " + line);
        if (!line.startsWith("%") && !line.equals("")) {
          List<TemplateEntry> newtempentry = new ArrayList<>();
          line = line.replaceAll("\\s+", ""); //get rid of whitespace
          String[] labelsyntaxsplit = line.split(";", 2); //splits only on first instance of :
          //find and delete {}
          String[] entrytypes = labelsyntaxsplit[1].split("\\},");
          for (String s : entrytypes) {
            s = s.replaceAll("\\{", "");
            s = s.replaceAll("\\}", "");
            String[] indiventrytypes = s.split(";");
            String[] syntaxes = indiventrytypes[0].split(",");
            for (String t : syntaxes) {
              TemplateEntry temp = new TemplateEntry();
              temp.syntax = t.trim();
              temp.semantics = indiventrytypes[1].trim();
              temp.cogstatus = null;
              if (indiventrytypes.length == 3) {
                temp.cogstatus = indiventrytypes[2].trim();
              }
              newtempentry.add(temp);
            }
          }
          templateTypes.put(labelsyntaxsplit[0].trim(), newtempentry);
        }
        line = br.readLine();
      }
    } catch (Exception e) {
      log.error("error reading template: "+e);
    }
  }

  private List<Entry> entriesFromTemplate(String[] splitLine) {

    List<Entry> matchingEntries = new ArrayList<>();

    List<TemplateEntry> temp = templateTypes.get(splitLine[1].trim());
    //if no functor is provided
    String semfuncname;

    //there are multiple names-> {grab, pass} : VERB0 : placeholdername
    if (splitLine[0].substring(0, 1).equals("{")) {
      splitLine[0] = splitLine[0].replaceAll("\\{", "");
      splitLine[0] = splitLine[0].replaceAll("\\}", "");
      String[] names = splitLine[0].split(",");
      //no functor name specified
      if (splitLine.length == 2) {
        semfuncname = names[0].trim();
      } else {
        semfuncname = splitLine[2].trim();
      }

      for (String n : names) {
        for (TemplateEntry t : temp) {
          String cogStat = (t.cogstatus != null) ? t.cogstatus : "";
          matchingEntries.add(new Entry(n.trim(),
              new SyntacticRule(t.syntax.trim()),
              new SemanticRule(t.semantics.replace("?PH", semfuncname)),
              cogStat));
        }
      }
    }
    //there's only one name
    else {
      if (splitLine.length == 2) {
        semfuncname = splitLine[0].trim();
      } else {
        semfuncname = splitLine[2].trim();
      }
      for (TemplateEntry t : temp) {
        String morpheme = splitLine[0].trim();
        String cogStat = (t.cogstatus != null) ? t.cogstatus : "";
        matchingEntries.add(new Entry(morpheme,
            new SyntacticRule(t.syntax.trim()),
            new SemanticRule(t.semantics.replace("?PH", semfuncname)),
            cogStat));
      }
    }
    return matchingEntries;
  }

  List<Entry> getMatchingEntries(Entry newEntry) {

    List<Entry> matching = new ArrayList<>();

    for (Map.Entry<String, List<TemplateEntry>> entry : templateTypes.entrySet()) {
      List<TemplateEntry> posclass = entry.getValue();
      for (TemplateEntry te : posclass) {
        if (te.syntax.trim().equals(newEntry.syntax.type.trim())) {
          String[] temp = new String[2];
          temp[0] = newEntry.morpheme;
          temp[1] = entry.getKey().trim();
          matching.addAll(entriesFromTemplate(temp));
        }
      }
    }
    return matching;
  }

  public void addEntry(Entry newEntry) {
    insertEntry(newEntry.morpheme, newEntry);
    log.debug("New Entry added!");
  }

  void addEntries(List<Entry> newEntries) {
    //TODO:brad: think about this
    if (!newEntries.isEmpty()) {
      insertEntries(newEntries.get(0).morpheme, newEntries);
      log.debug("New Entries added!");
    }
  }


  private String changeNumToDoubleFormat(String ID) {
    try {
      return "" + Double.parseDouble(ID);
    } catch (NumberFormatException e) {
      log.error("Something passed to NUM is not actually a number: " + ID);
    }
    return "";
  }

  List<Entry> lookUpEntries(String ID) {

    //IF we do this first that lets us use numbers in morphemes if we want. e.g "m2 screw"
    if(entries.containsKey(ID)){
      return entries.get(ID);
    }
    if(ID.matches(".*\\d.*")){
      log.debug("found number: "+ID);
      //TODO: Brad: I think this means we can only handle whole numbers rn, no floating point, we should maybe we able to work around that with units?
      Entry number= new Entry(ID,new SyntacticRule("NUM"),new SemanticRule(ID),"");
      List<Entry> results= new ArrayList<>();
      results.add(number);
      return results;
    }
    return wordToNumber(ID);
  }

  Pair<List<String>, List<String>> getLex() {
    List<String> refWords = new ArrayList<>();
    List<String> nonRefWords = new ArrayList<>();

    synchronized (entries) {
      for (List<Entry> el : entries.values()) {
        for (Entry e : el) {
          if (e.getCognitiveStatus().equals("VAR")) {
            refWords.add(e.morpheme);
          } else {
            nonRefWords.add(e.morpheme);
          }
        }
      }
    }
    Set<String> uniqueRefWords = new HashSet<>(refWords);
    Set<String> uniqueNonRefWords = new HashSet<>(nonRefWords);

    uniqueNonRefWords.removeAll(uniqueRefWords);
    refWords = new ArrayList<>(uniqueRefWords);
    nonRefWords = new ArrayList<>(uniqueNonRefWords);

    return Pair.of(refWords, nonRefWords);
  }

  //looks up words in dictionary, and combines them in the case of multi-word dictionary entries, also does wildcard stuff for wildcard entries
  public List<Pair<String,List<Entry>>> tokenize(List<String> words) {

    List<String> lowercase = new ArrayList<>();
    for (String word : words) {
      if(!word.isEmpty()) {
        //if(word.matches(".*\\d.*")){
        //only convert sub 10 digit numbers, because that's all convert works for and also, we need to be able to support goal IDs
//        if (word.matches("^\\d{0,9}$")) {
//          //word=convertIntoWords(Double.parseDouble(word));
//          word = convert(Long.parseLong(word));
//        }
        lowercase.add(word.toLowerCase());
      }
    }

    List<Pair<String,List<Entry>>>  tokenBindings = new ArrayList<>();

    for (ListIterator<String> iterator = lowercase.listIterator(); iterator.hasNext(); ) {
      String word = iterator.next();
      List<Map.Entry<String,Boolean>> prefixes = new ArrayList<>();
      synchronized (entries) {
        for (String k : entries.keySet()) {
          if (k.startsWith(word + " ")) {
            if (k.contains("*")) {
              prefixes.add(new AbstractMap.SimpleEntry<>(k, true));
            } else {
              prefixes.add(new AbstractMap.SimpleEntry<>(k, false));
            }
          }
        }
      }
      if (prefixes.isEmpty()) {
        List<Entry> matchingEntries= lookUpEntries(word);
        if(matchingEntries != null) {
          tokenBindings.add(Pair.of(word,matchingEntries));
          log.trace("added tokenBinding no prefix "+word);
          log.trace("tokenBindings: " + tokenBindings);
        }else{
          tokenBindings.add(Pair.of(word, new ArrayList<>()));
          log.trace("added tokenBinding no prefix, no dictionary entry "+word);
        }
      }
      else {
        boolean added = false;
        //check if any of the prefixes match the next n elements of words
        //TODO:brad: it seems like there should be a way to do this with actual regexs, but I haven't been able to figure it out...
        for (Map.Entry<String, Boolean> prefix : prefixes) {
          String prefixString = prefix.getKey();

          if (prefix.getValue()) {
            //wildcard in morpheme case
            String[] boundaryWords = prefixString.split("\\s\\*\\s");
            if (boundaryWords.length == 2) {
              String[] prefixWords = boundaryWords[0].split("\\s+");
              String[] postfixWords = boundaryWords[1].split("\\s+");

              StringBuilder token = new StringBuilder();
              StringBuilder wildCardBody = new StringBuilder();
              boolean needsEscaping = false;

              //prefix
                String combined;
                String combinedPrefix;
              if (prefixWords.length <= lowercase.size() - iterator.previousIndex()) {
                List<String> prefixSequence = lowercase.subList(iterator.previousIndex(), iterator.previousIndex() + prefixWords.length);
                  combined = StringUtils.join(prefixSequence, " ");
                  combinedPrefix = StringUtils.join(prefixWords, " ");
                if (combined.equals(combinedPrefix)) {
                  token.append(combined).append(" ");
                } else {
                  log.trace("prefix didn't match");
                  continue;
                }
                } else {
                  log.trace("potential prefix longer than supplied utterance");
                  continue;
              }

              //iterate over the sublist
              boolean ended = false;
              int wildcardLength=0;
              //TODO:brad: are these off by 1?
              if(iterator.previousIndex()+prefixWords.length<lowercase.size()-iterator.previousIndex()) {
                for (String nextWord : lowercase.subList(iterator.previousIndex() + prefixWords.length,lowercase.size() - iterator.previousIndex())) {
                  if (nextWord.equals(postfixWords[0])) {
                    ended = true;
                    break;
                  } else {
                    needsEscaping = needsEscaping || needsEscaping(nextWord);
                    wildCardBody.append(nextWord).append(" ");
                    wildcardLength++;
                  }
                }
                if (!ended) {
                  log.trace("wild card didn't match");
                  continue;
                }
              }else{
                log.trace("wildcard pattern longer than utterance");
                continue;
              }
              //If prefix and postfix match but there is nothing in between to match with the wildcard,
              if (wildcardLength == 0) {
                log.trace("wildcard matched on empty body, discarding");
                continue;
              }
//              else if (needsEscaping) {
//                wildCardBody.insert(0,'"');
//                wildCardBody.insert(wildCardBody.length()-1,'"');
//              }
              token.append(wildCardBody);
              //post fix
              List<String> postfixSequence = lowercase.subList(iterator.previousIndex()+prefixWords.length+wildcardLength, iterator.previousIndex()+prefixWords.length+wildcardLength + postfixWords.length);
              combined = StringUtils.join(postfixSequence, " ");
              String combinedPostfix = StringUtils.join(postfixWords, " ");
              if (combined.equals(combinedPostfix)) {
                token.append(combined);
              }else{
                log.trace("postfix didn't match");
                continue;
              }
              //get the entry
              List<Entry> unbound = entries.get(prefixString);
              List<Entry> bound = new ArrayList<>();
              for (Entry e : unbound) {
                bound.add(e.generateBoundWildcard(wildCardBody.toString().trim()));
              }
              tokenBindings.add(Pair.of(token.toString(), bound));
              log.trace("added tokenBinding wildcard prefix"+word);
              added = true;
              //update iterator in utterance
              for (int i = 0; i < prefixWords.length+wildcardLength+ postfixWords.length - 1; i++) {
                iterator.next();
              }
            } else {
              log.error("badly formed wild card key in dictionary");
              return tokenBindings;
            }
          } else {
            String[] prefixWords = prefixString.split("\\s+");
            if (iterator.previousIndex() + prefixWords.length <= lowercase.size()) {
              List<String> relevantSequence = lowercase.subList(iterator.previousIndex(), iterator.previousIndex() + prefixWords.length);
              String combined = StringUtils.join(relevantSequence, " ");
              if (combined.equals(prefixString)) {
                tokenBindings.add(Pair.of(combined, entries.get(combined)));
                log.trace("added tokenBinding prefix: "+word);
                added = true;
                for (int i = 0; i < prefixWords.length - 1; i++) {
                  iterator.next();
                }
                break;
              }
            }
          }
        }

        //TODO:brad is this what we want?
        if (!added){
          List<Entry> dictionaryEntries= new ArrayList<>();
          if(entries.containsKey(word)){
            dictionaryEntries.addAll(entries.get(word));
          }
          tokenBindings.add(Pair.of(word,dictionaryEntries));
          log.trace("added tokenBinding catchall case: "+word);
        }
      }
    }
    log.debug("tokens: " + tokenBindings);
    return tokenBindings;
  }

  /**
   * Helper method used to determine if string needs to be escaped with''
   */
  private boolean needsEscaping(String s) {
    //Should we just change this to !isalphanumeric?
    return s.startsWith("!") || s.startsWith("?") || s.contains(":") || s.contains(",") || s.contains(")") || s.contains("(") || s.contains("'") || s.contains("\"") || s.contains("{") || s.contains("}");

//  //looks up words in dictionary, and combines them in the case of multi-word dictionary entries
//  public List<List<Entry>> tokenize(List<String> words) {
//
//    StringBuilder lowercase = new StringBuilder();
//    for (String word : words) {
//      //if(word.matches(".*\\d.*")){
//      //only convert sub 10 digit numbers, because that's all convert works for and also, we need to be able to support goal IDS
//      if(word.matches("^\\d{0,9}$")){
//        //word=convertIntoWords(Double.parseDouble(word));
//        word=convert(Long.parseLong(word));
//      }
//      lowercase.append(word.toLowerCase()).append(" ");
//    }
//
//    String utterance= lowercase.toString().trim();
//
//    List<Matcher> matchingMatchers = new ArrayList<>();
//    for(Pattern morpheme: entries.keySet()){
//      Matcher m= morpheme.matcher(utterance);
//      if(m.find()){
//        matchingMatchers.add(m);
//      }
//    }
//
//    //sort the matches in the order they match, i think things should be unique?
//    // TODO:brad: not sure what to do about substrings....
//    matchingMatchers.sort(new Comparator<Matcher>() {
//      @Override
//      public int compare(Matcher m1, Matcher m2) {
//        return m1.regionStart() - m2.regionStart();
//      }
//    });
//
//    List<List<Entry>> tokens = new ArrayList<>();
//
//    for(Matcher m: matchingMatchers){
//      List<Entry> matchingEntries= entries.get(m.pattern());
//      if(m.groupCount() > 0){
//        for(Entry e: matchingEntries){
//          //TODO:brad: is this going to work? are we modifying the entries in the dictionary based on a given utterance, if so does that matter?
//          e.wildcard=m.group();
//        }
//      }
//      tokens.add(matchingEntries);
//    }
//    return  tokens;
  }

  //brad: number conversion stuff so that we don't need to use icu4j which is huge, and I cant figure out how to only include the relevant locales. The jar alone is bigger than the entire rest of the app.
//TODO:brad doing it his way does remove support for decimals, which I don't matters in the Temi case, but probably does matter in the more general case

  private static final String[] tensNames = {
          "",
          " ten",
          " twenty",
          " thirty",
          " forty",
          " fifty",
          " sixty",
          " seventy",
          " eighty",
          " ninety"
  };

  private static final String[] numNames = {
          "",
          " one",
          " two",
          " three",
          " four",
          " five",
          " six",
          " seven",
          " eight",
          " nine",
          " ten",
          " eleven",
          " twelve",
          " thirteen",
          " fourteen",
          " fifteen",
          " sixteen",
          " seventeen",
          " eighteen",
          " nineteen"
  };

  private static String convertLessThanOneThousand(int number) {
    String soFar;

    if (number % 100 < 20){
      soFar = numNames[number % 100];
      number /= 100;
    }
    else {
      soFar = numNames[number % 10];
      number /= 10;

      soFar = tensNames[number % 10] + soFar;
      number /= 10;
    }
    if (number == 0) return soFar;
    return numNames[number] + " hundred" + soFar;
  }


  public static String convert(long number) {
    // 0 to 999 999 999 999
    if (number == 0) { return "zero"; }

    String snumber = Long.toString(number);

    // pad with "0"
    String mask = "000000000000";
    DecimalFormat df = new DecimalFormat(mask);
    snumber = df.format(number);

    // XXXnnnnnnnnn
    int billions = Integer.parseInt(snumber.substring(0,3));
    // nnnXXXnnnnnn
    int millions  = Integer.parseInt(snumber.substring(3,6));
    // nnnnnnXXXnnn
    int hundredThousands = Integer.parseInt(snumber.substring(6,9));
    // nnnnnnnnnXXX
    int thousands = Integer.parseInt(snumber.substring(9,12));

    String tradBillions;
    switch (billions) {
      case 0:
        tradBillions = "";
        break;
      case 1 :
        tradBillions = convertLessThanOneThousand(billions)
                + " billion ";
        break;
      default :
        tradBillions = convertLessThanOneThousand(billions)
                + " billion ";
    }
    String result =  tradBillions;

    String tradMillions;
    switch (millions) {
      case 0:
        tradMillions = "";
        break;
      case 1 :
        tradMillions = convertLessThanOneThousand(millions)
                + " million ";
        break;
      default :
        tradMillions = convertLessThanOneThousand(millions)
                + " million ";
    }
    result =  result + tradMillions;

    String tradHundredThousands;
    switch (hundredThousands) {
      case 0:
        tradHundredThousands = "";
        break;
      case 1 :
        tradHundredThousands = "one thousand ";
        break;
      default :
        tradHundredThousands = convertLessThanOneThousand(hundredThousands)
                + " thousand ";
    }
    result =  result + tradHundredThousands;

    String tradThousand;
    tradThousand = convertLessThanOneThousand(thousands);
    result =  result + tradThousand;

    // remove extra spaces!
    return result.replaceAll("^\\s+", "").replaceAll("\\b\\s{2,}\\b", " ");
  }

  //copied from https://stackoverflow.com/questions/26948858/converting-words-to-numbers-in-java
  public List<Entry> wordToNumber(String input) {

    List<Entry> e = new ArrayList<>();

    long result = 0;
    long finalResult = 0;
    List<String> allowedStrings = Arrays.asList
            (
                    "zero", "one", "two", "three", "four", "five", "six", "seven",
                    "eight", "nine", "ten", "eleven", "twelve", "thirteen", "fourteen",
                    "fifteen", "sixteen", "seventeen", "eighteen", "nineteen", "twenty",
                    "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety",
                    "hundred", "thousand", "million", "billion", "trillion"
            );

    if (input != null && input.length() > 0) {
      input = input.replaceAll("-", " ");
      input = input.toLowerCase().replaceAll(" and", " ");
      String[] splittedParts = input.trim().split("\\s+");

      for (String str : splittedParts) {
        if (!allowedStrings.contains(str)) {
          return e;
        }
      }
      for (String str : splittedParts) {
        if (str.equalsIgnoreCase("zero")) {
          result += 0;
        } else if (str.equalsIgnoreCase("one")) {
          result += 1;
        } else if (str.equalsIgnoreCase("two")) {
          result += 2;
        } else if (str.equalsIgnoreCase("three")) {
          result += 3;
        } else if (str.equalsIgnoreCase("four")) {
          result += 4;
        } else if (str.equalsIgnoreCase("five")) {
          result += 5;
        } else if (str.equalsIgnoreCase("six")) {
          result += 6;
        } else if (str.equalsIgnoreCase("seven")) {
          result += 7;
        } else if (str.equalsIgnoreCase("eight")) {
          result += 8;
        } else if (str.equalsIgnoreCase("nine")) {
          result += 9;
        } else if (str.equalsIgnoreCase("ten")) {
          result += 10;
        } else if (str.equalsIgnoreCase("eleven")) {
          result += 11;
        } else if (str.equalsIgnoreCase("twelve")) {
          result += 12;
        } else if (str.equalsIgnoreCase("thirteen")) {
          result += 13;
        } else if (str.equalsIgnoreCase("fourteen")) {
          result += 14;
        } else if (str.equalsIgnoreCase("fifteen")) {
          result += 15;
        } else if (str.equalsIgnoreCase("sixteen")) {
          result += 16;
        } else if (str.equalsIgnoreCase("seventeen")) {
          result += 17;
        } else if (str.equalsIgnoreCase("eighteen")) {
          result += 18;
        } else if (str.equalsIgnoreCase("nineteen")) {
          result += 19;
        } else if (str.equalsIgnoreCase("twenty")) {
          result += 20;
        } else if (str.equalsIgnoreCase("thirty")) {
          result += 30;
        } else if (str.equalsIgnoreCase("forty")) {
          result += 40;
        } else if (str.equalsIgnoreCase("fifty")) {
          result += 50;
        } else if (str.equalsIgnoreCase("sixty")) {
          result += 60;
        } else if (str.equalsIgnoreCase("seventy")) {
          result += 70;
        } else if (str.equalsIgnoreCase("eighty")) {
          result += 80;
        } else if (str.equalsIgnoreCase("ninety")) {
          result += 90;
        } else if (str.equalsIgnoreCase("hundred")) {
          result *= 100;
        } else if (str.equalsIgnoreCase("thousand")) {
          result *= 1000;
          finalResult += result;
          result = 0;
        } else if (str.equalsIgnoreCase("million")) {
          result *= 1000000;
          finalResult += result;
          result = 0;
        } else if (str.equalsIgnoreCase("billion")) {
          result *= 1000000000;
          finalResult += result;
          result = 0;
        } else if (str.equalsIgnoreCase("trillion")) {
          result *= 1000000000000L;
          finalResult += result;
          result = 0;
        }
      }

      finalResult += result;
    }
    e.add(new Entry(Long.toString(finalResult),"NUM",Long.toString(finalResult),""));
    return e;
  }

  //EW: Webapp Firebase specific
  //Function to get all dictionary entry morphemes for use in external homophone definition injection
  public Set<String> getKeySet() {
    return new HashSet<>(entries.keySet());
  }

  ///////////////////////////////////////////////////////////////////////////////
  //  Homophone implementation - do we want this here or more generally somewhere in parser?
  ///////////////////////////////////////////////////////////////////////////////

  private void populateHomophoneGroups() {
    //TODO:brad: put this somewhere better
    homophoneGroups = new ArrayList<>();
    List<String> oneGroup= new ArrayList<>();
    oneGroup.add("one");
    oneGroup.add("1");
    homophoneGroups.add(oneGroup);
    List<String> twoGroup= new ArrayList<>();
    twoGroup.add("to");
    twoGroup.add("too");
    twoGroup.add("two");
    twoGroup.add("ii");
    twoGroup.add("2");
    homophoneGroups.add(twoGroup);
    List<String> threeGroup= new ArrayList<>();
    threeGroup.add("three");
    threeGroup.add("3");
    homophoneGroups.add(threeGroup);
    List<String> fourGroup= new ArrayList<>();
    fourGroup.add("for");
    fourGroup.add("four");
    fourGroup.add("iv");
    fourGroup.add("4");
    homophoneGroups.add(fourGroup);
    List<String> fiveGroup= new ArrayList<>();
    fiveGroup.add("five");
    fiveGroup.add("5");
    homophoneGroups.add(fiveGroup);
    List<String> sixGroup= new ArrayList<>();
    sixGroup.add("six");
    sixGroup.add("6");
    homophoneGroups.add(sixGroup);
    List<String> sevenGroup= new ArrayList<>();
    sevenGroup.add("seven");
    sevenGroup.add("7");
    homophoneGroups.add(sevenGroup);
    List<String> eightGroup = new ArrayList<>();
    eightGroup.add("eight");
    eightGroup.add("8");
    homophoneGroups.add(eightGroup);
    List<String> nineGroup = new ArrayList<>();
    nineGroup.add("nine");
    nineGroup.add("9");
    homophoneGroups.add(nineGroup);
    List<String> homebaseGroup = new ArrayList<>();
    homebaseGroup.add("homebase");
    homebaseGroup.add("home base");
    homophoneGroups.add(homebaseGroup);
  }

  //TODO:brad: this only works for a single instance of a given homophone, if we want to do more this will get super complicated.
  private List<String> genPermutations(String full, List<String> homophones){
    List<String> permutations= new ArrayList<>();
    for(String homophone: homophones) {
      if (full.contains(homophone)) {
        for (String hprime : homophones) {
          permutations.add(full.replaceAll(homophone, hprime));
        }
      }
    }
    return permutations;
  }

  //Remove just LOC entries for a given key (potentially a homophone variation)
  public void removeLocEntries(String key) {
    if (entries.containsKey(key)) {
      List<Entry> updatedEntries = new ArrayList<>();
      for (Entry e : entries.get(key)) {
        if (!e.syntax.type.equals("LOC")) {
          updatedEntries.add(e);
        }
      }
      if (updatedEntries.isEmpty()) {
        entries.remove(key);
      } else {
        entries.put(key, updatedEntries);
      }
    }
  }

  //TODO: merge this with the above function and make it more general
  @TRADEService
  @Action
  public void removeEntry(String name, String grammar, String meaning, String cognitiveStatus) {
    log.info("removing wild entry for: "+name+" type: "+grammar+" semantics: "+meaning+" cs: "+cognitiveStatus);
    if (entries.containsKey(name)) {
      List<Entry> updatedEntries = new ArrayList<>();
      for (Entry e : entries.get(name)) {
        if (!(e.syntax.type.equals(grammar) && e.semantics.getSemantics().equals(meaning) && e.getCognitiveStatus().equals(cognitiveStatus))) {
          updatedEntries.add(e);
        }
      }
      if (updatedEntries.isEmpty()) {
        entries.remove(name);
      } else {
        entries.put(name, updatedEntries);
      }
    }
  }

  /**
   * Add entries for all homophone variations of new location, overwriting any potentially lingering old entries
   * e.g. changing from chair 2 to chair two would originally still have homophone entries referencing chair 2
   * @param base
   */
  @TRADEService
  public void generateLocationRules(String base, String semanticType, boolean delete){

    if (base.startsWith("\"") && base.endsWith("\"")) {
      base = base.substring(1,base.length()-1);
    }

    List<String> variations= new ArrayList<>();
    variations.add(base);

    for(List<String> h: homophoneGroups){
      variations.addAll(genPermutations(base,h));
    }

    for(String v: variations) {
      removeLocEntries(v);
      if (!delete) {
        //TODO:brad: pass in semantic type
        addEntry(new Entry(v, new SyntacticRule("LOC"), SemanticRule.generateLiteralSemanticRule(base,semanticType), "DEFINITE"));
      }
    }
  }

  @TRADEService
  @Action
  public String checkLocationName(String name) {
    List<Entry> nameEntries = entries.get(name);

    //not in dict
    if (nameEntries == null) {
      return null;
    }

    for (Entry e : lookUpEntries(name)) {
      //existing location
      if (e.syntax.type.equals("LOC")) {
        return e.getSemantics().getSemantics();
      }
      else {
        //other meaning
        return "";
      }
    }
    return null;
  }
}
