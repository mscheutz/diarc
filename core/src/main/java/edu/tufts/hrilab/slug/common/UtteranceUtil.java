/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.common;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.util.Util;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static edu.tufts.hrilab.fol.util.PragUtil.getBindingsSymHelper;
import static edu.tufts.hrilab.fol.util.PragUtil.getBoundSymbol;
import static edu.tufts.hrilab.fol.util.PragUtil.parseModifiers;

public class UtteranceUtil {

  private static Logger log = LoggerFactory.getLogger(UtteranceUtil.class);

  public static Utterance createUtterance(String line) {
    log.trace("PARSING UTTERANCE: " + line);
    int opar = line.indexOf('(');
    int cpar = line.lastIndexOf(')');
    String name = line.substring(0, opar);
    String args = line.substring(opar + 1, cpar).replaceAll(" ", "");

    Symbol speaker;
    Symbol listener;
    Symbol semantics;
    List<Term> modifiers;

    List<String> tokens = Util.tokenizeArgs(args);

    assert tokens != null;
    for (String s : tokens) {
      log.debug("Token s: " + s);
    }

    //Brad: these are calls to create FoL because they can be Symbols, Variables, or Predicates
    speaker = Factory.createFOL(tokens.get(0));
    listener = Factory.createFOL(tokens.get(1));
    semantics = Factory.createFOL(tokens.get(2));
    modifiers = parseModifiers(tokens.get(3));

    // NOTE: it's not clear from the usage is this Utterance should be an input or output utterance (assuming input)
    Utterance retUtt = new Utterance.Builder()
            .setSpeaker(speaker)
            .addListener(listener)
            .setSemantics(semantics)
            .setSupplementalSemantics(modifiers)
            .setUtteranceType(UtteranceType.valueOf(name))
            .setIsInputUtterance(true).build();

    log.debug("RETURNING: " + retUtt);
    return retUtt;
  }

  //used by prag during applicability checking
  public static Map<Variable, Symbol> getUtteranceBindings(Map<Variable, Symbol> bindings, Utterance u1, Utterance u2) {

    // 0. check type
    if (u1.getType() != u2.getType()) {
      return null;
    }

    // 1. check speaker
    bindings = getBindingsSymHelper(bindings, u1.getSpeaker(), u2.getSpeaker());

    // 2. check listeners - only check first listener for now
    bindings = getBindingsSymHelper(bindings, u1.getListeners().get(0), u2.getListeners().get(0));

    // 3. check semantics
    bindings = getBindingsSymHelper(bindings, u1.getSemantics(), u2.getSemantics());

    // 4. check modifiers
    if (u1.getSupplementalSemantics() == null) {
      // this is the case if a single variable is used (e.g., "Z", as oppose
      // to "{Z}" which is also valid but means match any single modifier)
      // to represent the modifiers in the rules file, indicating a
      // successful match against all possible modifiers (i.e., any number of
      // modifiers including the empty set).
    } else if (u1.getSupplementalSemantics().size() != u2.getSupplementalSemantics().size()) {
      return null;
    } else {
      for (int i = 0; i < u1.getSupplementalSemantics().size(); i++) {
        bindings = getBindingsSymHelper(bindings, u1.getSupplementalSemantics().get(i), u2.getSupplementalSemantics().get(i));
      }
    }
    return bindings;
  }

  //Used for pattern matching in prag
  public static Map<Variable, Symbol> getUtteranceBindings(Utterance u1, Utterance u2) {
    Map<Variable, Symbol> bindings = new HashMap<>();
    return getUtteranceBindings(bindings, u1, u2);
  }

  //used by prag rules to get FOL like bindings
  public static Utterance getBoundUtterance(Map<Variable, Symbol> bindings, Utterance u) {
    Symbol newSemantics = getBoundSymbol(bindings, u.getSemantics());

    Utterance.Builder retUtt = new Utterance.Builder()
            .setSpeaker(getBoundSymbol(bindings, u.getSpeaker()))
            .setSemantics(newSemantics)
            .setUtteranceType(u.getType())
            .setIsInputUtterance(u.isInputUtterance());

    // add listeners
    for (Symbol list : u.getListeners()) {
      retUtt.addListener(getBoundSymbol(bindings, list));
    }

//    if (u.adverbs != null) {
//      ArrayList<Symbol> newMods = new ArrayList<>();
//      for (Symbol mod : u.adverbs) {
//        newMods.add(getBoundSymbol(bindings, mod));
//      }
//      retUtt.adverbs = newMods;
//    }

    if (u.hasWords()) {
      retUtt.setWords(u.getWords());
    }

    return retUtt.build();
  }
}

