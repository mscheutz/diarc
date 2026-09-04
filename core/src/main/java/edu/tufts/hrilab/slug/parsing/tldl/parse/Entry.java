package edu.tufts.hrilab.slug.parsing.tldl.parse;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

//This class represents a parse rule
//It has 3 (or maybe 4 but cognitive status is part of the semantic rule class) parts:
//-morpheme (string representation of the word)
//-syntactic rule
//-semantic rule
//--cognitive status info (this is a combination of semantic and syntactic info and probably shouldn't be its own thing) it is used by reference resolution
//It also is used during the parsing process to represent state in the parse tree
//so accordingly, Entries can be created from the definitions in the dictionary well as the combination of other Entries
public class Entry implements Cloneable {
  //static private Template template = new Template();
  private static Logger log = LoggerFactory.getLogger(Entry.class);
  String morpheme;
  SyntacticRule syntacticRule;
  SemanticRule semanticRule;

  //used to construct empty version of the object and have the rest of it filled out
  //maybe should be a builder instead?
  public Entry() {
    morpheme = "";
    syntacticRule = null;
    semanticRule = null;
  }

  //TODO:brad, do we need this, and the other constructor?
  public Entry(String name, String grammar, String meaning, String cognitiveStatus) {
    morpheme = name;
    syntacticRule = new SyntacticRule(grammar);
    semanticRule = new SemanticRule(meaning);
    semanticRule.setGHTier(cognitiveStatus);
  }

  public Entry(String name, SyntacticRule grammar, SemanticRule meaning, String cognitiveStatus) {
    morpheme = name;
    try {
      syntacticRule = grammar.clone();
    } catch (Exception e) {
      log.error("exception cloning syntax.", e);
    }
    try {
      semanticRule = meaning.clone();
      semanticRule.setGHTier(cognitiveStatus);
    } catch (Exception e) {
      log.error("exception cloning semantics.", e);
    }
  }

  public Entry clone() throws CloneNotSupportedException {
    Entry clonedEntry = (Entry) super.clone();
    clonedEntry.morpheme = this.morpheme;
    if (syntacticRule == null) {
      clonedEntry.syntacticRule = null;
    } else {
      clonedEntry.syntacticRule = this.syntacticRule.clone();
    }
    if (semanticRule == null) {
      clonedEntry.semanticRule = null;
    } else {
      clonedEntry.semanticRule = this.semanticRule.clone();
    }
    return clonedEntry;
  }

  public Entry generateBoundWildcard(String wildcardValue) {
    Entry bound = null;
    try {
      bound = this.clone();
      bound.morpheme = bound.morpheme.replace("*", wildcardValue);
      bound.semanticRule = bound.getSemanticRule().bindWildcard(wildcardValue);

    } catch (CloneNotSupportedException e) {
      log.error("exception cloning an entry, I dont this this can happen?", e);
    }
    return bound;
  }

  @Override
  public String toString() {
    return "morpheme: " + morpheme + " | sem: " + semanticRule.toString() + " | syn: " + syntacticRule.toString() + " | gh: " + semanticRule.ghTier;
  }

  public SemanticRule getSemanticRule() {
    return semanticRule;
  }

  public SyntacticRule getSyntacticRule() {
    return syntacticRule;
  }

  public String getMorpheme() {
    return morpheme;
  }

  public String getCognitiveStatus() {
    return semanticRule.ghTier;
  }

  @Deprecated
  public int numArgs() {
    return semanticRule.getBodyArgs();
  }

  //This maybe should be a utility method or something
  //generate syntactic rules for all possible combinations of args
  //used in the new rule inference process
  static List<SyntacticRule> generateSyntaxes(List<List<SyntacticRule>> preceding, List<List<SyntacticRule>> following) {
    List<SyntacticRule> syntaxes = new ArrayList<>();
    List<List<SyntacticRule>> precedingCombinations = new ArrayList<>();
    List<SyntacticRule> tempListPreceding = new ArrayList<>();
    GeneratePermutations(preceding, precedingCombinations, 0, tempListPreceding);

    List<List<SyntacticRule>> followingCombinations = new ArrayList<>();
    List<SyntacticRule> tempListFollowing = new ArrayList<>();
    GeneratePermutations(following, followingCombinations, 0, tempListFollowing);

    for (List<SyntacticRule> pArgs : precedingCombinations) {
      for (List<SyntacticRule> fArgs : followingCombinations) {
        //TODO: make this not always a command, but how do we tell if it's a statement or question verb?
        SyntacticRule returnType;
        log.trace("pArgs: " + pArgs);
        if (pArgs.size() == 1 && pArgs.get(0).getType().equals("SUBJAGENT")) {
          //If i is the subject, then it can't be a command
          log.debug("inferred returnType: S");
          returnType = new SyntacticRule("S");
        } else {
          returnType = new SyntacticRule("C");
          log.debug("inferred returnType: C");
        }

        List<SyntacticRule> fArgsRev = fArgs.subList(0, fArgs.size());
        Collections.reverse(fArgsRev);
        SyntacticRule syntax = new SyntacticRule(returnType, pArgs, fArgsRev);

        syntaxes.add(syntax);
      }
    }
    return syntaxes;
  }

  //helper method for above
  private static void GeneratePermutations(List<List<SyntacticRule>> Lists, List<List<SyntacticRule>> result, int depth, List<SyntacticRule> current) {
    //if depth is 0, who cares, why do we need this?
    if (depth == Lists.size()) {
      List<SyntacticRule> copy = new ArrayList<>(current.size());
      for (SyntacticRule item : current) {
        log.debug("copy " + item.getType());
        copy.add(item);
      }
      result.add(copy);

      return;
    }

    for (int i = 0; i < Lists.get(depth).size(); ++i) {
      current.add(Lists.get(depth).get(i));
      GeneratePermutations(Lists, result, depth + 1, current);
      current.remove(current.size() - 1);
    }
  }
}
