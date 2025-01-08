/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.tldl;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

import static java.util.Objects.isNull;  

//This class is the highest level unit of a parse tree.
//Because i didn't know what I was doing when I wrote this class,
// it contains a set of possible meanings for this point in the parse,
// but that doesn't really make sense, because it only has a single pair of ancestors.
//Really all of the possible meanings of a parse should be kept in separate nodes in separate trees
//so that different possible meanings can be associated with different possible parse structures,
//and then we wouldn't need Parse.reviseParse/
public class Node implements Cloneable {
  private static Logger log = LoggerFactory.getLogger(Node.class);

  String label;
  List<NodeEntry> rules;
  //assuming we read left to right
  Node lAncestor;
  Node rAncestor;

  boolean learned = false;
  //may or may not need

  //Construct a node form a list of entries returned by the dictionary
  public Node(List<Entry> entries) {
    label = entries.get(0).morpheme;
    rules = new ArrayList<>();
    for (Entry e : entries) {
      if(e.syntax != null) rules.add(new NodeEntry(e,null,null));
    }
    learned = true;
    lAncestor = null;
    rAncestor = null;
  }

 //constructor used to create a new Node during the combination process, when NodeEntries already exist.
  public Node(Node left, Node right, List<NodeEntry> validGrammar) {
    label = left.label + " " + right.label;
    rules = validGrammar;
    lAncestor = left;
    rAncestor = right;
  }

  //Combine Node with another Node in the parse space, assumed to occur after it in the utterance?
  public Node combine(Node partner) {
    ArrayList<NodeEntry> validCombinations = new ArrayList<>();
    for (NodeEntry current : this.rules) {
      for (NodeEntry next : partner.rules) {
        log.debug("checking possible combination: \""+current+"\" \""+next+"\"");
        //cloning here so multiple trees can be built
        NodeEntry currentClone=null;
        NodeEntry nextClone=null;
        try{
          if(current !=  null) {
            currentClone=current.clone();
          }
          if(next != null) {
            nextClone = next.clone();
          }
        }catch (Exception e){
          log.error("Exception in cloning in combine. "+e);
        }

        NodeEntry combination =null;
        if(currentClone != null){
          combination = currentClone.combine(nextClone);
          //if null, nextClone.combine(currentCLone);
        }
        if (combination != null) validCombinations.add(combination);
        log.trace("valid combinations size" + validCombinations.size());
      }
    }

    if (!validCombinations.isEmpty()) {
      log.debug("Combining " + this.label + " and " + partner.label);
      return new Node(this, partner, validCombinations);
    } else {
      return null;
    }
  }

  public Node reverseCombine(Node partner) {
    List<NodeEntry> validCombinations = new ArrayList<>();
    boolean reverse = false;
    boolean ref = false;
    for (NodeEntry current : this.rules) {
      for (NodeEntry next : partner.rules) {
        //cloning here so multiple trees can be built
        NodeEntry currentClone = null;
        NodeEntry nextClone = null;
        try {
          currentClone = isNull(current) ? null : current.clone();
          nextClone = isNull(next) ? null : next.clone();
        } catch (Exception e) {
          log.error("Exception in cloning in combine. " + e);
        }

        NodeEntry combination = null;
        if (!isNull(currentClone)) {
          combination = currentClone.combine(nextClone);
          if (currentClone.syntax.toString().endsWith(("REF")) &&
                  nextClone.syntax.toString().equals("RN")) {
            ref = true;
          }

          if (combination == null) {
            combination = nextClone.combine(currentClone);
            if (combination != null) {
              reverse = true;
            }
          }
        }
        if (combination != null) validCombinations.add(combination);
        log.debug("valid combinations size" + validCombinations.size());
      }
    }
    if (!validCombinations.isEmpty()) {
      if (reverse == true) {
        return new Node(partner, this, validCombinations);
      } else {
        return new Node(this, partner, validCombinations);
      }
    } else if (ref == true && validCombinations.isEmpty()) {
      String line = "the: REF/RN : #x.$x : DEFINITE";
      String[] splitLine = line.split(":");
      Entry e = new Entry(splitLine[0].trim(), new SyntacticRule(splitLine[1].trim()), new SemanticRule(splitLine[2].trim()), splitLine[3].trim());
      List<Entry> temp = new ArrayList();
      temp.add(e);
      Node n = new Node(temp);
      Node articleCombo = n.reverseCombine(partner);
      return this.reverseCombine(articleCombo);
    }

    return null;

  }

  public List<NodeEntry> combineTest(Node partner) {
    List<NodeEntry> validCombinations = new ArrayList<>();

    for (NodeEntry current : this.rules) {
      log.debug("POSSIBLE PARSE: {}", current);
      for (NodeEntry next : partner.rules) {
        log.debug("checking possible combination: \"" + current + "\" \"" + next + "\"");
        //cloning here so multiple trees can be built
        NodeEntry currentClone = null;
        NodeEntry nextClone = null;
        try {
          if (current != null) {
            currentClone = current.clone();
          }
          if (next != null) {
            nextClone = next.clone();
          }
        } catch (Exception e) {
          log.error("Exception in cloning in combine. " + e);
        }

        NodeEntry combination = null;
        if (currentClone != null) {
          combination = currentClone.combine(nextClone);
          //if null, nextClone.combine(currentCLone);
        }
        if (combination != null) validCombinations.add(combination);
        log.trace("valid combinations size" + validCombinations.size());
      }
    }

    if (!validCombinations.isEmpty()) {
      log.debug("Combining " + this.label + " and " + partner.label);
      return validCombinations;
    } else {
      return null;
    }
  }

  public String getLabel() {
    return label;
  }

  public List<NodeEntry> getRules() {
    return rules;
  }

  public List<String> getSemantics() {
    List<String> possibleSemantics = new ArrayList<>();
    for (NodeEntry e : rules) {
      possibleSemantics.add(e.semantics.getSemantics());
    }
    return possibleSemantics;
  }

  void addRule(Entry e) {
    rules.add(new NodeEntry(e, null, null));
  }

  //@Override
  public Node clone() throws CloneNotSupportedException {
    Node clonedNode = (Node) super.clone();

    // Deep copy the rules list
    if (this.rules != null) {
      clonedNode.rules = new ArrayList<>();
      for (NodeEntry rule : this.rules) {
        clonedNode.rules.add(rule.clone()); // NodeEntry already has a clone method
      }
    }

    // Deep copy the ancestors (if needed)
    if (this.lAncestor != null) {
      clonedNode.lAncestor = this.lAncestor.clone();
    }
    if (this.rAncestor != null) {
      clonedNode.rAncestor = this.rAncestor.clone();
    }

    return clonedNode;
  }
}


