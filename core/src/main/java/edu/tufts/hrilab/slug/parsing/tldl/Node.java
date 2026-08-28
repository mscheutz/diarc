package edu.tufts.hrilab.slug.parsing.tldl;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class Node implements Cloneable {
  private static Logger log = LoggerFactory.getLogger(Node.class);

  String label;
  List<NodeEntry> rules;
  Node lAncestor;
  Node rAncestor;
  boolean learned = false;

  // Constructor: Create a node from a list of dictionary entries
  public Node(List<Entry> entries) {
    label = entries.get(0).morpheme;
    rules = new ArrayList<>();
    for (Entry e : entries) {
      if (e.syntax != null) {
        rules.add(new NodeEntry(e, null, null));
      }
    }
    learned = true;
    lAncestor = null;
    rAncestor = null;
  }

  // Constructor: Create a new node during the combination process
  public Node(Node left, Node right, List<NodeEntry> validGrammar) {
    label = left.label + " " + right.label;
    rules = validGrammar;
    lAncestor = left;
    rAncestor = right;
  }

  // Combine this node with another
  public Node combine(Node partner) {
    List<NodeEntry> validCombinations = new ArrayList<>();
    for (NodeEntry current : this.rules) {
      for (NodeEntry next : partner.rules) {
        log.debug("Checking possible combination: \"{}\" \"{}\"", current, next);

        NodeEntry currentClone = null;
        NodeEntry nextClone = null;
        try {
          if (current != null) {
            currentClone = current.clone();
          }
          if (next != null) {
            nextClone = next.clone();
          }
        } catch (CloneNotSupportedException e) {
          log.error("Exception in cloning during combine: {}", e.getMessage());
        }

        if (currentClone == null || nextClone == null) {
          continue;
        }

        // directAddress checks: directAddress can't combine from both sides
        // i.e., direct address name can only appear at the beginning or end of utterance, not the middle
        if (currentClone.getSemantics().body.startsWith("directAddress") && this.lAncestor != null && this.rAncestor != null) {
          continue;
        }
        if (nextClone.getSemantics().body.startsWith("directAddress") && partner.lAncestor != null && partner.rAncestor != null) {
          continue;
        }

        NodeEntry combination = currentClone.combine(nextClone);
        if (combination != null) {
          validCombinations.add(combination);
        }
      }
    }

    if (!validCombinations.isEmpty()) {
      log.debug("Combining {} and {}", this.label, partner.label);
      return new Node(this, partner, validCombinations);
    } else {
      return null;
    }
  }

  // Reverse combine (special logic for handling REF nodes)
  public Node reverseCombine(Node partner) {
    List<NodeEntry> validCombinations = new ArrayList<>();
    boolean reverse = false;
    boolean ref = false;

    for (NodeEntry current : this.rules) {
      for (NodeEntry next : partner.rules) {
        NodeEntry currentClone = null;
        NodeEntry nextClone = null;
        try {
          currentClone = (current != null) ? current.clone() : null;
          nextClone = (next != null) ? next.clone() : null;
        } catch (Exception e) {
          log.error("Exception in cloning during reverse combine: {}", e.getMessage());
        }

        NodeEntry combination = null;
        if (currentClone != null) {
          combination = currentClone.combine(nextClone);
          if (combination == null) {
            combination = nextClone.combine(currentClone);
            if (combination != null) {
              reverse = true;
            }
          }
        }
        if (combination != null) {
          validCombinations.add(combination);
        }
      }
    }

    if (!validCombinations.isEmpty()) {
      return reverse ? new Node(partner, this, validCombinations) : new Node(this, partner, validCombinations);
    }

    return null;
  }

  // Getters
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

  @Override
  public Node clone() throws CloneNotSupportedException {
    Node clonedNode = (Node) super.clone();

    // Deep copy the rules list
    if (this.rules != null) {
      clonedNode.rules = new ArrayList<>();
      for (NodeEntry rule : this.rules) {
        clonedNode.rules.add(rule.clone());
      }
    }

    // Deep copy the ancestors
    clonedNode.lAncestor = (this.lAncestor != null) ? this.lAncestor.clone() : null;
    clonedNode.rAncestor = (this.rAncestor != null) ? this.rAncestor.clone() : null;

    return clonedNode;
  }

  // Check for node equality
  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null || getClass() != obj.getClass()) {
      return false;
    }
    Node other = (Node) obj;

    // Compare label, rules, and ancestors
    return Objects.equals(label, other.label) &&
            Objects.equals(rules, other.rules) &&
            Objects.equals(lAncestor, other.lAncestor) &&
            Objects.equals(rAncestor, other.rAncestor);
  }

  @Override
  public int hashCode() {
    return Objects.hash(label, rules, lAncestor, rAncestor);
  }
}
