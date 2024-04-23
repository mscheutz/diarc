/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.cache;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import static java.util.Map.entry;
import java.util.stream.Collectors;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.slug.common.UtteranceType;

public class CachedUtterance {
  private static Logger log = LoggerFactory.getLogger(CachedUtterance.class);

  private String type; //UtteranceType
  //public boolean needsValidation = false;
  //public String speaker;
  //public List<Symbol> listeners = new ArrayList<>(); //List<Symbol>
  public List<String> words;
  public String semantics = null; //Symbol
  //public List<String> boundSemantics = null; //List<Term>
  public List<String> indirectSemantics = new ArrayList<>(); //List<Term>
  //public List<String> supplementalSemantics = new ArrayList<>(); //List<Term>
  private List<Map<String, String>> bindings = new ArrayList<>(); //List<Map<Variable, Symbol>>
  //private Map<Variable, Symbol> tierAssignments = new LinkedHashMap<>();
  //private String language = "en";
  //private Map<String,List<String>> translations = new HashMap<>();

  public CachedUtterance (List<String> words, UtteranceType type, Symbol semantics, List<Term> indirectSemantics, List<Map<Variable, Symbol>> bindings) {
    this.words = words;
    this.type = type.toString();
    this.semantics = semantics.toString();
    this.indirectSemantics = new ArrayList<String>();

    for (Symbol indirectSemantic : indirectSemantics) {
      this.indirectSemantics.add(indirectSemantic.toString());
    }

    for (Map<Variable, Symbol> binding : bindings) {
      Map<String, String> bindingStrings = new HashMap<String, String>();
      for (Map.Entry<Variable, Symbol> entry : binding.entrySet()) {
        bindingStrings.put(entry.getKey().toString(), entry.getValue().toString());
      }
      this.bindings.add(bindingStrings);
    }
  }

  /**
   * Retrieves the UtteranceType based on the stored String value.
   *
   * @return The UtteranceType corresponding to the stored type value.
   */
  public UtteranceType getType () {
    return UtteranceType.valueOf(type);
  }

  /**
   * Retrieves the Predicate representing semantics.
   *
   * @return The Predicate representing semantics.
   */
  public Predicate getSemantics () {
    return Factory.createPredicate(semantics);
  }

  /**
   * Retrieves a list of Terms representing indirect semantics.
   *
   * @return A list of Terms representing indirect semantics.
   */
  public List<Term> getIndirectSemantics () {
    List<Term> indirectSemantics = new ArrayList<Term>();
    for (String indirectSemantic : this.indirectSemantics) {
      indirectSemantics.add(Factory.createPredicate(indirectSemantic));
    }
    return indirectSemantics;
  }

  /**
   * Retrieves a list of bindings represented as maps of Variable to Symbol.
   *
   * @return A list of bindings represented as maps of Variable to Symbol.
   */
  public List<Map<Variable, Symbol>> getBindings () {
    List<Map<Variable, Symbol>> bindings = new ArrayList<Map<Variable, Symbol>>();
    for (Map<String, String> bindingStrings : this.bindings) {
      Map<Variable, Symbol> binding = new HashMap<Variable, Symbol>();
      for (Map.Entry<String, String> entry : bindingStrings.entrySet()) {
        binding.put(new Variable(entry.getKey()), Factory.createSymbol(entry.getValue()) );
      }
      bindings.add(binding);
    }
    return bindings;
  }

  /**
   * Converts the object to its JSON representation.
   *
   * @return The JSON representation of the object.
   */
  public String toJSON () {
    Gson gson = new GsonBuilder().serializeNulls().create();
    return gson.toJson(this);
  }

  /**
   * Constructs a CachedUtterance object from its JSON representation.
   *
   * @param json The JSON string representing the CachedUtterance object.
   * @return The CachedUtterance object constructed from the JSON representation.
   */
  public static CachedUtterance fromJSON (String json) {
    Gson gson = new Gson();
    return gson.fromJson(json, CachedUtterance.class);
  }
}