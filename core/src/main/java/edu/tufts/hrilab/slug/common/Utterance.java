/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.slug.common;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;

import java.util.*;
import java.io.Serializable;

import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.tufts.hrilab.fol.util.PragUtil;

/**
 * An Utterance is used to transmit information between component, including the
 * semantic meaning, the dialogue act, the speaker, and the listener(s).
 */
public final class Utterance implements Serializable {
  private static Logger log = LoggerFactory.getLogger(Utterance.class);

  private UtteranceType type;
  /**
   * If true, the utterance is from speech input.
   * If false, the utterance is for speech output.
   */
  private boolean isInputUtterance;
  /**
   * If true, this utterance needs to be validated by a human user before it can be used
   * by the rest of the system. This is currently for verifying LLM generated parses.
   */
  private boolean needsValidation = false;
  /**
   * Speaker of the utterance.
   */
  private Symbol speaker;
  /**
   * Listeners of the utterance. For now, it is assumed that the first listener is
   * the addressee.
   */
  private List<Symbol> listeners = new ArrayList<>();
  /**
   * Individual words that were recognized or are to be spoken.
   */
  private List<String> words;
  /**
   * Nested representation used by dialogue and action.
   */
  private Symbol semantics = null;
  /**
   * Cached bound semantics. These are not to be set externally, but rather
   * populated internally when getBoundSemantics is called. These will be cleared if
   * the bindings or semantics are changed, and repopulated on subsequent getBoundSemantics calls.
   */
  private List<Term> boundSemantics = null;
  /**
   * Nested representation used by dialogue and action.
   */
  private List<Term> indirectSemantics = new ArrayList<>();
  /**
   * Free-variable entity descriptors used by reference resolution.
   */
  private List<Term> supplementalSemantics = new ArrayList<>();
  private List<Map<Variable, Symbol>> bindings = new ArrayList<>();
  //TODO:brad: this should be part of supplemental semantics
  private Map<Variable, Symbol> tierAssignments = new LinkedHashMap<>();

  //TODO:add:getters and update builder for these, add javadoc
  private String language = "en";
  private Map<String,String> translations = new HashMap<>();

  /**
   * Convenience method for instantiating Utterance from ASL.
   * @param speaker
   * @param listener
   * @return
   */
  static public Utterance createOutputUtterance(Symbol speaker, Symbol listener) {
    Utterance newUtterance = new Builder()
            .setSpeaker(speaker)
            .addListener(listener)
            .setUtteranceType(UtteranceType.UNKNOWN)
            .setIsInputUtterance(false).build();
    return newUtterance;
  }

  /**
   * Use Builder instead of constructor.
   */
  @Deprecated
  public Utterance(Symbol speaker, Symbol listener, List<String> words, UtteranceType type, boolean isInput) {
    this.speaker = speaker;
    this.listeners.add(listener);
    this.words = words;
    this.type = type;
    this.isInputUtterance = isInput;
  }

  /**
   * Private constructor used by the Builder.
   * @param speaker
   * @param listeners
   * @param words
   * @param semantics
   * @param indirectSemantics
   * @param supSementics
   * @param bindings
   * @param tierAssignments
   * @param type
   * @param isInput
   */
  private Utterance(Symbol speaker, List<Symbol> listeners, List<String> words,
                   Symbol semantics, List<Term> indirectSemantics, List<Term> supSementics,
                   List<Map<Variable, Symbol>> bindings, Map<Variable, Symbol> tierAssignments,
                   UtteranceType type, boolean isInput, boolean needsValidation,
                   String language, Map<String,String> translations) {

    this.type = type;
    this.isInputUtterance = isInput;
    this.needsValidation = needsValidation;
    this.speaker = speaker;
    this.listeners = listeners;
    this.words = words;
    this.semantics = semantics;
    this.indirectSemantics = indirectSemantics;
    this.supplementalSemantics = supSementics;
    this.bindings = bindings;
    this.tierAssignments = tierAssignments;
    this.language = language;
    this.translations = translations;
  }

  public static class Builder {

    private UtteranceType type;
    private boolean isInputUtterance;
    private boolean needsValidation;
    private Symbol speaker;
    private List<Symbol> listeners;
    private List<String> words;
    private Symbol semantics;
    private List<Term> indirectSemantics;
    private List<Term> supplementalSemantics;
    private List<Map<Variable, Symbol>> bindings;
    private Map<Variable, Symbol> tierAssignments;
    private String language;
    private Map<String,String> translations;

    //TODO:brad:why do we need this constructor, couldn't it all be inlined above?
    public Builder() {
      type = UtteranceType.UNKNOWN;
      isInputUtterance = true;
      needsValidation = false;
      speaker = null;
      listeners = new ArrayList<>();
      words = new ArrayList<>();
      semantics = null;
      indirectSemantics = new ArrayList<>();
      supplementalSemantics = new ArrayList<>();
      bindings = new ArrayList<>();
      tierAssignments = new LinkedHashMap<>();
      language = "en";
      translations = new HashMap<>();
    }

    public Builder(Utterance source) {
      this.type = source.type;
      this.isInputUtterance = source.isInputUtterance;
      this.needsValidation = source.needsValidation;
      this.speaker = source.speaker;
      this.listeners = new ArrayList<>(source.listeners);
      this.words = new ArrayList<>(source.words);
      this.semantics = source.semantics;
      this.indirectSemantics = new ArrayList<>(source.indirectSemantics);
      this.supplementalSemantics = new ArrayList<>(source.supplementalSemantics);
      this.bindings = new ArrayList<>(source.bindings);
      this.tierAssignments = new LinkedHashMap<>(source.tierAssignments);
      this.language = source.getLanguage();
      this.translations = new HashMap<>(source.getTranslations());
    }

    public Utterance build() {
      return new Utterance(speaker, listeners, words, semantics, indirectSemantics, supplementalSemantics, bindings, tierAssignments, type, isInputUtterance, needsValidation, language, translations);
    }

    public Builder setSpeaker(Symbol speaker) {
      this.speaker = speaker;
      return this;
    }

    public Builder addListener(Symbol listener) {
      this.listeners.add(listener);
      return this;
    }

    public Builder addListeners(List<Symbol> listeners) {
      this.listeners.addAll(listeners);
      return this;
    }

    public Builder setListener(Symbol listener) {
      this.listeners.clear();
      this.listeners.add(listener);
      return this;
    }

    public Builder setUtteranceType(UtteranceType type) {
      this.type = type;
      return this;
    }

    public Builder setIsInputUtterance(boolean isInputUtterance) {
      this.isInputUtterance = isInputUtterance;
      return this;
    }

    public Builder setNeedsValidation(boolean needsValidation) {
      this.needsValidation = needsValidation;
      return this;
    }

    public Builder setWords(List<String> words) {
      this.words = new ArrayList<>(words);
      return this;
    }

    public Builder addWord(String word) {
      this.words.add(word);
      return this;
    }

    public Builder setSemantics(Symbol semantics) {
      this.semantics = semantics;
      return this;
    }

    public Builder setIndirectSemantics(List<Term> indirectSemantics) {
      this.indirectSemantics = new ArrayList<>(indirectSemantics);
      return this;
    }

    public Builder addIndirectSemantics(Term indirectSemantics) {
      this.indirectSemantics.add(indirectSemantics);
      return this;
    }

    public Builder setSupplementalSemantics(List<Term> supplementalSemantics) {
      if (supplementalSemantics != null) {
        this.supplementalSemantics = new ArrayList<>(supplementalSemantics);
      }
      return this;
    }

    public Builder addSupplementalSemantics(Term supplementalSemantics) {
      this.supplementalSemantics.add(supplementalSemantics);
      return this;
    }

    public Builder setBindings(List<Map<Variable, Symbol>> bindings) {
      this.bindings = new ArrayList<>(bindings);
      return this;
    }

    public Builder addBindings(Map<Variable, Symbol> bindings) {
      this.bindings.add(bindings);
      return this;
    }

    public Builder setTierAssignments(Map<Variable, Symbol> tierAssignments) {
      this.tierAssignments.clear();
      this.tierAssignments.putAll(tierAssignments);
      return this;
    }

    public Builder addTierAssignment(Variable variable, Symbol tierAssignment) {
      this.tierAssignments.put(variable, tierAssignment);
      return this;
    }

    public Builder setTranslations(Map<String,String> translations) {
      this.translations.clear();
      this.translations.putAll(translations);
      return this;
    }

    public Builder addTranslation(String language, String translatedWordsAsString) {
      this.translations.put(language, translatedWordsAsString);
      return this;
    }

    public Builder setLanguage(String language) {
      this.language = language;
      return this;
    }

  }

  /**
   * Is utterance an input utterance (true) or output utterance (false).
   *
   * @return
   */
  public boolean isInputUtterance() {
    return isInputUtterance;
  }

  /**
   * If true, this utterance needs to be validated by a human user before it can be used
   * by the rest of the system. This is currently for verifying LLM generated parses.
   * @return
   */
  public boolean needsValidation() {
    return needsValidation;
  }

  /**
   * Set the needsValidation flag.
   * @param validationFlag
   */
  public void setNeedsValidation(boolean validationFlag) {
    this.needsValidation = validationFlag;
  }

  /**
   * Get speaker of the utterance.
   *
   * @return
   */
  public Symbol getSpeaker() {
    return speaker;
  }

  /**
   * Get actor being addressed. For now, assumed to be the first listener.
   *
   * @return
   */
  public Symbol getAddressee() {
    return listeners.get(0);
  }

  /**
   * Get all actors known to be listeners of the utterance.
   *
   * @return
   */
  public List<Symbol> getListeners() {
    return listeners;
  }

  /**
   * Get semantic meaning of utterance.
   *
   * @return
   */
  public Symbol getSemantics() {
    return semantics;
  }

  /**
   * Get supplemental semantic of utterance.
   *
   * @return
   */
  public List<Term> getSupplementalSemantics() {
    return supplementalSemantics;
  }

  /**
   * Get type of utterance. Defined by UtteranceType enum.
   *
   * @return
   */
  public UtteranceType getType() {
    return type;
  }

  /**
   * Check if Utterance has words.
   *
   * @return
   */
  public boolean hasWords() {
    return (words != null && !words.isEmpty());
  }

  /**
   * Get copy of words list.
   * @return
   */
  public List<String> getWords() {
    return new ArrayList<>(words);
  }

  /**
   * Get words joined as a single String.
   *
   * @return
   */
  public String getWordsAsString() {
    return String.join(" ", words);
  }

  public Map<Variable, Symbol> getTierAssignments() {
    return this.tierAssignments;
  }

  public void setType(UtteranceType type) {
    this.type = type;
  }

  /**
   * Set words of this Utterance. The input String will be split on spaces.
   * @param words
   */
  public void setWords(String words) {
    this.words = new ArrayList<>(Arrays.asList(words.trim().split(" ")));
  }

  /**
   * Set the words of this Utterance.
   * @param words
   */
  public void setWords(List<String> words) {
    this.words = new ArrayList<>(words);
  }

  /**
   * Set the semantic meaning of the utterance.
   *
   * @param semantics
   */
  public void setSemantics(Symbol semantics) {
    this.semantics = semantics;
    this.boundSemantics = null;
  }

  /**
   * Set the supplemental semantics of the utterance.
   *
   * @param supplementalSemantics
   */
  public void setSupplementalSemantics(List<? extends Term> supplementalSemantics) {
    if (supplementalSemantics != null) {
      this.supplementalSemantics = new ArrayList<>(supplementalSemantics);
    }
  }

  public List<Term> getIndirectSemantics() {
    return indirectSemantics;
  }

  public void setIndirectSemantics(List<Term> indirectSemantics) {
    this.indirectSemantics = indirectSemantics;
  }

  public void addBinding(Map<Variable, Symbol> binding) {
    bindings.add(binding);
  }

  public void setBindings(List<Map<Variable, Symbol>> bindings) {
    this.bindings = bindings;
    this.boundSemantics = null;
  }

  public List<Map<Variable, Symbol>> getBindings() {
    return bindings;
  }

  /**
   * Get bound semantics.
   * @return
   */
  public List<Term> getBoundSemantics() {
    if (boundSemantics != null) {
      // return cached bound semantics
      return boundSemantics;
    } else {
      // attempt to create bound semantics
      if (semantics != null) {
        if (semantics.isTerm()) {
          boundSemantics = getBoundSemantics((Term) semantics);
          return boundSemantics;
        } else {
          // should be a warn?
          log.debug("[getBoundSemantics] semantics is not of type Term: " + semantics);
        }
      }

      // no bound semantics can be created
      return new ArrayList<>();
    }
  }

  /**
   * Get bound indirect semantics.
   * @return
   */
  public List<List<Term>> getBoundIndirectSemantics() {
    List<List<Term>> semBindings = new ArrayList<>();
    for (Term is : indirectSemantics) {
      semBindings.add(getBoundSemantics(is));
    }
    return semBindings;
  }

  /**
   * Get bound supplemental semantics.
   * @return
   */
  public List<List<Term>> getBoundSupplementalSemantics() {
    List<List<Term>> semBindings = new ArrayList<>();
    for (Term is : supplementalSemantics) {
      semBindings.add(getBoundSemantics(is));
    }
    return semBindings;
  }

  /**
   * Get bound semantics in Predicate form.
   * e.g., semantics(boundOption1, ...)
   *
   * @return
   */
  public Predicate getBoundSemanticsPredicate() {
    return Factory.createPredicate("semantics", getBoundSemantics());
  }

  /**
   * Get bound indirect semantics in Predicate form.
   * e.g., indirectSemantics(semantics(boundOption1, ...),semantics(boundOption, ..),...)
   *
   * @return
   */
  public Predicate getBoundIndirectSemanticsPredicate() {
    List<Term> indirectSemanticsArgs = new ArrayList<>();
    for (List<Term> bis : getBoundIndirectSemantics()) {
      List<Term> semanticsArgs = new ArrayList<>();
      for (Term boundSem : bis) {
        semanticsArgs.add(boundSem);
      }
      indirectSemanticsArgs.add(Factory.createPredicate("semantics", semanticsArgs));
    }
    return Factory.createPredicate("indirectSemantics", indirectSemanticsArgs);
  }

  /**
   * Get bound supplemental semantics in Predicate form.
   * e.g., suppSemantics(semantics(boundOption1, ...),semantics(boundOption, ..),...)
   *
   * @return
   */
  public Predicate getBoundSupplementalSemanticsPredicate() {
    List<Term> suppSemanticsArgs = new ArrayList<>();
    for (List<Term> bss : getBoundSupplementalSemantics()) {
      List<Term> semanticsArgs = new ArrayList<>();
      for (Term boundSem : bss) {
        semanticsArgs.add(boundSem);
      }
      suppSemanticsArgs.add(Factory.createPredicate("semantics", semanticsArgs));
    }
    return Factory.createPredicate("suppSemantics", suppSemanticsArgs);
  }

  /**
   * Private helper function to bind any predicate with the local binding values.
   * @param semantics
   * @return
   */
  private List<Term> getBoundSemantics(Term semantics) {
    if (bindings.isEmpty()) {
      List<Term> noBindings = new ArrayList<>();
      noBindings.add(semantics);
      return noBindings;
    }
    List<Term> semBindings = new ArrayList<>();
    //We don't want to consider the types of the variables here I guess
    for (Map<Variable, Symbol> binding : bindings) {
      Term copyWithTypes = semantics.copyWithNewVariableTypes(binding.keySet());
      semBindings.add(PragUtil.getBoundTerm(binding, copyWithTypes));
    }
    return semBindings;
  }

  //brad: this assumes all free vars are put in to a binding, checks that there is a single binding for each variable that exists
  public boolean hasSingleBinding() {
    if (bindings.size() > 1) return false;
    for (Map<Variable, Symbol> binding : bindings) {
      for (Variable key : binding.keySet()) {
        if (binding.get(key) == null || binding.get(key).getName().equals("")) return false;
      }
    }
    return true;
  }

  public void addTranslation(String language, String translatedWordsAsString){
    translations.put(language,translatedWordsAsString);
  }

  public Map<String, String> getTranslations() {
    return translations;
  }

  public void setLanguage(String localeCode){
    language=localeCode;
  }
  public String getLanguage(){
    return language;
  }

  //TODO:brad:is this sufficient?
  @Override
  public boolean equals(Object o) {
    return o.toString().equals(this.toString());
  }

  @Override
  public int hashCode() {
    return this.toString().hashCode();
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();

    // add type info
    sb.append(type != null ? type.toString() : "_").append("(");

    // add speaker info
    sb.append(speaker != null ? speaker.toString() : "_").append(",");

    // add listener info
    if (listeners != null && !listeners.isEmpty() && listeners.get(0) != null) {
      sb.append(listeners.get(0).toString()).append(",");
    } else {
      sb.append("_").append(",");
    }

    // add semantics (or words if semantics is null or "null")
    if (semantics == null) {
//      sb.append("_");
      sb.append(words);
    } else if (semantics.toString().equalsIgnoreCase("null")) {
      sb.append("NLG(");
      if (words != null) {
        boolean firstWord = true;
        for (String word : words) {
          if (!firstWord) {
            sb.append(" ");
          }
          sb.append(word);
          firstWord = false;
        }
      }
      sb.append(")");
    } else {
      if (!bindings.isEmpty()) {
        sb.append(getBoundSemantics());
      } else {
        sb.append(semantics);
      }
    }
    sb.append(",");

    // print supplemental semantics modifiers
    sb.append("{");
    StringBuilder modifiers = new StringBuilder();
    boolean firstModifier = true;
    if (supplementalSemantics != null) {
      for (Symbol s : supplementalSemantics) {
        if (!firstModifier) {
          modifiers.append(",");
        }
        modifiers.append(s);
        firstModifier = false;
      }
    }

    // add tier assignments
    if (tierAssignments != null) {
      for (Variable k : tierAssignments.keySet()) {
        if (!firstModifier) {
          modifiers.append(",");
        }
        modifiers.append(new Term(tierAssignments.get(k).toString(), k));
        firstModifier = false;
      }
    }
    sb.append(modifiers);
    sb.append("}");

    // don't forget the closing paren
    sb.append(")");
    return sb.toString();
  }
}

