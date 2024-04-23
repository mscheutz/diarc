/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.nlg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.util.*;

import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.common.Utterance;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import simplenlg.framework.*;
import simplenlg.lexicon.*;
import simplenlg.realiser.english.*;
import simplenlg.phrasespec.*;
import simplenlg.features.*;
import edu.tufts.hrilab.fol.util.PragUtil;

import ai.thinkingrobots.trade.*;

import static java.lang.Math.floor;

/**
 * This uses SimpleNLG to translate a list of lemmas into a
 * sentence.
 */

public class NLG {

  private static Logger log = LoggerFactory.getLogger(NLG.class);
  private static Lexicon lexicon = Lexicon.getDefaultLexicon();
  private static NLGFactory nlgFactory = new NLGFactory(lexicon);
  private static Realiser realiser = new Realiser(lexicon);
  private List<String> listeners;
  private String speaker;
  private List<Map<Variable, Symbol>> bindings = new ArrayList<>();
  private List<Term> supplements = new ArrayList<>();
  private Map<Variable, Symbol> tierAssignments = new HashMap<>();

  public NLG() {
  }

  public synchronized String translate(Utterance utterance) {
    log.debug("Translating: " + utterance);
    listeners = new ArrayList<>();
    for (Symbol listener : utterance.getListeners()) {
      listeners.add(listener.getName());
    }
    speaker = utterance.getSpeaker().getName();
    Symbol semantics = utterance.getSemantics();
    bindings = utterance.getBindings();
    supplements = utterance.getSupplementalSemantics();
    tierAssignments = utterance.getTierAssignments();
    if (log.isDebugEnabled()) {
      log.debug("binds: " + bindings);
      log.debug("sems: " + semantics);
      log.debug("supp: " + supplements);
      log.debug("type: " + utterance.getType());
      log.debug("tier assignments: " + tierAssignments);
    }
    if (semantics.isTerm()) {
      //log.debug("semantics: "+semantics);
      if (semantics.getName().equals("wouldlike")
          && ((Term) semantics).get(0).getName().equals("self")
          && ((Term) semantics).get(1).getName().equals("know")) {
        //Brad: it seems like all of this functionality is subsumed later when the new utterance is created...
//        if (((Term) ((Term) semantics).get(1)).get(0).getName().equals("self")) {
//          // log.debug("UTT WAS A QUEST");
//          utterance.getType() = UtteranceType.QUESTION;
//        } else {
//          // log.debug("NOT A QUEST: "+((Term)((Term)semantics).get(1)).get(0).getName());
//          // log.debug("SYM: "+semantics);
//          // log.debug("A1: "+((Term)((Term)semantics).get(1)));
//        }
        semantics = (((Term) ((Term) semantics).get(1)).get(1));
      }
      //log.debug("SYMBOL: "+semantics);

      //log.debug("semantics.toString() = " + semantics.toString());
      //String str = semantics.toString();

      //log.debug("type: "+(utterance.type.equals(UtteranceType.QUESTION)));
      //log.debug("getname: "+ ((Term)semantics).getName());

      if (utterance.getType().equals(UtteranceType.QUESTION) &&
          (semantics.getName().equals("or") || semantics.getName().equals("wouldLike"))) {
        //log.debug("IN");
        Utterance u0 = new Utterance.Builder()
                .setSpeaker(new Symbol("self"))
                .addListeners(utterance.getListeners())
                .setSemantics(((Term) semantics).get(0))
                .setUtteranceType(UtteranceType.QUESTION)
                .setIsInputUtterance(utterance.isInputUtterance()).build();

        Utterance u1 = new Utterance.Builder()
                .setSpeaker(new Symbol("self"))
                .addListeners(utterance.getListeners())
                .setSemantics(((Term) semantics).get(1))
                .setUtteranceType(UtteranceType.QUESTION)
                .setIsInputUtterance(utterance.isInputUtterance()).build();

        return translate(u0) + " or " + translate(u1);
      } else {

        NLGElement phrase = buildPhrase((Term) semantics);
        //log.debug("BUILT: "+phrase);

        //VPPhraseSpec verbPhrase = nlgFactory.createVerbPhrase(verb);
        //String adverbs = getAdverbModifiers();

        InterrogativeType questionType = getQuestionType(utterance);
        if (questionType != null) {
          phrase.setFeature(Feature.INTERROGATIVE_TYPE, questionType);
        }
        //if(utterance.type.equals(UtteranceType.QUESTION)) {
        //	phrase.setFeature(Feature.INTERROGATIVE_TYPE, InterrogativeType.YES_NO);
        //}

        String output = "";
        try {
          output = realiser.realise(phrase).getRealisation();
          log.debug("final: '"+output+"'");
        } catch (Exception e) {
          log.error("exception realizing output", e);
        }
        return output;
      }
    } else {
      List<Symbol> args = new ArrayList<>();
      args.add(semantics);
      SPhraseSpec clause = nlgFactory.createClause();
      NLGElement phrase = generateNP(getObject(args));
      clause.setSubject(phrase);
      return realiser.realise(clause).getRealisation();
    }
  }

  private NLGElement generateNP(Symbol s) {
    if (s.getName().contains("_") && Character.isDigit(s.getName().charAt(s.getName().length()-1))) {
      log.debug("REG NP: " + s.getName());
      if (s.getName().startsWith("_")) { //not a POWER reference
        return nlgFactory.createNounPhrase("something");
      } else {
        //reference resolution
        return generateREG(s);
//        log.error("REG not currently supported");
//        return null;
      }
    } else {
      NLGElement np;
      if (s.isVariable() && bindings.isEmpty()) {
        //if there is no binding for a variable, create noun phrase using supplements and tier assignments
        np = nlgFactory.createNounPhrase(getVariableReplacement((Variable) s));
        for (Symbol sym : supplements) {
          if (sym.isTerm() && ((Term) sym).get(0).equals(s)) {
            NPPhraseSpec n = nlgFactory.createNounPhrase(sym.getName());
            n.setDeterminer(generateDeterminer((Variable) s));
            np = n;
          }
        }
      } else if (s.isTerm() && s.getName().equals("possessive")) {
        List<Symbol> args = ((Term) s).getArgs();
        Symbol symbol = args.get(1);
        NPPhraseSpec n;
        if (symbol.isTerm() && ((Term) symbol).hasArgs()) {
          NLGElement argument = buildPhrase((Term) symbol);
          n = nlgFactory.createNounPhrase(argument);
        } else {
          n = (NPPhraseSpec) generateNP(symbol);
        }
        NPPhraseSpec p = (NPPhraseSpec) generateNP(args.get(0));
        p.setFeature(Feature.POSSESSIVE, true);
        n.setSpecifier(p);
        np = n;
      } else if (s.isTerm() && s.getName().equals("num")) {
        //2 painkillers
        Symbol number = ((Term)s).get(0);
        Symbol sym = ((Term)s).get(1);
        NPPhraseSpec n;
        if (Integer.parseInt(number.toString()) > 1) {
          n = nlgFactory.createNounPhrase(sym.getName());
          n.setPlural(true);
        } else {
          n = nlgFactory.createNounPhrase(sym.getName());
          n.setPlural(false);
          n.setDeterminer("a");
        }
//        n.addFrontModifier(number.getName());
        np = n;
      } else if (s.isTerm() && s.getName().equals("that")) {
        //that X
        Term semantics = (Term) s;
        Symbol t = semantics.get(0);
        SPhraseSpec n = nlgFactory.createClause();
        n.addComplement(buildPhrase((Term) t));
        n.setFeature(Feature.COMPLEMENTISER, "that");
        np = n;
      } else if (s.isTerm() && s.getName().equals("time")) {
        Symbol math = ((Term) s).get(0);
        NPPhraseSpec n;
        if (math.isVariable() && bindings.isEmpty()) {
          n = nlgFactory.createNounPhrase("no time");
        } else {
          n = nlgFactory.createNounPhrase("about");
          if (math.isVariable() && !bindings.isEmpty()) {
            Symbol sym = PragUtil.fillSymbol(bindings.get(0), math);
            log.debug("variable fill is " + sym);
            math = sym;
          }
          String mathStr = math.getName();
          if (math.getName().startsWith("\"") && math.getName().endsWith("\"")) {
            mathStr = mathStr.substring(1, mathStr.length()-1);
          }
          double number = Double.parseDouble(mathStr);
          int minutes = (int) floor(number/60);
          int seconds = (int)(number - (floor(number / 60) * 60));
          if (minutes >= 1) {
            if (seconds == 60) {
              //n = (NPPhraseSpec) generateNP(Factory.createSymbol(String.valueOf(minutes+1)));
              n.addPostModifier(generateNP(Factory.createSymbol(String.valueOf(minutes+1))));
            } else {
              //n = (NPPhraseSpec) generateNP(Factory.createSymbol(String.valueOf(minutes)));
              n.addPostModifier(generateNP(Factory.createSymbol(String.valueOf(minutes))));
            }
            n.addPostModifier("minutes");
            if (seconds > 0 && seconds < 60) {
              n.addPostModifier("and");
              n.addPostModifier(generateNP(Factory.createSymbol(String.valueOf(seconds))));
              n.addPostModifier("seconds");
            }
          } else {
            if (seconds > 0) {
              //n = (NPPhraseSpec) generateNP(Factory.createSymbol(String.valueOf(seconds)));
              n.addPostModifier(generateNP(Factory.createSymbol(String.valueOf(seconds))));
            } else {
              //n = (NPPhraseSpec) generateNP(math);
              n.addPostModifier(generateNP(math));
            }
            n.addPostModifier("seconds");
          }
        }
        np = n;
      } else if (s.isTerm() && (s.getName().equals("did") || s.getName().equals("gerund"))) {
        Term semantics = (Term) s;
//        Symbol symbol = semantics.get(1);
        //if (symbol.isTerm() && symbol.hasArgs()) {
          //elide subject in gerund-- i.e. "learning to nod" instead of "I learning to nod"
//          List<Symbol> args = ((Term) symbol).getArgs();
          np = buildPhrase(semantics);
          NLGElement sub = ((SPhraseSpec) np).getSubject();
          sub.setFeature(Feature.ELIDED,true);
          ((SPhraseSpec) np).setSubject(sub);
          np.setFeature(Feature.FORM,Form.GERUND);
        //} else {
          //for "the next step of dance is ..."
          //np = nlgFactory.createVerbPhrase(verbalize(symbol.getName()));
          //np.setFeature(Feature.FORM,Form.INFINITIVE);
        //}

      }else {
        if (s.isTerm() && s.getName().equals("prep")) {
          List<Symbol> args = ((Term) s).getArgs();
          NPPhraseSpec n = nlgFactory.createNounPhrase(generateNP(args.get(0)));
          args.remove(0);
          for (Symbol prep : args) {
            n.addPostModifier(nlgFactory.createNounPhrase(prep.getName()));
            if (prep.isTerm()) {
              NLGElement iobject = generateNP(((Term) prep).get(0));
              n.addPostModifier(iobject);
            }
          }
          np = n;
        } else if (s.isTerm() && s.hasArgs()) {
          NLGElement argument = buildPhrase((Term) s);
          np = nlgFactory.createNounPhrase(argument);
        } else {

          if (s.isVariable()) {
            //np = buildPhrase((Term) s);
            CoordinatedPhraseElement c = nlgFactory.createCoordinatedPhrase();
            for (Map<Variable, Symbol> binding : bindings) {
              Symbol sym = PragUtil.fillSymbol(binding, s);
              log.debug("variable fill is "+sym);
              NLGElement n = generateNP(sym);
              c.addCoordinate(n);
            }
            np = c;
            //np = nlgFactory.createNounPhrase(argument);
          } else {

            log.debug(s.getName());
            //split at camel case
            //we want to include periods for probability decimals
            //String[] strs = s.getName().split("\\.");
            String[] phrase = s.getName().split("(?<=[a-z])(?=[A-Z])");
            String str = phrase[0];
            boolean isInt = false;
            try {
              int num = Integer.parseInt(str);
              str = String.valueOf(num);
              isInt = true;
            } catch (NumberFormatException e) {
              log.debug(str+" not an integer.");
            }
            if (!isInt) {
              try {
                double num = Double.parseDouble(str);
                num = floor(num * 100) / 100;
                str = String.valueOf(num);
              } catch (NumberFormatException e) {
                log.debug(str + " not a double.");
              }
            }
            NPPhraseSpec n = nlgFactory.createNounPhrase(pronounize(str).replace("_", " "));
            //add modifiers
            for (int i = 1; i < phrase.length; i++) {
              n.addPostModifier(generateNP(new Symbol(phrase[i])));
            }
            np = n;
          }
        }
      }

      //brad:if it hasn't been explicitly set to be plural check if it should be.
      if(!np.isPlural()) {
        np.setPlural(isPlural(s));
      }
      np.setFeature(Feature.PRONOMINAL,false);
      return np;
    }
  }

  private String generateDeterminer(Variable v) {
    Symbol ta = tierAssignments.get(v);
    log.debug("getting determiner for: "+v+ " determiner: "+ ta);
    if(ta != null) {
      switch (ta.getName()) {
        case "INDEFINITE":
          return "a";
        case "DEFINITE":
          return "the";
        case "FAMILIAR":
          return "that";
        case "REFACTIVATED":
          return "this";
      }
    }
//    case "ACTIVATED" => ACTIVATED
    return "";
  }

  private static NPPhraseSpec generateREG(Symbol s) {

    //TODO:brad: update this to use supplemental semantics instead of making a call to REG
    log.debug("in generateREG  ");
    Map<Symbol,List<Term>> refs = new HashMap<>();
    try {
      refs = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateRE").argTypes(Symbol.class)).call(Map.class, s);
    } catch (TRADEException e) {
      log.warn("[generateREG]",e);
    }
    if (refs == null || refs.isEmpty()) {
      return nlgFactory.createNounPhrase("it");
    } else {
      if (refs.keySet().iterator().next() == null ||refs.get(refs.keySet().iterator().next()).isEmpty()) {
        return nlgFactory.createNounPhrase("it");
      }

      return handleREGoutput(refs.keySet().iterator().next(), refs);
    }
  }

  private static NPPhraseSpec handleREGoutput(Symbol v, Map<Symbol, List<Term>> el) {
    //takes variable type v (reference) and map el (results of resolution)
    List<Term> upreds = new ArrayList<>();
    List<Term> bpreds = new ArrayList<>();
    //upreds contains preds with 1 argument, bpreds have multiple arguments

    for(Term p: el.get(v)){
      //get args of each property and add the head to predicate lists
      if (p.size() == 1) {
        upreds.add(p);
      } else {
        bpreds.add(p);
      }
    }

    NPPhraseSpec np;
    if (upreds.size() == 0) {
      np = nlgFactory.createNounPhrase("thing");
    } else {
      np = nlgFactory.createNounPhrase(upreds.get(0).getName());
    }

    //for each additional pred, add it as a modifier (probably for adjectives)
    for (int i = 1; i < upreds.size(); i++) {
      String str = upreds.get(i).getName();
      np.addModifier(str.substring(0, str.length()));
    }
    //for each predicate with multiple arguments, add it as a prepositional phrase
    for (Term bp : bpreds) {
      if (bp.getArgs().get(1).isVariable()) {
        Variable var = (Variable) bp.getArgs().get(1);
        //TODO:brad: this creation of a new symbol from var.getType is less than ideal
        np.addModifier(nlgFactory.createPrepositionPhrase(bp.getName(), handleREGoutput(new Symbol(var.getType()), el)));
      } else if (bp.getArgs().get(1) != null) {
        Symbol s = bp.getArgs().get(1);
        if (s.getName().contains("_")) {
          np.addModifier(nlgFactory.createPrepositionPhrase(bp.getName(),generateREG(s)));
        } else
        //TODO:brad: move this out of the java into a prag rule?
        if(bp.getName().equals("side") || bp.getName().equals("wingid")) {
          np.addModifier(nlgFactory.createPrepositionPhrase(s.getName()));
        } else {
          np.addModifier(nlgFactory.createPrepositionPhrase(bp.getName(), s.getName()));
        }
      }
    }

    //set "the" for refs that require an article
    if ( !realiser.realise(np).equals("it") ) { np.setSpecifier("the"); }
    return np;
  }

//    public static String getAdverbModifiers(Utterance utterance) {
//    	//returns adverbs from utterance adverb arg
//    	if (!utterance.adverbs.isEmpty()){
//			Iterator<Symbol> advIt = utterance.adverbs.iterator();
//			StringBuilder adverbStr = new StringBuilder();
//			String adv;
//			while (advIt.hasNext()){
//				adv = advIt.next().toString();
//				adverbStr.append(" " + adv);
//			}
//			return adverbStr.toString();
//		}
//		return "";
//	}

  private InterrogativeType getQuestionType(Utterance utterance) {

    if (utterance.getType() == UtteranceType.QUESTION) {
      if (utterance.getSemantics().toString().contains("needsLocation"))
        return InterrogativeType.WHERE;
      if (utterance.getSemantics().toString().contains("itk"))
        return InterrogativeType.YES_NO;
      if (utterance.getSemantics().toString().contains("where"))
        return InterrogativeType.WHERE;
      else
        return InterrogativeType.WHAT_OBJECT;
    }

    return null;
  }

  /**
   * A coordinated phrase has two predicates inside it.
   * A tense, modal, or aspectual phrase has one predicate inside it.
   * Verbs have arguments inside them.
   */
  private SPhraseSpec buildPhrase(Term semantics) {
    SPhraseSpec clause = nlgFactory.createClause();
    log.debug("Building phrase for :" + semantics);
    String predicateName = semantics.getName();

    /**************************************/
    /**         Clause Wrappers          **/
    /**************************************/

    if (predicateName.equals("not")) {
      Symbol symbol = semantics.get(0);
      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        argument.setFeature(Feature.NEGATED, true);
        //argument.setFeature(Feature.MODAL,"do"); --> not necessary as "do" is automatically added
        return argument;
      }
      return null;
    } else if (predicateName.equals("greeting")) {
      NLGElement addressee = nlgFactory.createNounPhrase(getIndirectObject(semantics.getArgs()).getName());
      NLGElement phrase = generateNP(getObject(semantics.getArgs()));
      clause.setSubject(phrase);
      clause.setObject(addressee);
      return clause;
    } else if (predicateName.equals("ack")) {
      List<Symbol> args = semantics.getArgs();
      if (args.get(0).isTerm()) {
        clause.setSubject(buildPhrase((Term) args.get(0)));
      } else {
        clause.setSubject(generateNP(args.get(0)));
      }
      args.remove(0);
      for (Symbol symbol : args) {
        SPhraseSpec argument = nlgFactory.createClause();
        if (symbol.isTerm()) {
          argument = buildPhrase((Term) symbol);
        } else {
          argument.setSubject(generateNP(symbol));
        }
        argument.setFeature(Feature.APPOSITIVE,true);
        clause.setPostModifier(argument);
      }
      return clause;
    } else if (predicateName.equals("progressive")) {
      Symbol symbol = semantics.get(0);
      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        argument.setFeature(Feature.PROGRESSIVE, true);
        return argument;
      }
      return null;
    } else if (predicateName.equals("perfect")) {
      Symbol symbol = semantics.get(0);
      //clause.setSubject(predicateName);
      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        argument.setFeature(Feature.PERFECT, true);
        return argument;
      }
      return null;
    } else if (predicateName.equals("where")) {
      Symbol symbol = semantics.get(0);
      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        argument.setFeature(Feature.INTERROGATIVE_TYPE, InterrogativeType.WHERE);
        return argument;
      }
      return null;
    } else if (predicateName.equals("but")) {
      Symbol symbol = semantics.get(0);
      SPhraseSpec phrase = buildPhrase((Term) symbol);
      SPhraseSpec p = nlgFactory.createClause("", "", "");
      phrase.setFeature(Feature.COMPLEMENTISER, "but");
      p.addComplement(phrase);
      return clause;
    } else if (predicateName.equals("tell")) {
      Symbol symbol = semantics.get(1);
      VPPhraseSpec v = nlgFactory.createVerbPhrase(predicateName);
      clause.setVerbPhrase(v);
      clause.setSubject(generateNP(semantics.get(0)));
      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        //"i cannot tell shafer to walk forward"
        NLGElement s = argument.getSubject();
        clause.setIndirectObject(s);
        argument.setFeature(Feature.FORM, Form.INFINITIVE);
        clause.setObject(argument);
        return clause;
      }
      return null;
    } else if (predicateName.equals("cannot") || predicateName.equals("willnot") || predicateName.equals("shouldnot")) {
      Symbol symbol = semantics.get(0);
      //NPPhraseSpec subject = generateNP(getSubject(semantics.getArgs()));
      //clause.setSubject(subject);
      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        clause.setVerbPhrase(argument);
        if (semantics.size() > 1 && semantics.get(1).isTerm()) {
          NLGElement explanation = buildPhrase((Term) semantics.get(1));
          if (explanation!=null) {
            clause.addComplement(explanation);
          }
        }

        String featureValue = predicateName.substring(0, predicateName.indexOf("not"));
        clause.setFeature(Feature.MODAL, featureValue); //--> not necessary as "do" is automatically added
        clause.setFeature(Feature.NEGATED, true);
        return clause;
      }
      return null;
      } else if (predicateName.equals("failed")) { //TEMP
      Symbol symbol = semantics.get(0);

      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        clause.setVerbPhrase(argument);
        if (semantics.size() > 1 && semantics.get(1).isTerm()) {
          NLGElement explanation = buildPhrase((Term) semantics.get(1));
          if (explanation != null) {
            clause.addComplement(explanation);
          }
        }

        String featureValue = predicateName;
        clause.setFeature(Feature.MODAL, featureValue);
        return clause;
      }
      return null;
      } else if (predicateName.equals("because")) {
        List<Symbol> args = semantics.getArgs();
        log.debug("because args = "+args);
        if (!args.isEmpty()) {
          log.debug("args of because are: "+args);
          CoordinatedPhraseElement c = nlgFactory.createCoordinatedPhrase();
          for (Symbol arg : args) {
            if (arg.isTerm()) {
              NLGElement argument = buildPhrase((Term) arg);
              c.addCoordinate(argument);
            } else {
              c.addCoordinate(arg.getName());
            }
          }
          SPhraseSpec sp = nlgFactory.createClause("", "");
          sp.setVerb(c);

          sp.setFeature(Feature.COMPLEMENTISER, "because");

          return sp;
        }
        return null;
    } else if (predicateName.equals("did") || predicateName.equals("gerund")) {
      Symbol symbol = semantics.get(0);
      if (symbol.isTerm() && symbol.hasArgs()) {
        List<Symbol> args = ((Term) symbol).getArgs();
//        args.add(0, semantics.get(0));
        SPhraseSpec argument = buildPhrase(new Term(symbol.getName(), args));
        return argument;
      } else {
        NPPhraseSpec subject = (NPPhraseSpec) generateNP(getSubject(semantics.getArgs()));
        clause.setSubject(subject);
        clause.setVerb(verbalize(symbol.getName()));
        return clause;
      }

    } else if (predicateName.equals("itk")) {
      Symbol symbol = semantics.get(1);
      //clause.setVerb("mean");
      //clause.setSubject("you");
      //clause.setFeature(Feature.PERSON, Person.SECOND);
      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        NPPhraseSpec np = nlgFactory.createNounPhrase(argument);
        //clause.setObject(np);
        //argument.setFeature(Feature.MODAL,"do"); //--> not necessary as "do" is automatically added
        argument.setFeature(Feature.INTERROGATIVE_TYPE,InterrogativeType.YES_NO);
        return argument;
      } else {
        clause.setObject(generateNP(symbol));
        return clause;
      }
    } else if (predicateName.equals("actionDescription")) {
      NLGElement vp = buildPhrase((Term) semantics.get(0));
      //vp.setFeature(Feature.TENSE, Tense.FUTURE);
      vp.setFeature(Feature.FORM, Form.INFINITIVE);
      Symbol symbol = semantics.get(1);
      if (symbol.isVariable()) {
        if (bindings.isEmpty()) {
          VPPhraseSpec v = nlgFactory.createVerbPhrase(getVariableReplacement((Variable) symbol));
          clause.setVerbPhrase(v);
          clause.setFeature(Feature.PERSON, Person.SECOND);
          clause.addFrontModifier(vp);
          return clause;
        }
        CoordinatedPhraseElement c = nlgFactory.createCoordinatedPhrase();
        for (Map<Variable, Symbol> binding : bindings) {
          Symbol s = PragUtil.fillSymbol(binding, symbol);
          boolean first= true;
          if (s.getName() == "step" || s.isTerm()) {
            List<Symbol> args = ((Term) s).getArgs();
            for (Symbol step : args) {
              SPhraseSpec argument = buildPhrase((Term) step);
              if(!first) {
                argument.addModifier("then");
              } else {
                first = false;
              }
              c.addCoordinate(argument);
            }
          }
        }
        clause.setVerbPhrase(c);
        clause.addFrontModifier(vp);
        return clause;
      } else if (symbol.getName().equals("steps")) {
        CoordinatedPhraseElement c = nlgFactory.createCoordinatedPhrase();
        boolean first= true;
        List<Symbol> args = ((Term)symbol).getArgs();
        for (Symbol step : args) {
          SPhraseSpec argument = buildPhrase((Term) step);
          if(!first) {
            argument.addModifier("then");
          } else {
            first = false;
          }
          c.addCoordinate(argument);
        }
        clause.setVerbPhrase(c);
        clause.addFrontModifier(vp);
        return clause;
      }
      return null;
    } else if (predicateName.equals("currently")) {
      SPhraseSpec argument = buildPhrase((Term) semantics.get(0));
      argument.addFrontModifier("currently");
      return argument;
    } else if (predicateName.equals("past")) {
      Symbol symbol = semantics.get(0);
      if (symbol.isTerm() && ((Term) symbol).hasArgs()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        argument.setFeature(Feature.TENSE, Tense.PAST);
        return argument;
      }
      return null;
    } else if (predicateName.equals("probabilityOf")) {
      //the probability that i X is Y
      List<Symbol> args = semantics.getArgs();
      log.debug("NLG HERE: "+args);
      //NLGElement s = generateNP(Factory.createPredicate("that",args.get(0)));
      NPPhraseSpec sub = (NPPhraseSpec) generateNP(new Symbol("theProbability"));
      Symbol math;
      //if (args.size()>1) {
      //  NLGElement s = generateNP(Util.createPredicate("that", args.get(0)));
      //  sub.addPostModifier(s);
      //  math = args.get(1);
      //} else {
        math = args.get(0);
      //}
      clause.setSubject(sub);
      clause.setVerb("is");
      log.debug("NLG HERE prior to math");
      log.debug("math = "+math.getName());
      NLGElement o = generateNP(math);
      clause.setObject(o);
      return clause;
    } else if (predicateName.toLowerCase().endsWith("durationof")) {
      //the duration of X is Y seconds
      List<Symbol> args = semantics.getArgs();
      log.debug("NLG HERE: "+args);
      NLGElement s = generateNP(args.get(0));
      NPPhraseSpec sub = (NPPhraseSpec) generateNP(new Symbol(predicateName));
      sub.addPostModifier(s);
      clause.setSubject(sub);
      clause.setVerb("is");
      log.debug("NLG HERE prior to math");
      log.debug("math = "+args.get(1).getName());
      NLGElement o = generateNP(args.get(1));
      clause.setObject(o);
      clause.addPostModifier(generateNP(new Symbol("seconds")));
      return clause;
    } else if (predicateName.equals("can") || predicateName.equals("should") || predicateName.equals("might") || predicateName.equals("could") || predicateName.equals("would")) {
      Symbol symbol = semantics.get(0);
      if (symbol.isTerm() && ((Term) symbol).hasArgs()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        argument.setFeature(Feature.MODAL, predicateName);
        return argument;
      }
      return null;
    } else if (predicateName.equals("contextDescription")) {
      //stepOf(LOC,did(B,ACT)),step
      Symbol s = semantics.get(0);
      NPPhraseSpec sub = (NPPhraseSpec) generateNP(new Symbol(s.getName()));
      sub.setDeterminer("the");
      sub.addPreModifier(generateNP(((Term) s).get(0)));
      Symbol action = ((Term)s).get(1); //did(B,ACT)
      sub.addPostModifier(generateNP(((Term)action)));//.get(1)));
      Symbol actor = ((Term)action).get(0);
      if (!actor.getName().equals(speaker)) {
        log.debug(speaker+" : "+actor);
        sub.addPostModifier("for "+actor.getName());
      }
      clause.setSubject(sub);
      clause.setVerb("is");
      Symbol step = ((Term) semantics).get(1);
      if (step.getName().equals("step")) {
        Symbol sym = ((Term) step).get(0);
        if (sym.isVariable()) {
          CoordinatedPhraseElement c = nlgFactory.createCoordinatedPhrase();
          if (bindings.isEmpty()) {
            NLGElement np = generateNP(sym);
            c.addCoordinate(np);
          }
          for (Map<Variable, Symbol> binding : bindings) {
            NLGElement np = buildPhrase((Term)PragUtil.fillSymbol(binding, sym));
            c.addCoordinate(np);
          }
          clause.addComplement(c);
          clause.setFeature(Feature.COMPLEMENTISER,"that");
        }
      }
      return clause;
    } else if (predicateName.equals("will")) {
      /*Symbol symbol = semantics.get(0);
      if (symbol.isTerm() && ((Term) symbol).hasArgs()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        argument.setFeature(Feature.TENSE, Tense.FUTURE);
        return argument;
      */
      //will(X,verb(Y)) -> X will verb Y
      Symbol symbol = semantics.get(1);
      if (symbol.isTerm() && ((Term) symbol).hasArgs()) {
        List<Symbol> args = ((Term) symbol).getArgs();
        args.add(0,semantics.get(0));
        log.debug("will: "+symbol.getName()+" "+args);
        SPhraseSpec argument = buildPhrase(new Term(symbol.getName(),args));
        clause.setVerbPhrase(argument);
        clause.setFeature(Feature.TENSE, Tense.FUTURE);
        //argument.setFeature(Feature.MODAL,"do"); --> not necessary as "do" is automatically added
        return clause;
      }
      return null;
    } else if (predicateName.equals("how")) {
      List<Symbol> args = semantics.getArgs();
      //clause.setSubject(generateNP(getSubject(args)));
      NLGElement argument;
      if (args.size() > 1) {
        Symbol symbol = semantics.get(1);
        if (symbol.isTerm()) {
          argument = buildPhrase((Term) symbol);
        } else {
          argument = nlgFactory.createVerbPhrase(verbalize(symbol.getName()));
        }
        clause.setComplement(argument);
        clause.setFeature(Feature.COMPLEMENTISER, "how");
        return clause;
      } else {
        //"how to"
        Symbol symbol = semantics.get(0);
        if (symbol.isTerm()) {
          argument = buildPhrase((Term) symbol);
        } else {
          argument = nlgFactory.createVerbPhrase(verbalize(symbol.getName()));
        }
        argument.setFeature(Feature.FORM, Form.INFINITIVE);
        clause.setComplement(argument);
        clause.setFeature(Feature.COMPLEMENTISER, "how");
        return clause;
      }
    } else if (predicateName.equals("that")) {
      NLGElement argument;
      Symbol symbol = semantics.get(0);
      if (symbol.isTerm()) {
        argument = buildPhrase((Term) symbol);
        clause.setComplement(argument);
        clause.setFeature(Feature.COMPLEMENTISER, "that");
      }
      return clause;
    } else if (predicateName.equals("wouldlike")) {
      Symbol target = pronounize(semantics.get(1));
      //log.debug(semantics);
      //log.debug("target: "+target);
      Symbol desire = semantics.get(1);
      Term wouldlike = new Term("like", target, desire);
      SPhraseSpec argument = buildPhrase(wouldlike);
      //log.debug("argument: "+argument);
      if (argument instanceof SPhraseSpec) {
        if (desire.getName().equals("bring")) {
          ((SPhraseSpec) argument).setIndirectObject("me");
        }
        //log.debug("object: "+((SPhraseSpec)argument).getObject());
        //log.debug("indirect object: "+((SPhraseSpec)argument).getIndirectObject());
      }
      argument.setFeature(Feature.MODAL, "would");
      return argument;
    } else if (predicateName.equals("report")) {
      Symbol symbol = semantics.get(1);
      if (symbol.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) symbol);
        argument.setFeature(Feature.MODAL, "do");
        return argument;
      }
      return null;
    } else if (predicateName.equals("pre")) {
      Symbol s = semantics.get(0);
      if (semantics.get(1).isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) semantics.get(1));
        clause.setVerb(argument);
        if (s.isTerm()) {
          SPhraseSpec pre = buildPhrase((Term) s);
          clause.addPreModifier(pre);
          return clause;
        }
        clause.addPreModifier(generateNP(s));
        return clause;
      }
      return null;
    } else if (predicateName.equals("if")) {
      Symbol s = semantics.get(0);
      if(s.isTerm()) {
        SPhraseSpec condition = buildPhrase((Term) s);
        condition.setFeature(Feature.COMPLEMENTISER, "if");
        clause.setComplement(condition);
      }
      if(semantics.getArgs().size()>1){
        if(semantics.get(1).isTerm()) {
          SPhraseSpec argument = buildPhrase((Term) semantics.get(1));
          clause.setVerb(argument);
        }
      }
      return clause;
    } else if (predicateName.equals("infinitive")) {
      Symbol s = semantics.get(0);
      if (s.isTerm()) {
        List<Symbol> args = ((Term)s).getArgs();
        args.add(0,new Symbol("placeholder"));
        SPhraseSpec argument = buildPhrase(new Predicate(s.getName(),args));
        argument.setFeature(Feature.FORM,Form.INFINITIVE);
        return argument;
      }
      return null;
    } else if (predicateName.equals("must")) {
      Symbol s = semantics.get(0);
      if (s.isTerm()) {
        SPhraseSpec argument = buildPhrase((Term) s);
        argument.setFeature(Feature.MODAL,"must");
        return argument;
      }
      return null;

    } else if (predicateName.equals("possessive")) {
      NLGElement n = generateNP(semantics);
      clause.setSubject(n);
      return clause;

    } else if (predicateName.startsWith("grasp") && semantics.getArgs().size()>2) {
      NPPhraseSpec n = (NPPhraseSpec) generateNP(semantics.get(2));
      List<Symbol> args = semantics.getArgs();
      args.remove(2);
      args.remove(0);
      args.add(0, new Symbol(realiser.realise(setPossessives(semantics.get(0), n)).getRealisation()));
      Term t = new Predicate(predicateName, args);
      SPhraseSpec argument = buildPhrase(t);
      return argument;
    } else if (predicateName.equals("currentTime")) {
      List<Symbol> args = semantics.getArgs();
      clause.setSubject("it");
      clause.setVerb("is");
      clause.setObject(generateNP(args.get(0)));
      clause.addPostModifier(generateNP(args.get(1)));
      return clause;
    } else if (predicateName.equals("and") ||
        predicateName.equals("or")) {
      CoordinatedPhraseElement coordination = nlgFactory.createCoordinatedPhrase();
      List<Symbol> args = semantics.getArgs();
      for (Symbol sym : args) {
        NLGElement arg = generateNP(sym);
        coordination.addCoordinate(arg);
      }
      coordination.setConjunction(predicateName);
      clause.setObject(coordination);

      return clause;
      /**************************************/
      //TODO:brad: move this out of the java into a prag rule?
    } else if (predicateName.equals("wingid") || predicateName.equals("side")){
      clause.setObject(semantics.get(0));
      return clause;
    } else {
      //log.debug("IN ELSE");
      /* Replace with pronouns anything that should be pronounized **/
      ArrayList<Symbol> arguments = new ArrayList<Symbol>();
      for (int i = 0; i < semantics.size(); i++) {
        Symbol arg = semantics.get(i);
        arguments.add(pronounize(arg));
      }
      //log.debug("arguments: "+arguments);

      /* the first argument is the subject **/
      //e.g., know(*self*, fact);

      NLGElement subject;
      Symbol sub = getSubject(arguments);
      if (sub.isVariable()) {
        CoordinatedPhraseElement c = nlgFactory.createCoordinatedPhrase();
        if (bindings.isEmpty()) {
          NLGElement np = generateNP(sub);
          c.addCoordinate(np);
        }
        for (Map<Variable, Symbol> binding : bindings) {
          Symbol s = PragUtil.fillSymbol(binding, new Variable("WHAT"));
          NLGElement np = generateNP(s);
          c.addCoordinate(np);
        }
        subject = c;
      } else {
        subject = generateNP(sub);
      }

      //log.debug("Subject: "+subject);
      /* The name of the predicate is the verb **/
      //e.g., *know*(self, fact);
      String[] verbParts = predicateName.split("_");
      String preposition = verbParts.length > 1 ? verbParts[1] : "";
      String verb = verbalize(verbParts[0]);
      log.debug("verb: " + verb);

      if (verb.equals("locationof") && (arguments.size() > 0)) {
        if (!arguments.get(0).toString().equals(arguments.get(0).toString().toLowerCase())) {//contains("WHERE")){
          //if(arguments.get(0) instanceof edu.tufts.hrilab.fol.Variable){
          //log.debug("it was isloc");
          verb = "to_find";
          clause.setFeature(Feature.COMPLEMENTISER, "");
          clause.setFeature(Feature.MODAL, "");
          clause.setFeature(Feature.INTERROGATIVE_TYPE, InterrogativeType.WHERE);
        } else {
          return buildPhrase(new Term("is located in", arguments.get(1), arguments.get(0)));
        }
      }

      clause.setVerb(verb);
      if (verb.endsWith("ing") && !verb.matches("bring") && !verb.matches("sing")) {
        clause.setFeature(Feature.PROGRESSIVE, true);
      }

      if (!subject.toString().equals("")) {
        clause.setSubject(subject);
      }

      Symbol obj = getObject(arguments);

      NLGElement object;
      if (obj.isVariable()) {
        CoordinatedPhraseElement c = nlgFactory.createCoordinatedPhrase();
        if (bindings.isEmpty()) {
          NLGElement np = generateNP(obj);
          c.addCoordinate(np);
        }
        for (Map<Variable, Symbol> binding : bindings) {
          Symbol s = PragUtil.fillSymbol(binding, obj);
          NLGElement np = generateNP(s);
          np = setPossessives(sub, np);
          c.addCoordinate(np);
        }
        object = c;
        clause.setObject(object);
      } else {
        //TODO:brad: move this out of the java into a prag rule?
        if(obj.getName().equals("psi")){
          obj= Factory.createSymbol(((Predicate)obj).getArgs().get(0)+obj.getName());
        }
        NLGElement np = generateNP(obj);
        if (!sub.getName().equals("")) {
          object = setPossessives(sub, np);
        } else {
          object = np;
        }

        log.debug("CURRENT OBJECT: " + realiser.realise(object));

        if (!object.toString().equals("")) {
          if (obj.isTerm()) {
            //Term phraseTerm = (Term) arguments.get(1);

            if (verb.equals("say")) {
              NLGElement phrase = buildPhrase((Term) obj);
              clause.setFeature(Feature.TENSE, Tense.PAST);
              //NLGElement phrase = buildPhrase((Term) arguments.get(1));
              clause.addComplement(phrase);
              //return clause;
            } else if (verb.endsWith("That")) {
              //forgetThat(X)
              NLGElement phrase = buildPhrase((Term) obj);
              log.debug("Realized phrase: " + realiser.realise(phrase));
              phrase.setFeature(Feature.COMPLEMENTISER, "");
              clause.addComplement(phrase);
            } else if (obj.getName().equals("meaningOf")) {
              NLGElement phrase = buildPhrase((Term) obj);
              //translate as "what X means" or "X means this" depending on # of args
              if (((Term) obj).getArgs().size() == 1) {
                //SPhraseSpec ne = nlgFactory.createClause(object,"");
                phrase.setFeature(Feature.COMPLEMENTISER, "what");
              }
              clause.addComplement(phrase);
              //} else if (obj.getName().equals("that")) {
              //} else if (verb.equals("know") || verb.equals("describe")) {
              //  phrase.setFeature(Feature.COMPLEMENTISER, "that");
            } else if (obj.getName().equals("how")) {
              NLGElement phrase = buildPhrase((Term) obj);
              phrase.setFeature(Feature.COMPLEMENTISER, "how");
              clause.addComplement(phrase);
            } else if (verb.equals("want")) {
              NLGElement phrase = buildPhrase((Term) obj);
              SPhraseSpec sphrase = (SPhraseSpec) phrase;
              NLGElement s = sphrase.getSubject();
              NLGElement v = sphrase.getVerbPhrase();
              v.setFeature(Feature.FORM,Form.INFINITIVE);
              clause.setObject(s);
              sphrase.setVerbPhrase(v);
              sphrase.setFeature(Feature.COMPLEMENTISER, "");
              clause.addComplement(sphrase);
            } else {
              clause.setObject(object);
//              phrase.setFeature(Feature.COMPLEMENTISER, "");
//              clause.addComplement(phrase);
            }
            //SPhraseSpec ne = nlgFactory.createClause("I","help");
            //ne.setFeature(Feature.COMPLEMENTISER, "test");
            //clause.addComplement(phrase);

            log.debug("Realized object: " + realiser.realise(object));
            log.debug("Realized clause: " + realiser.realise(clause));
          } else {
            clause.setObject(object);
            log.debug("subj: " + clause.getSubject() + ", obj: " + clause.getObject());
          }
        }
      }

      //if there is a preposition, create prepositional phrase
      if (!preposition.isEmpty()) {
        NLGElement iobject = generateNP(pronounize(getIndirectObject(arguments)));
        PPPhraseSpec pp = nlgFactory.createPrepositionPhrase();
        pp.addComplement(iobject);
        pp.setPreposition(preposition);
        clause.addComplement(pp);
      } else {
        NLGElement iobj = generateNP(pronounize(getIndirectObject(arguments)));
        clause.setIndirectObject(iobj);
      }
      clause.addComplement(getPrepositionalPhrase(arguments));
      return clause;
    }
  }

  private String getVariableReplacement(Variable v) {
    if (v.getName().equals("WHO")) {
      return "nobody";
    } else if (v.getName().equals("HOW")) {
      return "do nothing";
    } else if (v.getName().equals("X")) {
      return "nothing";
    } else if (v.getName().equals("VAR0")) {
      return "it";
    }
    return "nothing";
  }

  private boolean isPlural(Symbol s) {
    WordElement word = lexicon.getWordFromVariant(pronounize(s).getName());
    if (!word.getCategory().equalTo(LexicalCategory.VERB)) {
      InflectedWordElement plural = new InflectedWordElement(word);
      plural.setFeature(Feature.NUMBER, NumberAgreement.PLURAL);
      String pluralstr = realiser.realise(plural).getRealisation();
      String nonplural = realiser.realise(word).getRealisation();
      //if the plural form matches the string and the nonplural form doesn't match the string
      if (pluralstr.equals(s.getName()) && !nonplural.equals(s.getName())) {
        return true;
      }
    }
    return false;
  }

  private Symbol pronounize(Symbol arg) {
    if (arg.isTerm()) {
      return arg;
    } else if (arg.isVariable()) {
      //returns Variable instead of Symbol to keep variable binding functionality
      return new Variable(pronounize((arg.getName().equals("") ? ((Variable) arg).getType() : arg.getName())));
    }
    //Previously we were discarding type information by calling getName which was causing reference resolution to fail
    // bc refs are now stored including type information.
//    return new Symbol(pronounize(arg.getName()));
    if (arg.getType().equals("")) {
      return new Symbol(pronounize(arg.getName()));
    } else {
      return new Symbol(pronounize(arg.getName()) + ":" + arg.getType());
    }
  }

  private String pronounize(String arg) {
    if (arg.equalsIgnoreCase("self") || speaker.equals(arg)) {
      arg = "I";
    } else if (arg.equalsIgnoreCase("listener") || listeners.contains(arg)) {
      arg = "you";
    } else if (arg.contains("cmdr")) {
      //arg = "him";
    }
    return arg;
  }

  private String verbalize(String verb) {
    if (verb.equals("state")) {
      verb = "is";
    } else if (verb.equals("at")) {
      verb = "is at";
    } else if (verb.equals("needsVisionType")) {
      verb = "look like";
    } else if (verb.equals("needsLocation")) {
      verb = "is";
    } else if (verb.equals("knowHow")) {
      verb = "know how";
    } else if (verb.startsWith("doa")) {
      verb = "do a " + verb.substring(3,verb.length());
    } else if (verb.endsWith("Object")) {
      verb = verb.substring(0,verb.length()-6);
    } else if (verb.equals("meaningOf")) {
      verb = "means";
    } else if (verb.equals("propertyOf")) {
      verb = "is";
    } else if (verb.equals("shakeyourhead")) {
      verb = "shake my head";
    } else {
      verb = verb.replaceAll("([A-Z])", " $1");
    }

    return verb;
  }

  private NPPhraseSpec setPossessives(Symbol owner, NLGElement object) {
    NPPhraseSpec newObj = nlgFactory.createNounPhrase(object);
    log.debug(realiser.realise(object).getRealisation());
    List<String> parts = new ArrayList<String>(Arrays.asList("arms","leftarm","rightarm","head"));
    if (parts.contains(realiser.realise(object).getRealisation().replace(" ",""))) {
      //owner needs to be realised to prevent the owner from also being made possessive
      NLGElement pn = nlgFactory.createInflectedWord(generateNP(owner), LexicalCategory.PRONOUN);
      log.debug(owner +" - "+realiser.realise(pn));
      pn.setFeature(Feature.POSSESSIVE, true);
      pn.setFeature(Feature.PRONOMINAL, true);
      log.debug("Realized pn: " + realiser.realise(pn));
      if (pn.isPlural()) {
        pn.setFeature(Feature.NUMBER, NumberAgreement.PLURAL);
      } else {
        pn.setFeature(Feature.NUMBER, NumberAgreement.SINGULAR);
      }
      newObj.setSpecifier(pn);
    }
    return newObj;
  }

  private Symbol getSubject(List<Symbol> arguments) {
    //if there is only one argument, it's an subject and not an object
    if (arguments != null && arguments.size() > 0) {
      return arguments.get(0);
    }
    return new Symbol("");
  }

  private Symbol getObject(List<Symbol> arguments) {
    switch (arguments.size()) {
      case 4:
        return arguments.get(1);
      case 3:
        return arguments.get(2);
      case 2:
        return arguments.get(1);
    }
    return new Symbol("");
  }

  private Symbol getIndirectObject(List<Symbol> arguments) {
    switch (arguments.size()) {
      case 3:
        return arguments.get(1);
    }
    return new Symbol("");
  }

  private PPPhraseSpec getPrepositionalPhrase(List<Symbol> arguments) {
    switch (arguments.size()) {
      case 4:
        PPPhraseSpec pp = nlgFactory.createPrepositionPhrase();
        pp.addComplement(generateNP(pronounize(arguments.get(3))));
        pp.setPreposition(generateNP(pronounize(arguments.get(2))));
        return pp;
    }
    return nlgFactory.createPrepositionPhrase();
  }
}
