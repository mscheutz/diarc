package edu.tufts.hrilab.slug.parsing.tldl;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.slug.common.Utterance;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class ParseReasoning {
  private static Logger log = LoggerFactory.getLogger(ParseReasoning.class);

  /**
   * Prunes a list of candidate parses using Prolog-based belief reasoning.
   *
   * @param candidateParses List of possible Utterance interpretations.
   * @return Filtered list of valid Utterances after belief reasoning.
   */
  public static List<Utterance> pruneParsesWithBelief(List<Utterance> candidateParses) {
    List<Utterance> filteredParses = new ArrayList<>();

    if (candidateParses.isEmpty()) {
      log.warn("[pruneParsesWithBelief] No candidate parses to evaluate.");
      return filteredParses;
    }

    log.debug("[pruneParsesWithBelief] Evaluating {} candidate parses.", candidateParses.size());

    // Query each parse individually
    for (Utterance parse : candidateParses) {
      Predicate query = generateQueryFromParse(parse);
      if (query == null) {
        log.warn("[pruneParsesWithBelief] Skipping null query for parse: {}", parse);
        continue;
      }

      List<Map<Variable, Symbol>> beliefResults = new ArrayList<>();

      // try {
      //   // Query the belief system with a single parse (using Predicate)
      //   beliefResults = TRADE.getAvailableService(
      //           new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)
      //   ).call(List.class, query);
      // } catch (TRADEException e) {
      //   log.warn("Could not query beliefs for parse: {}", parse, e);
      //   continue;
      // }

      // Apply reasoning to determine if the parse should be kept
      if (isValidParse(parse, beliefResults)) {
        filteredParses.add(parse);
      } else {
        log.debug("[pruneParsesWithBelief] Pruning parse: {}", parse);
      }
    }

    log.info("[pruneParsesWithBelief] Returning {} filtered parses.", filteredParses.size());
    return filteredParses;
  }

  /**
   * Converts an Utterance into a Prolog query Predicate.
   *
   * @param parse The Utterance object representing a parse.
   * @return A Predicate representing the Prolog query.
   */
  private static Predicate generateQueryFromParse(Utterance parse) {
    if (parse == null || parse.getSemantics() == null) {
      return null;
    }
    // Construct a Prolog-style query as a Predicate
    return Factory.createPredicate("query_parse", "X", parse.getSemantics().toString());
  }

  /**
   * Determines if a parse is valid based on Prolog belief reasoning.
   *
   * @param parse         The candidate Utterance.
   * @param beliefResults The corresponding Prolog query results.
   * @return True if the parse is valid, False otherwise.
   */
  private static boolean isValidParse(Utterance parse, List<Map<Variable, Symbol>> beliefResults) {
    if (beliefResults == null || beliefResults.isEmpty()) {
      return false; // No supporting belief means it's likely invalid.
    }

    // Example: Apply a custom filtering rule based on belief results.
    for (Map<Variable, Symbol> binding : beliefResults) {
      Symbol beliefAmount = binding.get(Factory.createVariable("beliefAmount"));
      Symbol worldAmount = binding.get(Factory.createVariable("worldAmount"));

      if (beliefAmount != null && worldAmount != null) {
        // Compute the belief difference
        Symbol difference = Factory.createSymbol(String.valueOf(
                Integer.parseInt(beliefAmount.getName()) - Integer.parseInt(worldAmount.getName())
        ));

        // Example: Keep parses where belief and world state are consistent
        if (Integer.parseInt(difference.getName()) >= 0) {
          return true;
        }
      }
    }

    return false;
  }
}
