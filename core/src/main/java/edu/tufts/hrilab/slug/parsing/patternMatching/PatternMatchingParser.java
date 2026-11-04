/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.patternMatching;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.interfaces.NLUInterface;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;


public class PatternMatchingParser extends DiarcComponent implements NLUInterface {
    final protected Logger log = LoggerFactory.getLogger(this.getClass());

    private final Map<String,Class<? extends UtterancePattern>> knownPatterns= new HashMap<>();
    private final List<Class<? extends UtterancePattern>> activePatterns= new CopyOnWriteArrayList<>();


    /**
     *
     */
    abstract static class UtterancePattern {
        final protected Logger log = LoggerFactory.getLogger(this.getClass());
        public UtterancePattern getInstance(){
            return this;
        }
        /**
         * @param pattern
         * @return
         */
         public abstract Utterance matches(Utterance pattern);

    }

    /**
     *
     */
    static class IntegerPattern extends UtterancePattern {
        /**
         * @param u
         * @return
         */
        @Override
        public Utterance matches(Utterance u) {
            if ( u.getWordsAsString().matches("[-+]?\\d*\\.?\\d+[fFlLdD]?")) {
                Utterance.Builder builder= new Utterance.Builder(u);
                builder.setSemantics(Factory.createPredicate("val",Factory.createSymbol(u.getWordsAsString())));
                builder.setUtteranceType(UtteranceType.REPLY);
                return builder.build();
            } else {
                return null;
            }
        }
    }

    static class OptionPattern extends UtterancePattern {
        /**
         * @param u
         * @return
         */
        @Override
        public Utterance matches(Utterance u) {
            //the $REF the sauce
            //the type of $REF the type of sauce
            //where possible values of ref are all of the phyobj properties?
            //"REPLY(brad,self,option(VAR0:physobj),{sauce(VAR0:physobj),DEFINITE(VAR0:physobj)})
//            if ( u.getWordsAsString().matches("")) {
//                Utterance.Builder builder= new Utterance.Builder(u);
//                builder.setSemantics(Factory.createPredicate("val",Factory.createSymbol(u.getWordsAsString())));
//                builder.setUtteranceType(UtteranceType.REPLY);
//                return builder.build();
//            } else {
                return null;
//            }
        }
    }
    static class AreaPattern extends UtterancePattern {
        /**
         * @param u
         * @return
         */
        @Override
        public Utterance matches(Utterance u) {

            List<Term> areaProperties= new ArrayList<>();

            //TODO:brad: what if we need to filter groups by agent?
            try {
                areaProperties =TRADE.getAvailableService(new TRADEServiceConstraints().name("getPropertiesHandled").inGroups("area")).call(List.class);
            } catch (TRADEException e) {
                log.error("[areaPattern.matches] ",e);
            }

            for(Term areaProperty: areaProperties){
                //$REF
                //area $ref?
                //values for ref are all of the properties handled by the area consultant
                //TODO:brad make this an actual regex? and use .matches()
                String areaPattern = areaProperty.getName();
                if ( areaPattern.equals("\""+u.getWordsAsString()+"\"")) {
                    //"REPLY(brad,self,area(VAR0:area),{pantry(VAR0:area),DEFINITE(VAR0:area)})"
                    Utterance.Builder builder= new Utterance.Builder(u);
                    builder.setSemantics(Factory.createPredicate("area(VAR0:area)"));
                    List<Term> supplementalSemantics= new ArrayList<>();
                    supplementalSemantics.add(Factory.createPredicate(areaProperty.getName(),Factory.createVariable("VAR0","area")));
                    builder.setSupplementalSemantics(supplementalSemantics);
                    Map<Variable, Symbol> teirAssignments = new HashMap<>();
                    teirAssignments.put(Factory.createVariable("VAR0","area"),Factory.createSymbol("DEFINITE"));
                    builder.setTierAssignments(teirAssignments);
                    builder.setUtteranceType(UtteranceType.REPLY);
                    return builder.build();
                }

            }
            return null;
        }
    }

    static class LocationPattern extends UtterancePattern {
        /**
         * @param u
         * @return
         */
        @Override
        public Utterance matches(Utterance u) {

            List<Term> locationProperties= new ArrayList<>();

            //TODO:brad: what if we need to filter groups by agent?
            try {
                locationProperties =TRADE.getAvailableService(new TRADEServiceConstraints().name("getPropertiesHandled").inGroups("location")).call(List.class);
            } catch (TRADEException e) {
                log.error("[LocationPattern.matches] ",e);
            }

            for(Term locationProperty: locationProperties){
                //$REF
                //location $ref?
                //values for ref are all of the properties handled by the location consultant
                String locationPattern = locationProperty.getName();
                if ( u.getWordsAsString().matches(locationPattern)) {
                    //"REPLY(brad,self,location(VAR0:location),{pantry(VAR0:location),DEFINITE(VAR0:location)})"
                Utterance.Builder builder= new Utterance.Builder(u);
                builder.setSemantics(Factory.createPredicate("location(VAR0:location)"));
                List<Term> supplementalSemantics= new ArrayList<>();
                supplementalSemantics.add(Factory.createPredicate(locationProperty.getName(),Factory.createVariable("VAR0","location")));
                supplementalSemantics.add(Factory.createPredicate("DEFINITE",Factory.createVariable("VAR0","location")));
                builder.setSupplementalSemantics(supplementalSemantics);
                builder.setUtteranceType(UtteranceType.REPLY);
                return builder.build();
                }

            }
            return null;
        }
    }

    static class CognexJobPattern extends UtterancePattern {
        /**
         * @param u
         * @return
         */
        @Override
        public Utterance matches(Utterance u) {
            //plantain
            //job detect plantain
            //detect plantain
//            if ( u.getWordsAsString().matches("")) {
//                Utterance.Builder builder= new Utterance.Builder(u);
//                builder.setSemantics(Factory.createPredicate("val",Factory.createSymbol(u.getWordsAsString())));
//                builder.setUtteranceType(UtteranceType.REPLY);
//                return builder.build();
//            } else {
                return null;
//            }
        }
    }

    public PatternMatchingParser(){
        super();
            //TODO:brad: is there a better way to do this?
            knownPatterns.put(IntegerPattern.class.getSimpleName(),IntegerPattern.class);
            knownPatterns.put("area",AreaPattern.class);
    }

    /**
     * @param patternName
     * @return
     */
    @TRADEService
    public boolean activatePattern(String patternName){
        try {
            activePatterns.add(knownPatterns.get(patternName));
            return true;
        } catch (Exception e) {
            log.error("[activatePattern] no pattern found for patternName: {}",patternName,e);
            return false;
        }
    }

    /**
     * @param patternName
     * @return
     */
    @TRADEService
    public boolean deactivatePattern(String patternName){
       boolean removed= activePatterns.removeIf(x -> x.getSimpleName().equals(patternName));
       if(!removed){
           log.warn("[deactivatePattern] no pattern with name: {} was found",patternName);
       }
       return removed;
    }

    /**
     * @param u
     * @return
     */
    @TRADEService
    @Action
    public Utterance parseUtterance(Utterance u){
        //TODO:brad: what if multiple match? right now we get the first
        for (Class<? extends UtterancePattern> p:activePatterns){
            Utterance match= null;
            try {
               match = p.getDeclaredConstructor().newInstance().matches(u);
            } catch (Exception e) {
                log.error("[parseUtterance]",e);
            }
            if(match != null){
                log.info("[parseUtterance] match found, resulting semantics: {}",match);
                return match;
            }
        }

        log.info("no match found for {}",u);
        return null;
    }

    /**
     * Used for testing not sure if we want/need to keep in the long run
     * @return
     */
    public int getNumberOfActivePatterns(){
        return activePatterns.size();
    }


    /**
     * Below is derived from Matthias's reference code.
     */

    // Table to store orders and their semantic representations
    private  Map<String, OrderComponents> orderTable = new HashMap<>();

    class OrderComponents {
        private String foodItem;
        private Set<String> additions;
        private Set<String> subtractions;

        public OrderComponents(String foodItem, Set<String> additions, Set<String> subtractions) {
            this.foodItem = foodItem;
            this.additions = additions;
            this.subtractions = subtractions;
        }

        public String getFoodItem() {
            return foodItem;
        }

        public Set<String> getAdditions() {
            return additions;
        }

        public Set<String> getSubtractions() {
            return subtractions;
        }

        @Override
        public String toString() {
            return "OrderComponents{" +
                    "foodItem='" + foodItem + '\'' +
                    ", additions=" + additions +
                    ", subtractions=" + subtractions +
                    '}';
        }
    }

    // private static void initializeOrderTable() {
    //     // Initialize the order table with some example entries
    //     orderTable.put("beef burger with cheese and mustard but no lettuce",
    // 		       parseOrder("beef burger with cheese and mustard but no lettuce"));
    //     orderTable.put("burger with a side of fries",
    // 		       parseOrder("burger with a side of fries"));
    // }

    public void processOrder(String order) {
        log.info("Considering order: " + order);
        // Check if the order is already in the table
        if (orderTable.containsKey(order)) {
            log.info("Order already exists in the table: " + orderTable.get(order));
        } else {
            // If not, parse the order
            OrderComponents semanticRepresentation = parseOrder(order);
            // Check for similar orders in the table
            OrderComponents similarOrder = findSimilarOrder(semanticRepresentation);
            if (similarOrder != null) {
                // Ask the user for confirmation
                log.info("Similar order found: " + similarOrder.getFoodItem());
                System.out.print("Do you believe the semantic interpretation is correct for this order? (yes/no): ");
                log.info("semanticRepresentation: " + semanticRepresentation);
                String confirmation = new Scanner(System.in).nextLine().toLowerCase();
                if ("yes".equals(confirmation)) {
                    // If yes, add the new order with the same semantics
                    orderTable.put(order, similarOrder);
                    log.info("Order added with the same semantics as the similar order.");
                    return;
                }
            }

            // check that the parse is correct, then add it to the table
            log.info("Verify that this is the correct interpretation: " + semanticRepresentation);
            Scanner scanner = new Scanner(System.in);
            String confirmation = scanner.nextLine().toLowerCase();
            if ("".equals(confirmation)) {
                orderTable.put(order, semanticRepresentation);
                log.info("Order added to the table: " + semanticRepresentation);
            } else {
                System.out.print("Enter the food item: ");
                String foodItem = scanner.nextLine().toLowerCase();

                System.out.print("Enter the additions (comma-separated): ");
                Set<String> additions = parseCommaSeparatedInput(scanner.nextLine());

                System.out.print("Enter the subtractions (comma-separated): ");
                Set<String> subtractions = parseCommaSeparatedInput(scanner.nextLine());

                // put it in the table
                orderTable.put(order, new OrderComponents(foodItem, additions, subtractions));
            }
        }
    }

    private Set<String> parseCommaSeparatedInput(String input) {
        if (input.equals("")) {
            return new HashSet<>();
        } else {
            // Parse comma-separated input into a set of items
            String[] itemsArray = input.split(",");
            Set<String> items = new HashSet<>();
            for (String item : itemsArray) {
                items.add(item.trim().toLowerCase());
            }
            return items;
        }
    }

    private OrderComponents findSimilarOrder(OrderComponents order) {
        // Find a similar order in the table
        for (OrderComponents standardOrder : orderTable.values()) {
            if (areOrdersSimilar(order, standardOrder)) {
                return standardOrder;
            }
        }
        return null;
    }

    private boolean areOrdersSimilar(OrderComponents components1, OrderComponents components2) {

        //	log.info("Food: " + components1.getFoodItem() + "=" + components2.getFoodItem() + " " + components1.getFoodItem().equalsIgnoreCase(components2.getFoodItem()));
        //	log.info("Add : " + components1.getAdditions() + "=" + components2.getAdditions() + " " + areSetsSimilar(components1.getAdditions(), components2.getAdditions()));
        //	log.info("Sub : " + components1.getSubtractions() + "=" + components2.getSubtractions() + " " + areSetsSimilar(components1.getSubtractions(), components2.getSubtractions()));
        // Compare order components
        return components1 != null && components2 != null &&
                components1.getFoodItem().equalsIgnoreCase(components2.getFoodItem()) &&
                areSetsSimilar(components1.getAdditions(), components2.getAdditions()) &&
                areSetsSimilar(components1.getSubtractions(), components2.getSubtractions());
    }

    private boolean areSetsSimilar(Set<String> set1, Set<String> set2) {
        if (set1.isEmpty() && set2.isEmpty()) {
            return true;
        } else {
            // Check if two sets are similar (allowing for variations)
            return set1.stream().anyMatch(item1 ->
                    set2.stream().anyMatch(item2 -> areItemsSimilar(item1, item2))
            );
        }
    }


    private  void modifyStandardOrder(String standardOrder, String modifications) {
        // Parse modifications and update the standard order
        OrderComponents modifiedComponents = parseOrder(modifications);
        OrderComponents existingComponents = orderTable.get(standardOrder);

        // Merge additions and subtractions
        Set<String> mergedAdditions = new HashSet<>(existingComponents.getAdditions());
        mergedAdditions.addAll(modifiedComponents.getAdditions());

        Set<String> mergedSubtractions = new HashSet<>(existingComponents.getSubtractions());
        mergedSubtractions.addAll(modifiedComponents.getSubtractions());

        // Update the standard order with merged semantics
        OrderComponents mergedComponents = new OrderComponents(
                existingComponents.getFoodItem(),
                mergedAdditions,
                mergedSubtractions
        );

        orderTable.put(standardOrder, mergedComponents);
        log.info("Modified order: " + orderTable.get(standardOrder));
    }


    private boolean doOrdersHaveCommonTerms(String order1, String order2) {
        // Check if two orders are similar based on key terms
        Set<String> terms1 = extractKeyTerms(order1);
        Set<String> terms2 = extractKeyTerms(order2);

        // Intersection of significant terms
        terms1.retainAll(terms2);

        // Orders are considered similar if they share key terms
        return !terms1.isEmpty();
    }


    private boolean areItemsSimilar(String item1, String item2) {
        // Check if two items are similar (allowing for variations)
        // This is a basic example; you may need a more sophisticated approach
        return item1.equalsIgnoreCase(item2);
    }


    private Set<String> extractKeyTerms(String order) {
        // Extract key terms (food items and significant words) from the order
        Set<String> keyTerms = new HashSet<>();

        String[] terms = order.split("\\s+");
        for (String term : terms) {
            // Basic check for a food item (you may need a more sophisticated approach)
            if (term.equalsIgnoreCase("burger") || term.equalsIgnoreCase("fries")) {
                keyTerms.add(term.toLowerCase());
            } else if (!Arrays.asList("I", "you", "a", "an", "the", "and", "with", "without", "of", "for", "to", "in", "on")
                    .contains(term.toLowerCase())) {
                keyTerms.add(term.toLowerCase());
            }
        }
        return keyTerms;
    }

    private String mergeSemantics(String existingSemantics, String modifications) {
        // Merge existing semantics with modifications
        // This is a simple concatenation; you may need a more sophisticated approach for a real-world application
        return existingSemantics + ", " + modifications;
    }

    private void parseOrder(String[] orders) {
        String[] cleaned = new String[orders.length];
        // first filter out disfluencies then check for various openings:
        // may/could/can i have/do/get/order...
        for (int i = 0; i < orders.length; i++)
            cleaned[i] = orders[i].toLowerCase().replaceAll("\\b(make it|could you|can you|would you|please|um|uh|ah|yeah|yes|oh|too|you know|thanks|well|so)\\b", "").replace(",", "").replace(".", "").replace("?", "").replace("  ", " ").replaceAll("\\b(may i have|can i have|could i have|may i do|can i do|could i do|may i get|can i get|could i get|give me|get me|make me|i'll do|i'll get|i'll have|i'll take|i'll go for|i'm going for|i want|i guess|let's do|let's go with|i'd like|i'm thinking|i'm in the mood for|i'm craving|i'm opting for|in the mood for|for my meal|for my order|make my order|to that|with that|maybe|place an order for|order up|ordering|combo)\\b", "").replace("like", "").replace(": ", "").replace("  ", " ").trim();

        // examine each string
        String[] foodItem = new String[orders.length];
        String[] remainingOrder = new String[orders.length];
        Pattern p1 = Pattern.compile("(?<=\\b(a |the )\\b).*?(?=\\b( with| and)\\b)");
        Pattern p2 = Pattern.compile("\\b( with| and)\\b");
        for (int i = 0; i < cleaned.length; i++) {
            Matcher m1 = p1.matcher(cleaned[i]);
            Matcher m2 = p2.matcher(cleaned[i]);
            if (m1.find()) {
                // get the first
                foodItem[i] = m1.group(0);
                // get the rest
                remainingOrder[i] = cleaned[i].substring(m1.end()).trim();
                System.out.print("=> " + foodItem[i]);
            } else if (m2.find()) {
                // they just named the noun with no determiner
                foodItem[i] = cleaned[i].substring(0, m2.start()).trim();
                // get the rest
                remainingOrder[i] = cleaned[i].substring(m2.end()).trim();
                System.out.print("=> " + foodItem[i]);
            } else {
                System.out.print("XX " + cleaned[i]);
                remainingOrder[i] = cleaned[i];
            }
            remainingOrder[i] = remainingOrder[i].replaceAll("\\b(a |with|extra|add|and|side of|side order of|on the side|some |add|portion of|serving of|include|the|throw in|don't forget the|let's|don't skimp|generous helping of|crispy|to|the order|on the|delicious|make sure|pair|that|those|mouth-watering|small|large)\\b", "").replace("  ", " ").trim();
            String[] terms = remainingOrder[i].split("\\s+");
            log.info(" WITH " + terms[0]);
        }
    }

	    /*

	    // split the rest by spaces
	    String[] terms = remainingOrder[i].split("\\s+");
	    // now parse each phrase
	    if (terms!=null && terms[0].equals("with")) {
		// look for "and"
		int k=1;
		while (terms.length>k && !terms[k].equals("and")) k++;


	    if (remainingOrder[i].startsWith("and") || remainingOrder[i].startsWith("with")) {

	    }
	    */


    public OrderComponents parseOrder(String order) {
        // this should leave only the additions
        String foodItem = extractFoodItem(order);
        Set<String> additions = extractAdditions(order);
        Set<String> subtractions = extractSubtractions(order);

        OrderComponents o= new OrderComponents(foodItem, additions, subtractions);
        log.info("parsed order: "+o);
        return o;
    }

    private String extractFoodItem(String order) {
        // Extract the food item from the order
        String[] terms = order.split("\\s+");
        for (String term : terms) {
            // Basic check for a food item (you may need a more sophisticated approach)
            if (term.equalsIgnoreCase("burger") || term.equalsIgnoreCase("taco")) {
                return term.toLowerCase();
            }
        }
        return "unknown";
    }

    private Set<String> extractAdditions(String order) {
        // Extract items to add based on the specified modifier
        return extractItems(order, "with");
    }

    private Set<String> extractSubtractions(String order) {
        // Extract items to remove based on the specified modifier
        return extractItems(order, "without");
    }

    private Set<String> extractItems(String order, String modifier) {
        // Extract items based on the specified modifier (e.g., "with" or "without")
        Set<String> items = new HashSet<>();
        Pattern pattern = Pattern.compile(modifier + "\\s*([^.,]+)([.,]|$)");
        Matcher matcher = pattern.matcher(order);

        while (matcher.find()) {
            items.add(matcher.group(1).toLowerCase());
        }

        return items;
    }

    public void printOrderTable() {
        log.info("\nOrder Table:");
        for (Map.Entry<String, OrderComponents> entry : orderTable.entrySet()) {
            log.info(entry.getKey() + " => " + entry.getValue());
        }
    }

}
