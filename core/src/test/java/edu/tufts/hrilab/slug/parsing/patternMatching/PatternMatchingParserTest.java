/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.slug.parsing.patternMatching;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestRule;
import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class PatternMatchingParserTest {
    private PatternMatchingParser component;
    private static Logger log = LoggerFactory.getLogger(PatternMatchingParserTest.class);

    @Rule
    public TestRule watcher = new TestWatcher() {
        protected void starting(Description description) {
            log.info(description.getMethodName());
        }
    };


    public PatternMatchingParserTest() {
        component = DiarcComponent.createInstance(PatternMatchingParser.class, "");
    }
    private boolean testUtterance(String utterance, String desired, String speaker, String listener) {
        // component.addWords(complete);
        Utterance parseResult = component.parseUtterance(new Utterance(Factory.createSymbol(speaker),
                Factory.createSymbol(listener),
                Arrays.asList(utterance.split(" ")),
                UtteranceType.UNKNOWN,
                true));

        String outcome = parseResult.toString();
        if (outcome.equals(desired)) {
            log.info("outcome: " + outcome);
            log.info("desired: " + desired + "\n");
        } else {
            log.error("outcome: " + outcome);
            log.error("desired: " + desired + "\n");
        }
        return outcome.equals(desired);
    }
    @Test
    public void IntegerPatternTest(){
        component.activatePattern("IntegerPattern");
        assertEquals(1,component.getNumberOfActivePatterns());

        assertTrue(testUtterance("5",
                "REPLY(brad,self,val(5),{})",
                "brad",
                "self"
        ));

        assertTrue(testUtterance("-73",
                "REPLY(brad,self,val(-73),{})",
                "brad",
                "self"
        ));

        component.deactivatePattern("IntegerPattern");
        assertEquals(0,component.getNumberOfActivePatterns());
    }

    //TODO:convert this into a real test that checks the expected output vs the generated output input
    @Ignore
    @Test
    public void baseUtterancesTest() {
        // Populate the initial order table
        //initializeOrderTable();

        // Provided examples
        component.parseOrder("I'd like a beef burger with cheese and mustard but no lettuce.");
        component.parseOrder("Could I have a burger with a side of fries, please?");
        component.parseOrder("Make it a burger, and don't forget the fries.");
        component.parseOrder("I'm in the mood for a burger with fries.");
        component.parseOrder("Let's do a burger, and add some fries to that.");
        component.parseOrder("I'll go for a classic burger with a serving of fries.");
        component.parseOrder("I'm craving a burger and fries.");
        component.parseOrder("Burger with fries, please.");
        component.parseOrder("I'll have the burger, and yes, include the fries.");
        component.parseOrder("Make my order a burger, and throw in some fries, too.");

        component.parseOrder("For my order, a burger with a side of fries.");
        component.parseOrder("I'm opting for the burger, and let's include fries with that.");
        component.parseOrder("In the mood for a classic burger and a portion of fries.");
        component.parseOrder("Give me a burger, and don't skimp on the fries, please.");
        component.parseOrder("I'd like a burger, and add some crispy fries to the order.");
        component.parseOrder("Make it simple-a burger and a side of fries.");
        component.parseOrder("For my meal, a burger with a generous helping of fries.");
        component.parseOrder("Ordering the burger, and yes, include the fries, too.");
        component.parseOrder("I'll take a burger, and make it a combo with fries.");
        component.parseOrder("A burger, and make sure to include those delicious fries.");

        component.parseOrder("Place an order for a burger, and let's pair that with some fries.");
        component.parseOrder("I'm thinking a burger, and throw in a side of fries, please.");
        component.parseOrder("For my meal, a classic burger and a side order of fries.");
        component.parseOrder("Give me a burger with fries on the side, please.");
        component.parseOrder("Let's go with a burger, and include a serving of fries.");
        component.parseOrder("I'd like a burger, and make it a combo with fries.");
        component.parseOrder("Order up a burger, and add some delicious fries to that.");
        component.parseOrder("I'm going for the burger, and don't forget the fries.");
        component.parseOrder("Burger with fries, that's what I'm in the mood for.");
        component.parseOrder("Make it a burger, and include those mouth-watering fries.");

        component.parseOrder("Burger with fries, please.");
        component.parseOrder("Classic burger, add fries.");
        component.parseOrder("Burger and fries combo.");
        component.parseOrder("Order: burger, side of fries.");
        component.parseOrder("Burger with extra fries.");
        component.parseOrder("Burger with a side of fries.");
        component.parseOrder("Combo: burger and fries.");
        component.parseOrder("Burger + fries, thanks.");
        component.parseOrder("Burger, make it with fries.");
        component.parseOrder("Give me a burger, fries too.");

        component.parseOrder("Um, yeah, I'll take a, uh, burger with, you know, fries.");
        component.parseOrder("So, like, can I get a burger? And, uh, throw in some fries?");
        component.parseOrder("I'm, uh, thinking a burger? With, like, fries, I guess?");
        component.parseOrder("I, um, want a burger. And, uh, make it, you know, with fries.");
        component.parseOrder("Could I, like, get a burger? Oh, and, um, add fries, please?");
        component.parseOrder("Let's, um, do a burger? And, like, add some fries to that?");
        component.parseOrder("I'd, um, like a burger? With, uh, a side of fries, maybe?");
        component.parseOrder("So, I'm in the mood for a, uh, burger? And, you know, fries.");
        component.parseOrder("Ah, yeah, I'll have, um, a burger? And, uh, small fries.");
        component.parseOrder("Make it, um, a burger with, you know, fries on the side.");

        component.parseOrder("Um, yeah, I'll take a, uh, burger with, you know, fries, please.");
        component.parseOrder("So, like, can I get a burger? And, uh, could you throw in some fries?");
        component.parseOrder("I'm, uh, thinking a burger? With, like, fries, I guess?");
        component.parseOrder("Could I, um, have a burger? And, uh, make it with no lettuce, add fries, please?");
        component.parseOrder("I, um, want a burger. And, uh, make it, you know, with fries on the side.");
        component.parseOrder("Let's, um, do a burger? And, like, add some fries to that?");
        component.parseOrder("I'd, um, like a burger? With, uh, a side of fries, maybe?");
        component.parseOrder("So, I'm in the mood for a, uh, burger? And, you know, fries.");
        component.parseOrder("Ah, yeah, I'll have, um, a burger? And, uh, small fries, please.");
        component.parseOrder("Make it, um, a burger with, you know, fries on the side, but make the fries large.");
	/*
String[] orders = new String[]{"I'd like a classic chicken burger with grilled chicken, lettuce, tomato, red onion, mayo, and mustard on a whole wheat bun, accompanied by crispy fries and a medium soft drink.","Give me the BBQ beef burger deluxe with a char-grilled beef patty, bacon, cheddar cheese, BBQ sauce, and onion rings on a brioche bun, plus a side of coleslaw and a large iced tea.","I'm craving the spicy po
rk burger fiesta—seasoned pork patty with pepper jack cheese, jalapeños, banana peppers, chipotle mayo on a ciabatta roll, sweet potato fries
on the side, and a small fruit smoothie.","For me, it's the veggie delight burger with a black bean or quinoa patty, avocado slices on a ciaba
tta roll, a small garden salad on the side, skip the fries, and a refreshing fruit smoothie."};

	"I'll take a classic beef taco with seasoned ground beef, shredded lettuce, diced tomatoes, shredded cheddar cheese, and a dollop of sour cream in a soft corn tortilla, please."

"How about a Southwest chicken burger with a grilled chicken patty, pepper jack cheese, guacamole, lettuce, and tomato on a sesame seed bun, and a side of crispy onion rings?"

"I'm in the mood for a BBQ pulled pork taco—slow-cooked pulled pork, coleslaw, pickles, and tangy barbecue sauce in a soft flour tortilla, and can I get a side of sweet potato fries?"

"Give me a veggie supreme burger featuring a hearty veggie patty, Swiss cheese, sautéed mushrooms, caramelized onions, and Dijon mustard on a whole grain bun, and add a side of garlic parmesan fries."

"For a light option, I'll have a grilled fish taco with marinated grilled fish, cabbage slaw, pico de gallo, and chipotle mayo in a soft corn tortilla, and could I get a side of black bean and corn salad?"

"I'd like a build-your-own burger, please—grilled chicken, cheddar cheese, lettuce, tomato, red onion, pickles, and jalapeños on a brioche bun, and make it a combo with sweet potato tots and a mango iced tea."


"Give me a loaded beef nacho taco with seasoned ground beef, nacho cheese, jalapeños, diced tomatoes, and sour cream in a crunchy taco shell, and throw in a side of chili cheese fries."

"I'm craving a bacon lover's burger—beef patty, crispy bacon strips, cheddar cheese, lettuce, tomato, and smoky barbecue sauce on a pretzel bun, with a side of onion rings."

"How about a buffalo chicken taco with spicy shredded chicken, blue cheese crumbles, shredded lettuce, and ranch dressing in a soft flour tortilla, and add a side of zesty coleslaw?"

"For a taste of the tropics, I'll have a Hawaiian pork burger—grilled pork patty, pineapple ring, teriyaki glaze, lettuce, and Swiss cheese on a Hawaiian sweet roll, and add a side of mango salsa."

"Create a Tex-Mex veggie taco for me, please—black bean and corn salsa, guacamole, shredded lettuce, and chipotle mayo in a soft flour tortilla, and a side of cilantro lime rice."

"I'd like a classic double cheeseburger with two beef patties, American cheese, pickles, ketchup, and mustard on a sesame seed bun, and make it a combo with seasoned curly fries and a chocolate shake."

"Give me a smoky chipotle turkey burger—turkey patty, pepper jack cheese, avocado, lettuce, and chipotle mayo on a multigrain bun, and add a side of sweet potato wedges."

"I'll go for a build-your-own taco trio with shredded chicken, queso fresco, pickled red onions, cilantro, and lime crema in soft corn tortillas, and a side of street corn."

"I'm feeling adventurous, so surprise me with a fusion taco—Korean BBQ beef, kimchi slaw, gochujang aioli, and sesame seeds in a soft flour tortilla, and add a side of loaded tots."

"Create a Mediterranean veggie burger for me—falafel patty, feta cheese, tzatziki sauce, cucumber, and tomato on a pita bun, and a side of Greek salad, please."

"I'll take a spicy chorizo taco with crumbled chorizo, diced onions, cilantro, and lime wedge in a soft corn tortilla, and a side of chili-lime seasoned fries."

"How about a gourmet blue cheese and caramelized onion burger—beef patty, blue cheese crumbles, caramelized onions, arugula, and balsamic glaze on a ciabatta roll, with a side of truffle fries?"

"Give me a California turkey avocado burger with a turkey patty, Swiss cheese, avocado slices, lettuce, and sun-dried tomato aioli on a whole grain bun, and add a side of quinoa salad."

"I'm in the mood for a shrimp po'boy taco—fried shrimp, shredded lettuce, diced tomatoes, and remoulade sauce in a soft flour tortilla, and add a side of Cajun-seasoned sweet potato fries."

"Create a Tex-Mex BBQ pulled jackfruit taco for me—slow-cooked jackfruit, coleslaw, pickles, and barbecue sauce in a soft flour tortilla, and a side of elote corn salad."

"I'd like a breakfast burger with a beef patty, fried egg, bacon, American cheese, and hash brown on a buttered English muffin, and make it a combo with hash brown bites and a coffee."

"Give me a teriyaki salmon taco with grilled salmon, pineapple salsa, cabbage slaw, and teriyaki glaze in a soft corn tortilla, and add a side of seaweed salad."

"How about a Southwest black bean burger—black bean patty, pepper jack cheese, roasted red pepper, avocado, and chipotle mayo on a whole wheat bun, and a side of sweet potato tots."

"I'm craving a Nashville hot chicken taco—spicy fried chicken, coleslaw, pickles, and comeback sauce in a soft flour tortilla, and add a side of mac 'n' cheese bites."

"Create a build-your-own loaded nacho burger with a beef patty, nacho cheese, jalapeños, guacamole, and salsa on a pretzel bun, and a side of chili cheese tots."


	*/


        // Allow the user to specify modifications or a new order
//        Scanner scanner = new Scanner(System.in);
//        log.info("Enter a modification or a new order: ");
//        String userOrder = scanner.nextLine();
//        component.processOrder(userOrder);

        // Print the updated order table
        component.printOrderTable();
        assertTrue(true);
    }
}