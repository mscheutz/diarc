/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.unity.space_station.llm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.UUID;
import org.apache.commons.lang3.StringEscapeUtils;
import com.google.gson.Gson;
import com.google.gson.annotations.Expose;
import edu.tufts.hrilab.slug.parsing.llm.ParserResponse;
import edu.tufts.hrilab.slug.parsing.llm.Descriptor;
import edu.tufts.hrilab.slug.parsing.llm.Referent;
import edu.tufts.hrilab.slug.parsing.llm.Intention;
import edu.tufts.hrilab.slug.parsing.llm.Proposition;

public class LlamaSpaceStationResponse {
    private static Gson gson = new Gson();

    public String action;
    public String area;
    public String side;
    public String number;

    private List<String> actions = Arrays.asList(new String[]{ "stop", "dontKnow", "repair", "monitor" });
    private List<String> locationActions = Arrays.asList(new String[]{ "goto", "repair" });
    private List<String> areas = Arrays.asList(new String[]{ "alpha", "beta", "gamma" });
    private List<String> sides = Arrays.asList(new String[]{ "left", "right" });
    private List<String> numbers = Arrays.asList(new String[]{ "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve" });

    public static LlamaSpaceStationResponse fromString (String s) {
        return gson.fromJson(s, LlamaSpaceStationResponse.class);
    }

    @Override
    public String toString () {
        return gson.toJson(this);
    }

    public ParserResponse toParserResponse () {
        ParserResponse parserRes;

        if (valid(number, numbers) && valid(side, sides) && valid(area, areas) && valid(action, locationActions)) {
            parserRes = actionResponse(3);
            parserRes.intention.proposition.arguments[0] = area;
            parserRes.intention.proposition.arguments[1] = side;
            parserRes.intention.proposition.arguments[2] = number;
        }else if (valid(side, sides) && valid(number, numbers) && action.equals("goto")) {
            parserRes = actionResponse(2);
            parserRes.intention.proposition.arguments[0] = side;
            parserRes.intention.proposition.arguments[1] = number;
        } else if (valid(area, areas) && (action.equals("goto") || action.equals("monitor")) ) {
            parserRes = actionResponse(1);
            parserRes.intention.proposition.arguments[0] = area;
        } else if (action.equals("repair") || action.equals("dontKnow")) {
            parserRes = actionResponse(0);
        } else if (action.equals("stop")) {
            action = "cancelGoal";
            parserRes = actionResponse(0);
        } else {
            action = "dontKnow";
            parserRes = actionResponse(0);
        }

        return parserRes;
    }

    private ParserResponse actionResponse (int args) {
        ParserResponse parserRes = new ParserResponse();

        parserRes.intention = actionIntention(args);
        parserRes.referents = new Referent[0];
        parserRes.descriptors = new Descriptor[0];

        return parserRes;
    }

    private Intention actionIntention (int args) {
        Intention intention = new Intention();
        Proposition proposition = new Proposition();

        //{"intent":"instruct","proposition":{"text":"pick up","type":"action","arguments":["VAR0"]}}
        intention.intent = "instruct";
        proposition.text = action;
        proposition.type = "action";
        proposition.arguments = new String[args];

        for (int i = 0; i < args; i++) {
            proposition.arguments[i] = "VAR" + i;
        }

        intention.proposition = proposition;

        return intention;
    }


    /**
     * Takes the object and produces a predicate string containing a dialog action goal predicate.
     *
     * @param actor The name of the actor being instructed
     * @return string containing the goal predicate
     */
    public String toPredicate (String actor) {
        String str = "";
        if (valid(number, numbers) && valid(side, sides) && valid(area, areas) && valid(action, locationActions)) {
            str = action + "(" + actor + "," + area + "," + side + "," + number + ")";
        } else if (valid(side, sides) && valid(number, numbers) && action.equals("goto")) {
            str = action + "(" + actor + "," + side + "," + number + ")";
        } else if (valid(area, areas) && action.equals("goto")) {
            str = action + "(" + actor + "," + area + ")";
        } else if (valid(area, areas) && action.equals("monitor")) {
            str = action + "(" + actor + "," + area + ")";
        } else if (action.equals("repair") || action.equals("dontKnow")) {
            str = action + "(" + actor + ")";
        } else if (action.equals("stop")) {
            str = "cancelGoal(" + actor + ")";
        } else {
            str = "dontKnow(" + actor + ")";
        }
        return str;
    }
    //goal(dempster,handled(dempster,utterance(evan,dempster,and(dempster),semantics(want(evan,stand(dempster))),indirectSemantics(),suppSemantics())))
    public String toDialogGoal (String actor, String predicate) {
        return "goal(" + actor + ",handled(" + actor + ",utterance(brad," + actor + ",and(" + actor + "),semantics(want(brad," + predicate + ")),indirectSemantics(),suppSemantics())))";
    }

    private boolean valid (String val, List<String> comp) {
        if (val == null) {
            return false;
        }
        if (!comp.contains(val)) {
            return false;
        }
        return true;
    }
}