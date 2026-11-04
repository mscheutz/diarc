/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.llm;

/*
    {"referents":[{"text":"apple","type":"physobj","role":"central","variable_name":"VAR0","cognitive_status":"DEFINITE"}],
     "intention":{"intent":"instruct","proposition":{"text":"pick up","type":"action","arguments":["VAR0"]}},
     "descriptors":[{"text":"apple","arguments":["VAR0"]}]}
 */
public class ParserResponse {
    public Referent[] referents;
    public Intention intention;
    public Descriptor[] descriptors;
}

