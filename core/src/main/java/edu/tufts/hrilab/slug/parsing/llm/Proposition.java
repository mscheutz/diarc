/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.llm;

/* {"text":"pick up","type":"action","arguments":["VAR0"]} */

public class Proposition {
    public String text;
    public String type;
    public String[] arguments;
}