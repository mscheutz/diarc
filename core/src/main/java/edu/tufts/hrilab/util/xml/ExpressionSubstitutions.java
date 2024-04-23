/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util.xml;

import java.util.HashMap;
import java.util.Map.Entry;

public class ExpressionSubstitutions {
	private HashMap<String, Value> substitutions = new HashMap<String, Value>();  // everything is in UPPPER CASE

	public void add(String name, Value value) {
		substitutions.put(name.toUpperCase(), value);
	}

	public String substitute(String expression) {
		expression = expression.toUpperCase();
		for (Entry<String, Value> eachEntry : substitutions.entrySet()) {
			String name = eachEntry.getKey();
			Value value = eachEntry.getValue();
			expression = expression.replaceAll(name, Double.toString(value.getValue()));
			// NOTE!!!  For now I'm relying on the lack of many possible substitutions 
			//          to ensure there's not crossover of terms.  Eventually it might be 
			//          best to do a "whole word only" replacement.
		}
		return expression;
	}
	
	
}
