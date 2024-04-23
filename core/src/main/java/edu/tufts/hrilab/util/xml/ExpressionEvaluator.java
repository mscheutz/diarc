/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util.xml;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

public class ExpressionEvaluator {
	
	private ScriptEngine engine;
	private ExpressionSubstitutions replacementTable;
	
	public ExpressionEvaluator() {
        engine = new ScriptEngineManager().getEngineByName("JavaScript");
        initializeStringReplacementTable();
	}

	private void initializeStringReplacementTable() {
		replacementTable = new ExpressionSubstitutions();
		replacementTable.add("PI", new Value.Constant(Math.PI));
		replacementTable.add("RAND", new Value.RandomValue0To1());
	}

	/** Evaluates the expression, substituting the default replacements and 
	 * also any additional replacements.  Note that the additional replacements
	 * must be in UPPER CASE 
	 *
	 */
	
	public double evaluateDouble(String originalExpression, Substitution... additionalSubstitutions) {
		String workingExpression = originalExpression;
		
		// begin with additional replacements (to allow possible overrides, for instance)
		if (additionalSubstitutions.length > 0) {
			ExpressionSubstitutions additionalReplacements = new ExpressionSubstitutions();
			for (Substitution each : additionalSubstitutions) {
				additionalReplacements.add(each.name, each.value);
			}
			workingExpression = additionalReplacements.substitute(workingExpression);
		}

		// then move onto "standard" substitutions:
		workingExpression = replacementTable.substitute(workingExpression);
		
		
		try {
			return Double.parseDouble(engine.eval(workingExpression).toString());
		} catch (ScriptException e) {
			throw new Error("Could not evaluate the expression " + originalExpression + 
					"due to: \n" + e);
		}
	}
}
