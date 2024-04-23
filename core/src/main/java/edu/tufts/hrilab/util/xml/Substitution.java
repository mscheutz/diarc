/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util.xml;

/* Tiny data structure for holding substitutions (String and their Double values) 
 * Also holds common static final values (like RANDOM_ANGLE_SUBSTITUTION)*/

public class Substitution {
	
	public static final Substitution RANDOM_ANGLE_SUBSTITUTION = 
			new Substitution("rand", new Value.RandomAngle());

	
	public String name;
	public Value value;

	public Substitution(String name, Value value) {
		this.name = name;
		this.value = value;
	}
}
