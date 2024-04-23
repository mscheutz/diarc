/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util.xml;

public interface Value {
	public double getValue();
	
	
	
	public class Constant implements Value {
		double value;
		
		public Constant(double value) {
			this.value = value;
		}
		
		@Override
		public double getValue() {
			return value;
		}
	}
	
	
	public class RandomAngle implements Value {
		@Override
		public double getValue() {
			return Math.random() * (Math.PI * 2);
		}
		
	}
	
	
	public class RandomValue0To1 implements Value {
		@Override
		public double getValue() {
			return Math.random();
		}
		
	}
	
}
