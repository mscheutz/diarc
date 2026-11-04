package edu.tufts.hrilab.slug.parsing.tldl;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Variable;

//Used to generate unique free variable IDs within a parse
public class VariableFactory {
  private static VariableFactory instance;
  private int counter = 0;

  private VariableFactory() {
    // Private constructor to prevent instantiation
  }

  public static synchronized VariableFactory getInstance() {
    if (instance == null) {
      instance = new VariableFactory();
    }
    return instance;
  }


  public Variable getNextVariable(String type) {
    return Factory.createVariable("VAR" + counter++, type);
  }

  public void reset() {
    counter = 0;
  }
}