/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.msg;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.polycraft.util.SymbolResolver;

import java.util.HashSet;
import java.util.Set;

public class Recipe extends Msg {

  public RecipeItem[] inputs;
  public RecipeItem[] outputs;

  @Override
  public Set<Predicate> generateAssertions() {
    Set<Predicate> assertions = new HashSet<>();
    assertions.addAll(generateTypeAssertions());
    assertions.addAll(generateRecipeAssertions());
    return assertions;
  }

  private Set<Predicate> generateTypeAssertions() {
    Set<Predicate> assertions = new HashSet<>();

    for (int j = 0; j < inputs.length; j++) {
      String material = SymbolResolver.toGridItem(inputs[j].Item);
      Predicate predicate = Factory.createPredicate("constant", material, material);
      assertions.add(predicate);
    }

    String output = SymbolResolver.toGridItem(outputs[0].Item);
    Predicate predicate = Factory.createPredicate("constant", output, output);
    assertions.add(predicate);

    return assertions;
  }

  private Set<Predicate> generateRecipeAssertions() {
    Set<Predicate> assertions = new HashSet<>();

    StringBuilder sb = new StringBuilder("recipe(");

    // add input ingredients
    String[] predInputs = {"0", "0", "0", "0", "0", "0", "0", "0", "0"};
    for (RecipeItem item : inputs) {
      int slot = item.slot;
      if (slot == -1) {
        slot = 0;
      }
      predInputs[slot] = SymbolResolver.toGridItem(item.Item);
    }
    sb.append(String.join(",", predInputs));

    // add output from recipe
    sb.append(",");
    sb.append(SymbolResolver.toGridItem(outputs[0].Item));
    sb.append(",");
    sb.append(outputs[0].stackSize);

    sb.append(")");
    Predicate p = Factory.createPredicate(sb.toString());
    assertions.add(p);

    return assertions;
  }
}
