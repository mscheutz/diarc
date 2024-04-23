/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author willie
 */
public class ActionBindingTest {
  ActionBinding local;
  ActionBinding nonlocal;
  ActionBinding objArg;
  ActionBinding unbound;
  ActionBinding bound;
  ActionBinding outer;
  ActionBinding nested;
  ActionBinding defVal;
  
  public ActionBindingTest() {
  }
  
  @Before
  public void setUp() {
    local = new ActionBinding.Builder("!arg", String.class).setIsLocal(true).build();
    nonlocal = new ActionBinding.Builder("?arg", String.class).build();
    objArg = new ActionBinding.Builder("?arg", Object.class).build();
    unbound = local;
    bound = new ActionBinding.Builder("?arg", String.class).setValue("foo").build();
    nested = unbound;
    outer = new ActionBinding.Builder("?arg", String.class).build();
    defVal = new ActionBinding.Builder("?arg", String.class).setDefaultValue("bar").build();
  }

  /**
   * Test of isBound method, of class ActionBinding.
   */
  @Test
  public void testIsBound() {
    assertTrue(bound.isBound());
    assertFalse(unbound.isBound());
  }

  /**
   * Test of bind method, of class ActionBinding.
   */
  @Test
  public void testBind() {
    local.bind(1);
    assertEquals("1", local.getBinding());
  }

  /**
   * Test of bindDeep method, of class ActionBinding.
   */
  @Test
  public void testBindDeep() {
    outer.bind(nested);
    outer.bindDeep("test");
    assertEquals("test", nested.getBinding());
  }

  /**
   * Test of getBinding method, of class ActionBinding.
   */
  @Test
  public void testGetBinding() {
    assertEquals("bar", defVal.getBinding());
  }

  /**
   * Test of getBindingDeep method, of class ActionBinding.
   */
  @Test
  public void testGetBindingDeep() {

    assertEquals("bar", defVal.getBindingDeep());
  }

  /**
   * Test of getBindingNameDeep method, of class ActionBinding.
   */
  @Test
  public void testGetBindingNameDeep() {
  }

  /**
   * Test of getBindingTypeDeep method, of class ActionBinding.
   */
  @Test
  public void testGetBindingTypeDeep() {
  }

  /**
   * Test of clone method, of class ActionBinding.
   */
  @Test
  public void testClone() {
  }

  /**
   * Test of toString method, of class ActionBinding.
   */
  @Test
  public void testToString() {
  }
  
}
