/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fol;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TermTest {

    @Test
    public void testInstanceOf() {
        Term t1 = Factory.createPredicate("facing(?actor, east)");
        Term t2 = Factory.createPredicate("facing(self, east)");

        assertTrue(t2.instanceOf(t1));
        assertFalse(t1.instanceOf(t2));

        t1 = Factory.createPredicate("facing(?actor, east)");
        t2 = Factory.createPredicate("facing(self, west)");

        assertFalse(t2.instanceOf(t1));
        assertFalse(t1.instanceOf(t2));
    }
}
