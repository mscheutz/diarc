package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.fol.Factory;
import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.ObjectStreamClass;

import static org.junit.Assert.*;

public class GoalVersionSerializationTest {
  @Test
  public void versionedInfoRetainsExistingSerializationIdentifier() {
    assertEquals(-5548368486200074670L, ObjectStreamClass.lookup(GoalInfo.class).getSerialVersionUID());
  }

  @Test
  public void snapshotsAndWaitResultsPreserveVersionsAcrossSerialization() throws Exception {
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    goal.setAsStarted();
    GoalInfo original = goal.getInfo();
    assertEquals(1, new GoalInfo(original).statusVersion);
    GoalWaitResult result = new GoalWaitResult(GoalWaitResult.Outcome.UPDATED, original);
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    try (ObjectOutputStream stream = new ObjectOutputStream(bytes)) {
      stream.writeObject(result);
    }
    GoalWaitResult restored;
    try (ObjectInputStream stream = new ObjectInputStream(new ByteArrayInputStream(bytes.toByteArray()))) {
      restored = (GoalWaitResult) stream.readObject();
    }
    assertEquals(result.outcome, restored.outcome);
    assertEquals(original.gid, restored.getGoalInfo().gid);
    assertEquals(original.statusVersion, restored.getGoalInfo().statusVersion);
    assertEquals(original.status, restored.getGoalInfo().status);
  }
}
