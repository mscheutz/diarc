import java.lang.Long;
import javax.vecmath.Point3d;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;

() = findAndPressButton["findAndPressButton takes a refrence to a physical object, searches for it, and presses it"](Symbol ?objectRef) {
    java.util.List !tokenIds;
    Predicate !failCond;
    Point3d !forward;
    Point3d !up;
    Point3d !down;

    !forward = op:newObject("javax.vecmath.Point3d", .5, 0, 1);
    !up = op:newObject("javax.vecmath.Point3d", .5, 0, 1.4);
    !down = op:newObject("javax.vecmath.Point3d", .5, 0, .7);

    // clear and rebuild octomap
    act:lookAround();

    // point head forward
    act:pointHeadTo(!forward);
    op:sleep(3000);

    // look for ?objectRef
    act:startVisualSearch(?objectRef);
    !tokenIds = act:getTokenIds(?objectRef);

    // if no object detected, look up and try again
    if (op:isEmpty(!tokenIds)) {
      act:stopVisualSearch(?objectRef);
      act:pointHeadTo(!up);
      act:startVisualSearch(?objectRef);
    }

    // if no object detected, look down and try again
    !tokenIds = act:getTokenIds(?objectRef);
    if (op:isEmpty(!tokenIds)) {
      act:stopVisualSearch(?objectRef);
      act:pointHeadTo(!down);
      act:startVisualSearch(?objectRef);
    }

    // center the object in the frame
//    act:pointHeadTo(?objectRef);
//    act:stopVisualSearch(?objectRef);
//    act:startVisualSearch(?objectRef);
//    op:sleep(5000);
//    !tokenIds = act:getTokenIds(?objectRef);

    try {
      if (~op:isEmpty(!tokenIds)) {
        op:log("info", ">> pressObject ?objectRef");
        act:pressObject("presser", ?objectRef);
      } else {
        !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(see(?actor, ?objectRef))");
        exit(FAIL, !failCond);
      }
    } finally {
      op:log("info", ">> stopVisualSearch for ?objectRef");
      act:stopVisualSearch(?objectRef);
    }
}

() = lookForPanel["looks for button panel"](Point3d ?direction, Symbol ?objectRef) {
    java.util.List !tokenIds;
    java.lang.Long !fiveSeconds;
    java.lang.Long !startTime;
    java.lang.Long !currTime;
    java.lang.Long !comp;
    java.lang.Boolean !bool;
    java.lang.Boolean !true;

    !fiveSeconds = op:newObject("java.lang.Long", 5000);
    !true = op:newObject("java.lang.Boolean", true);
    !bool = op:newObject("java.lang.Boolean", true);
    (!tokenIds) = act:getTokenIds(?objectRef);

    !startTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
    act:pointHeadTo(?direction);
    // wait 5 seconds (ends early if object is found)
    while (op:isEmpty(!tokenIds) && op:==(!bool, !true)) {
      !currTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
      !comp = op:invokeStaticMethod("java.lang.Long", "sum", !startTime, !fiveSeconds);
      
      if (op:gt(!currTime, !comp)) {
        !bool = op:newObject("java.lang.Boolean", false);
      }
      (!tokenIds) = act:getTokenIds(?objectRef);
    }
}

() = pressUpDownLoop["press up then down"]() {
    Symbol !upButton;
    Symbol !downButton;

    !upButton = op:newObject("edu.tufts.hrilab.fol.Symbol", "physobj_1:physobj");
    act:findAndPressButton(!upButton);

    !downButton = op:newObject("edu.tufts.hrilab.fol.Symbol", "physobj_2:physobj");
    act:findAndPressButton(!downButton);
}
