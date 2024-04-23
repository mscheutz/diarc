import java.lang.Long;
import javax.vecmath.Point3d;

() = findAndPressButton["findAndPressButton takes a refrence to a physical object, searches for it, and presses it"](edu.tufts.hrilab.fol.Symbol ?objectRef) {
    java.util.List ?tokenIds;

    act:lookAround(); // rebuild octomap

    op:log(debug, "[findObject] finding object ?objectRef");
    act:getTypeId(?objectRef); // start vision search
    (?tokenIds) = act:getTokenIds(?objectRef);

    javax.vecmath.Point3d !forward;
    javax.vecmath.Point3d !up;
    javax.vecmath.Point3d !down;

    !forward = op:newObject("javax.vecmath.Point3d", .5, 0, 1);
    !up = op:newObject("javax.vecmath.Point3d", .5, 0, 1.4);
    !down = op:newObject("javax.vecmath.Point3d", .5, 0, .7);

    act:lookForPanel(!forward, ?objectRef);
    (?tokenIds) = act:getTokenIds(?objectRef);
    if (op:isEmpty(?tokenIds)) {
      act:lookForPanel(!up, ?objectRef);
    }
    (?tokenIds) = act:getTokenIds(?objectRef);
    if (op:isEmpty(?tokenIds)) {
      act:lookForPanel(!down, ?objectRef);
    }

    act:pointHeadTo(?objectRef);

    act:pressObj("arm", ?objectRef);
    op:log("info", "ran pressButton");

    act:stopAllSearches();
    op:log("info", "stopped searching for ?objectRef");
}

() = lookForPanel["looks for button panel"](javax.vecmath.Point3d ?direction, edu.tufts.hrilab.fol.Symbol ?objectRef) {
    java.util.List ?tokenIds;
    java.lang.Long !fiveSeconds;
    java.lang.Long !startTime;
    java.lang.Long !currTime;
    java.lang.Long !comp;
    java.lang.Boolean !bool;
    java.lang.Boolean !true;

    !fiveSeconds = op:newObject("java.lang.Long", 5000);
    !true = op:newObject("java.lang.Boolean", true);
    !bool = op:newObject("java.lang.Boolean", true);
    (?tokenIds) = act:getTokenIds(?objectRef);

    !startTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
    act:pointHeadTo(?direction);
    // wait 5 seconds (ends early if object is found)
    while (op:isEmpty(?tokenIds) && op:==(!bool, !true)) {
      !currTime = op:invokeStaticMethod("java.lang.System", "currentTimeMillis");
      !comp = op:invokeStaticMethod("java.lang.Long", "sum", !startTime, !fiveSeconds);
      
      if (op:gt(!currTime, !comp)) {
        !bool = op:newObject("java.lang.Boolean", false);
      }
      (?tokenIds) = act:getTokenIds(?objectRef);
    }
}

() = pressUpDownLoop["press up then down"]() {
    act:lookAround();

    edu.tufts.hrilab.fol.Symbol !upButton;
    edu.tufts.hrilab.fol.Symbol !downButton;

    !upButton = op:newObject("edu.tufts.hrilab.fol.Symbol", "physobj_1:physobj");
    act:getTypeId(!upButton);
    act:pressObj("arm", !upButton);
    act:stopAllSearches();

    !downButton = op:newObject("edu.tufts.hrilab.fol.Symbol", "physobj_2:physobj");
    act:getTypeId(!downButton);
    act:pressObj("arm", !downButton);
    op:log("info", "ran pressUpDownButton");
    act:stopAllSearches();
}
