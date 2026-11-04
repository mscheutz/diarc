() = pour["?actor pours ?contents from ?jar1 into ?jar2"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?contents:contents, edu.tufts.hrilab.fol.Symbol ?jar1:jar, edu.tufts.hrilab.fol.Symbol ?jar2:jar) {

    java.lang.Double !angle = 1.0;
    edu.tufts.hrilab.fol.Predicate !cancelCause;

    conditions : {
        pre : holding(?actor, ?jar1);
        pre : above(?actor, ?jar2);
        pre : in(?jar1, ?contents);
    }
    effects : {
        success : not(in(?jar1, ?contents));
        success : in(?jar2, ?contents);
        nonperf: not(in(?jar1, ?contents));
        nonperf: in(?jar2, ?contents);
    }

    op:log("info", "starting to pour");
//    try{
        act:startPouring(?jar1);
//    }
//    catch (CANCEL){
//        op:log("debug","caught cancel status in pour");
//        (!cancelCause) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "canceled(startPouring())");
//        exit(CANCEL,!cancelCause);
//    }
//    finally{
        act:stopPouring(?jar1);
//    }
    op:log("info", "Successfully poured");

}


() = pickup["?actor grabs ?physobj"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?objectRef:physobj) {
    conditions : {
        pre : free(?actor);
    }
    effects : {
        success : holding(?actor, ?objectRef);
        success : not(free(?actor));
    }

    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    java.lang.String !arm = "manipulator";

    (!qt) = op:newObject("javax.vecmath.Quat4d",0,0,0,1);
    (!pt) = op:newObject("javax.vecmath.Point3d",0,0,0.15);

    op:log("info", "pickup");
    //act:openGripper("arm");
    //act:findObject(?physobj);
    //act:moveTo("arm", ?physobj);
    //act:closeGripper("arm");
    //act:findObject(?physobj);
    //TODO:brad/will: this change is so it can put thinks back where it picked them up from, we maybe want a more general solution moving on.
    //act:pauseCapture();
//    act:grab(?physobj);

    float !closePosition = 0.0f;
    edu.tufts.hrilab.fol.Predicate !failCond;

    // TODO: this probably shouldn't be an explicit step here, maybe pre-condition?
    //act:findObject(?objectRef);
    act:openGripper(!arm);

    op:log("debug", "Trying to grasp ?objectRef with !arm.");
    if (act:moveTo(!arm, ?objectRef)) {
      op:log("debug", "Moved to ?objectRef with !arm.");
    } else {
      op:log("debug", "Couldn't plan path to grasp ?objectRef with !arm.");
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(find(?actor, path))");
      exit(fail, !failCond);
    }
    if (act:graspObject(?objectRef)) {
      op:log("debug", "Grasped ?objectRef with !arm.");
    } else {
      op:log("debug", "Couldn't grasp ?objectRef with !arm.");
      !failCond = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "not(graspObject(?objectRef))");
      exit(fail, !failCond);
    }

    act:moveToRelative(!arm, !pt, !qt);
    op:sleep(2000);
    op:log("info", "Successfully picked up");

}

() = release["?actor releases ?physobj"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?physobj:physobj) {
    conditions : {
        pre : holding(?actor, ?physobj);
    }
    effects : {
        success : not(holding(?actor, ?physobj));
        success : free(?actor);
    }

    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    (!qt) = op:newObject("javax.vecmath.Quat4d",0,0,0,1);
    (!pt) = op:newObject("javax.vecmath.Point3d",0,0,-0.15);

    op:log("info", "release");
    act:moveToRelative("manipulator", !pt, !qt);
    //act:openGripper("manipulator");
    act:releaseObject("default", ?physobj, 1.0);
    //act:moveBackward();
    (!pt) = op:newObject("javax.vecmath.Point3d",0,0,0.25);
    act:moveToRelative("manipulator", !pt, !qt);
    act:resumeCapture();
    op:log("info", "Successfully released");

}

() = moveobjectabove["moves ?physobj1 above ?physobj2"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?physobj1:physobj, edu.tufts.hrilab.fol.Symbol ?physobj2:physobj){

   edu.tufts.hrilab.fol.Predicate !cancelCause;

    conditions : {
        pre : above(?actor, ?physobj1);
    }
    effects : {
        //todo:brad: add observers
        success : not(above(?actor, ?physobj1));
        success : above(?actor, ?physobj2);
        nonperf : not(above(?actor, ?physobj1));
        nonperf : not(above(?actor, ?physobj2));
        nonperf : above(?actor, none);
    }
    act:moveAbove(?physobj1,?physobj2);
    op:log("info", "Successfully moved object above");

}

(long ?typeId) = startPourSearch["?actor looks for all GlobalFeatureValidator matches along with their grasp points in current FOV and returns the search typeId"](edu.tufts.hrilab.fol.Symbol ?actor) {
    java.util.List !tmpDescriptors;
    edu.tufts.hrilab.fol.Symbol !tmpDescriptor;

    !tmpDescriptors = op:newObject("java.util.ArrayList");
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "object X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "red X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "jar X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    ?typeId = act:getTypeId(!tmpDescriptors);

    !tmpDescriptors = op:newObject("java.util.ArrayList");
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "object X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "purple X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "jar X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    ?typeId = act:getTypeId(!tmpDescriptors);

    !tmpDescriptors = op:newObject("java.util.ArrayList");
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "object X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "yellow X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "jar X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    ?typeId = act:getTypeId(!tmpDescriptors);

    !tmpDescriptors = op:newObject("java.util.ArrayList");
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "object X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "blue X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    !tmpDescriptor = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "jar X");
    op:add(!tmpDescriptors, !tmpDescriptor);
    ?typeId = act:getTypeId(!tmpDescriptors);

}
