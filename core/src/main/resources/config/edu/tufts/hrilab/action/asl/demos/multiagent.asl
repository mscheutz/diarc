//() = packRef(edu.tufts.hrilab.fol.Symbol ?assista:arm, edu.tufts.hrilab.fol.Symbol ?ur:arm, edu.tufts.hrilab.fol.Symbol ?conv:arm) {
//    //assista stuff
//    edu.tufts.hrilab.fol.Symbol !armoneconveyorpose = "conveyor";
//    edu.tufts.hrilab.fol.Symbol !conveyorstart = "conveyor_surface";
//    edu.tufts.hrilab.fol.Symbol !armoneloading_areapose = "work_area";
//    edu.tufts.hrilab.fol.Symbol !armoneplatform = "work_surface";
//    //Temi stuff
//    edu.tufts.hrilab.fol.Symbol !lab = "lab";
//    edu.tufts.hrilab.fol.Symbol !loading_area = "loading_area";
//    //ur stuff
//    edu.tufts.hrilab.fol.Symbol !conveyorpickup = "conveyorpickup";
//    edu.tufts.hrilab.fol.Symbol !conveyorend = "conveyorend";
//    edu.tufts.hrilab.fol.Symbol !dropoff = "dropoff";
//    //misc/kit stuff
//    edu.tufts.hrilab.fol.Symbol !medkit1 = "medkit1";
//    edu.tufts.hrilab.fol.Symbol !bandage = "bandage";
//    edu.tufts.hrilab.fol.Symbol !antiseptic = "anticeptic";
//    edu.tufts.hrilab.fol.Symbol !painkillers = "painkillers";
//
//    ?assista.act:find(?assista,);
//}

() = load["?actor delivers ?physobj to ?base at ?loc"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?base:base, edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?place:place, edu.tufts.hrilab.fol.Symbol ?pose:pose){
    conditions : {
        pre : free(?base);
        pre : holding(?actor, ?physobj);
        pre : at(?base, ?place);
        pre : at(?actor, ?pose);
        pre : above(?pose, ?place);
    }
    effects : {
        success : free(?actor);
        success : not(free(?base));
        success : not(holding(?actor, ?physobj));
        success : at(?physobj, ?place);
    }

    javax.vecmath.Point3d !pt;
    javax.vecmath.Quat4d !qt;
    (!qt) = op:newObject("javax.vecmath.Quat4d",0,0,0,1);
    (!pt) = op:newObject("javax.vecmath.Point3d",0,0,-0.15);

    //act:goToEEPose(?pose);
    (!pt) = op:newObject("javax.vecmath.Point3d",0,0,-0.19);
    act:moveToRelative("manipulator", !pt, !qt);
    act:openGripper("manipulator");
    (!pt) = op:newObject("javax.vecmath.Point3d",0,0,0.19);
    act:moveToRelative("manipulator", !pt, !qt);

    op:log("info","?actor delivered ?physobj to ?base at ?place");
}

() = unload["?actor retrieves ?physobj from ?base at ?loc"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?base:base, edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?place:place, edu.tufts.hrilab.fol.Symbol ?pose:pose){
    conditions : {
        pre : free(?actor);
        pre : at(?base, ?place);
        pre : at(?physobj, ?place);
        pre : at(?actor, ?pose);
        pre : above(?pose, ?place);
    }
    effects : {
        success : free(?actor);
        success : not(free(?base));
        success : at(?physobj, ?place);
        success : holding(?actor, ?physobj);
    }
    op:log(info, "finished unloading");

}
() = deliver["?actor delivers a lunch box of ?recipe"](edu.tufts.hrilab.fol.Symbol ?recipe, edu.tufts.hrilab.fol.Symbol ?location) {

   edu.tufts.hrilab.fol.Term !tempMod;
   (!tempMod) = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "none(none)");

   act:deliver(?recipe,!tempMod,?location);

}


//Wrapper script that is used to assemble a "package"
//TODO:rename this to something more general? maybe assemblePackage
() = deliver["?actor delivers a lunch box of ?recipe"](edu.tufts.hrilab.fol.Symbol ?recipe, edu.tufts.hrilab.fol.Term ?mod, edu.tufts.hrilab.fol.Symbol ?location) {

  edu.tufts.hrilab.fol.Predicate !goalPred;
  long !goalID;
  edu.tufts.hrilab.action.goal.GoalStatus !goalStatus;
  edu.tufts.hrilab.fol.Predicate !g;

  (!goalPred)=op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "did(goToLocation(robotone:agent,drop_off,true))");

  //use ?recipe to get contains pred from belief
//  (!goalPred) =act:createRecipeGoalUngrounded(?recipe, ?mod);

  //submit goal to planner to assemble desired lunchbox
  //%and(contains(box,object_1), contains(box,object_4), contains(box,object_7))
//  state:!goalPred;
    act:submitGoal(!goalPred, !goalID);
//    act:joinOnGoal(!goalID, !goalStatus);
}

() = bindPose["asserts that ?pose is  from ?pose2"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?pose:pose, edu.tufts.hrilab.fol.Symbol ?place:place) {
    edu.tufts.hrilab.fol.Term !toAssert;

    //brad: this is a belief assertion because if it was a success effect it would get brought into the planner
    op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "above(?pose,?place)", !toAssert);
    act:assertBelief(!toAssert);
}

