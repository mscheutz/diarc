() = pickup["?actor grabs ?physobj"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?physobj:physobj, edu.tufts.hrilab.fol.Symbol ?pose:pose, edu.tufts.hrilab.fol.Symbol ?place:place, edu.tufts.hrilab.fol.Symbol ?dest:physobj) {
    conditions : {
        pre : free(?actor);
        //pre: currenteetype(?actor,gripper); TODO:brad: not needed for multiRobotMedkit, needed for Automate
        pre : at(?physobj,?place);
        pre : at(?actor, ?pose);
        pre : above(?pose, ?place);
        pre : found(?dest);

    }
    effects : {
        success : holding(?actor, ?physobj);
        success : not(free(?actor));
        success : not(at(?physobj,?place));
        nonperf : not(at(?actor,?pose));
        nonperf : unknownlocation(?actor);
    }

    javax.vecmath.Point3d !pt;
    javax.vecmath.Point3d !pt2;
    javax.vecmath.Quat4d !qt;
    java.lang.String !arm = "manipulator";

    (!qt) = op:newObject("javax.vecmath.Quat4d",0,0,0,1);
    (!pt) = op:newObject("javax.vecmath.Point3d",0,0,-0.205);
    (!pt2) = op:newObject("javax.vecmath.Point3d",0,0, 0.205);

    op:log("info", "pickup");
    act:goToEEPose(?pose);
    act:moveToRelative(!arm, !pt, !qt);
    act:closeGripper(!arm);
    act:moveToRelative(!arm, !pt2, !qt);
    op:log("info", "Successfully picked up");

}

//TODO:update this to mach deliver in reference pddl
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

//todo: gotopose
() = gotopose["moves to ?pose1, from ?pose2"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?pose1:pose, edu.tufts.hrilab.fol.Symbol ?pose2:pose, edu.tufts.hrilab.fol.Symbol ?place1:place, edu.tufts.hrilab.fol.Symbol ?place2:place) {
    conditions : {
        pre : at(?actor, ?pose2);
        pre : available(?pose1);
        pre : available(?place1);
        pre : above(?pose1, ?place1);
        pre : above(?pose2, ?place2);
        pre : accessible(?actor, ?pose1);
    }
    effects : {
        success : not(at(?actor, ?pose2));
        success : at(?actor,?pose1);
        success : not(available(?pose1));
        success : not(available(?place1));
        success : available(?pose2);
        success : available(?place2);
        nonperf : not(at(?actor,?pose1));
        nonperf : not(at(?actor,?pose2));
        nonperf : unknownlocation(?actor);
    }

    //Todo: Will: Assuming there is no meaningful usage for the "from" pose for the UR5
    act:goToEEPose(?pose1);
    op:log(info, "?actor finished going from pose ?pose2 to pose ?pose1");

}

//TODO:brad: we need a gotopose equivalent for when it's carrying something that doesn't change ee orientation, maybe called carry?
//Will: Not sure if moveObjectAbove is necessary - if we want this to be a template for carry, move the conditions and effects to the above
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

() = pack["?actor packs ?physobj into ?container"](edu.tufts.hrilab.fol.Symbol ?actor:arm, edu.tufts.hrilab.fol.Symbol ?container:container, edu.tufts.hrilab.fol.Symbol ?smallobj:smallobj, edu.tufts.hrilab.fol.Symbol ?pose:pose, edu.tufts.hrilab.fol.Symbol ?place:place, edu.tufts.hrilab.fol.Symbol ?counter:counter) {
    conditions : {
        pre : accessible(?actor, ?place);
        pre : holding(?actor, ?smallobj);
        pre : above(?pose, ?place);
        pre : at(?actor,?pose);
        pre : at(?container,?place);
        pre : found(?container);
        pre : oftype(?smallobj, ?counter);
    }
    effects : {
        success : not(holding(?actor, ?smallobj));
        success : free(?actor);
        success : in(?container,?smallobj);
        success : fluent_increase(amount, ?container, ?counter, 1);
    }

    op:log("info", "?actor finished packing ?physobj into ?container at ?place");
}