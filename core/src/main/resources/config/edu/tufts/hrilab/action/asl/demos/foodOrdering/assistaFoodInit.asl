import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;

//====================== Setup action scripts ======================
() = init["workaround for not being able to retract facts from belief init files"](){

    Predicate !tmp;
    Symbol !prepArea = "prepArea";
    Symbol !servingArea = "servingArea";
    Symbol !hotPlate = "hotPlate";
    Symbol !cookTop = "cookTop";
    Symbol !pantry = "pantry";

    Symbol !conveyorHeight = "50";
    Symbol !hotPlateHeight = "58";
    Symbol !cookTopHeight = "52";
    Symbol !default = "default";
    Symbol !prepAreaPose;
    Symbol !servingAreaPose;
    Symbol !pantryAreaPose;
    Symbol !hotPlatePose;
    Symbol !cookTopPose;
    Symbol !pantryPose;
    edu.tufts.hrilab.mtracs.util.MPose !pose;

    edu.tufts.hrilab.fol.Term !toAssert;
    edu.tufts.hrilab.fol.Symbol !assista="assista:agent";
    Symbol !human= "human:mobileManipulator";

    edu.tufts.hrilab.fol.Symbol !corn="corn";

//TODO:brad: what is this for? how should we handle it?
    edu.tufts.hrilab.fol.Predicate !fact;
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,pose_0:pose)");
    act:assertBelief(!fact);

    edu.tufts.hrilab.fol.Predicate !cameraHeight;
    !cameraHeight = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,338)");
    act:assertBelief(!cameraHeight);
    !cameraHeight = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(!assista,338)");
    act:assertBelief(!cameraHeight);


    (!pose)= op:newObject("edu.tufts.hrilab.mtracs.util.MPose", 510.00f, -100.0f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    !prepAreaPose = !assista.tsc:recordPose(!prepArea, !pose, !conveyorHeight);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!assista, !prepAreaPose)");
    act:assertBelief(!tmp);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!human, !prepAreaPose)");
    act:assertBelief(!tmp);
    op:log(info, "Setup !prepArea for ?actor");

    (!pose)= op:newObject("edu.tufts.hrilab.mtracs.util.MPose", 510.00f, 100.0f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    !servingAreaPose = !assista.tsc:recordPose(!servingArea, !pose, !conveyorHeight);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!assista, !servingAreaPose)");
    act:assertBelief(!tmp);
    op:log(info, "Setup !servingArea for ?actor");

    (!pose)= op:newObject("edu.tufts.hrilab.mtracs.util.MPose", 138.6f, -460.00f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    !hotPlatePose = !assista.tsc:recordPose(!hotPlate, !pose, !hotPlateHeight);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!assista, !hotPlatePose)");
    act:assertBelief(!tmp);
    op:log(info, "Setup !hotPlate for ?actor");

    (!pose)= op:newObject("edu.tufts.hrilab.mtracs.util.MPose", -217.00f, -460.00f, 200.0f, 3.14159f, 0.0f, 3.14159f);
    !cookTopPose = !assista.tsc:recordPose(!cookTop, !pose, !cookTopHeight);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!assista, !cookTopPose)");
    act:assertBelief(!tmp);
    op:log(info, "Setup !cookTop for ?actor");

    //TODO: make pantry it's own real pose
    !pantryPose = !assista.tsc:recordPose(!pantry, !pose, !cookTopHeight);
    !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!human, !pantryPose)");
    act:assertBelief(!tmp);

    //These are here so they are done for each robot
    !assista.tsc:openGripper();
    !assista.act:rotateToEE(!default);

    //add descriptor -> Cognex job mappings
    //This is mainly left as example code, this is handled within CognexConsultant now

    //specify height of object to be used in calculations to determine z height
    //for running Cognex Jobs
    //objectDefinition(descriptor,zOffset,xOffset,)

    !assista.act:defineIngredientHelper(!corn, !prepAreaPose, "cornDet");
    !assista.act:defineIngredientHelper("servingBox",!servingAreaPose,"detBox");

    !assista.act:goTo(!prepAreaPose);
}

() = getIn(Symbol ?item:physobj, Symbol ?destination:physobj) {
    goal:itemOn(?item, ?destination);
}