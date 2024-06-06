import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;

//====================== Setup action scripts ======================
() = init["workaround for not being able to retract facts from belief init files"](){

    Symbol !prepArea = "prep area";
    Symbol !servingArea = "serving area";
    Symbol !hotPlate = "hotplate";
    Symbol !cookTop = "cooktop";

    Symbol !conveyorHeight = "50";
    Symbol !hotPlateHeight = "58";
    Symbol !cookTopHeight = "52";
    Symbol !default = "default";
    Symbol !prepAreaPose;
    ai.thinkingrobots.mtracs.util.MPose !pose;

    edu.tufts.hrilab.fol.Term !toAssert;
    edu.tufts.hrilab.fol.Symbol !robotone="robotone:agent";

    edu.tufts.hrilab.fol.Symbol !chicken="chicken";
    edu.tufts.hrilab.fol.Symbol !corn="corn";

//TODO:brad: what is this for? how should we handle it?
    edu.tufts.hrilab.fol.Predicate !fact;
    !fact = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "at(?actor,pose_0:pose)");
    act:assertBelief(!fact);

    edu.tufts.hrilab.fol.Predicate !cameraHeight;
    !cameraHeight = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "cameraHeight(?actor,338)");
    act:assertBelief(!cameraHeight);

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", 510.00f, -100.0f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    !prepAreaPose = !robotone.tsc:recordPose(!prepArea, !pose, !conveyorHeight);
    op:log(info, "Setup !prepArea for ?actor");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", 510.00f, 100.0f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    !robotone.tsc:recordPose(!servingArea, !pose, !conveyorHeight);
    op:log(info, "Setup !servingArea for ?actor");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", 138.6f, -460.00f, 200.00f, 3.14159f, 0.0f, 3.14159f);
    !robotone.tsc:recordPose(!hotPlate, !pose, !hotPlateHeight);
    op:log(info, "Setup !hotPlate for ?actor");

    (!pose)= op:newObject("ai.thinkingrobots.mtracs.util.MPose", -217.00f, -460.00f, 200.0f, 3.14159f, 0.0f, 3.14159f);
    !robotone.tsc:recordPose(!cookTop, !pose, !cookTopHeight);
    op:log(info, "Setup !cookTop for ?actor");

    //These are here so they are done for each robot
    !robotone.tsc:openGripper();
    !robotone.act:rotateToEE(!default);

    //add descriptor -> Cognex job mappings
    //This is mainly left as example code, this is handled within CognexConsultant now

    //specify height of object to be used in calculations to determine z height
    //for running Cognex Jobs
    //objectDefinition(descriptor,zOffset,xOffset,)

    !robotone.act:defineIngredientHelper(!corn, !prepAreaPose, "cornDet");
    !robotone.act:defineIngredientHelper(!chicken, !prepAreaPose, "detectChicken");

    !robotone.act:goTo(!prepAreaPose);
}