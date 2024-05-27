import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;

() = init["simulates environmental setup"]() {
    Predicate !tmp;
   //robot identifiers
   Symbol !human= "human:mobileManipulator";
   Symbol !yumiL = "leftArm:yumi";
   Symbol !yumiR = "rightArm:yumi";

   Symbol !leftSafePose;
   Symbol !rightSafePose;

   //location reference local vars
   Symbol !pantryRef;
   Symbol !tableOneRef;

   Symbol !pantryArea;
   Symbol !servingArea;
   Symbol !sauceArea;
   Symbol !boxArea;
   Symbol !prepArea;
   Symbol !cooktopArea;
   Symbol !hotplateArea;
   Symbol !noneArea;

   !yumiL.act:calibrateGripper();
   !yumiR.act:calibrateGripper();

   //Save locations
   !pantryRef= act:saveLocation("\"pantry location\"");
   !tableOneRef= act:saveLocation("\"table one\"");

   !noneArea = !yumiL.act:addArea("\"safe area\"", !tableOneRef); //todo: this is used just for the camera pattern. should remove this.
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiL, !noneArea, !tableOneRef)");
   act:assertBelief(!tmp);
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiR, !noneArea, !tableOneRef)");
   act:assertBelief(!tmp);

   //workcell areas
   !pantryArea = !yumiL.act:addArea("pantry", !pantryRef); //human only
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!human, !pantryArea, !pantryRef)");
   act:assertBelief(!tmp);

   !servingArea = !yumiL.act:addArea("servingArea", !tableOneRef); //both arms
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiL, !servingArea,!tableOneRef)");
   act:assertBelief(!tmp);
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiR, !servingArea,!tableOneRef)");
   act:assertBelief(!tmp);

   !sauceArea = !yumiL.act:addArea("\"sauce area\"", !tableOneRef); //left
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiL, !sauceArea , !tableOneRef)");
   act:assertBelief(!tmp);

   !boxArea = !yumiL.act:addArea("\"box area\"", !tableOneRef); //left
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiL, !boxArea, !tableOneRef)");
   act:assertBelief(!tmp);

   !prepArea = !yumiL.act:addArea("prepArea", !tableOneRef); //right and human
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiR, !prepArea ,!tableOneRef)");
   act:assertBelief(!tmp);
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!human, !prepArea,!tableOneRef)");
   act:assertBelief(!tmp);

   !cooktopArea = !yumiL.act:addArea("cookTop", !tableOneRef); //right
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiR, !cooktopArea , !tableOneRef)");
   act:assertBelief(!tmp);

   !hotplateArea = !yumiL.act:addArea("hotPlate", !tableOneRef); //right
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "reachable(!yumiR, !hotplateArea , !tableOneRef)");
   act:assertBelief(!tmp);

   //safe poses
   !leftSafePose = !yumiL.act:saveCameraPose("carry", "[[27.19,507.57,570.98],[0.501972,-0.511133,-0.471277,-0.514455],[0,-1,2,4],[91.5657,9E+09,9E+09,9E+09,9E+09,9E+09]]", !noneArea);
   !yumiL.act:goToCameraPose(!leftSafePose);
   !rightSafePose = !yumiR.act:saveCameraPose("carry","[[180.50,-507.96,573.45],[0.00558492,-0.707077,-0.00557768,-0.707092],[0,0,0,4],[2.6532,9E+09,9E+09,9E+09,9E+09,9E+09]]", !noneArea);
   !yumiR.act:goToCameraPose(!rightSafePose);

    //primary poses
   !yumiL.act:saveCameraPose("\"camera pose serving area left\"", "[[480.00,131.06,186.94],[0.000185906,-0.0147232,-0.999892,-1.89815E-05],[1,2,1,0],[-110.753,9E+09,9E+09,9E+09,9E+09,9E+09]]", !servingArea);
   !yumiL.act:saveCameraPose("\"camera pose sauce area\"", "[[268.39,443.34,315.23],[0.00405957,-0.682573,0.730784,0.00571689],[0,-2,1,4],[-123.295,9E+09,9E+09,9E+09,9E+09,9E+09]]",  !sauceArea);
   !yumiL.act:saveCameraPose("\"camera pose box area\"","[[-83.98,393.56,316.98],[0.00268627,0.705239,-0.708959,-0.00270396],[0,-1,1,4],[90.6635,9E+09,9E+09,9E+09,9E+09,9E+09]]",  !boxArea);
   !yumiL.act:saveCameraPose("\"camera pose sauce pour\"","[[300.54,125.78,141.99],[0.00808252,0.0127641,-0.999846,0.00887834],[0,-2,1,0],[-95.0695,9E+09,9E+09,9E+09,9E+09,9E+09]]",  !servingArea);

   !yumiL.act:saveAreaPose("\"dropoff pose serving area left\"", "[[493.12,95.41,55.00],[0.00661954,-0.999227,-0.0322726,-0.0214439],[1,1,1,0],[-123.642,9E+09,9E+09,9E+09,9E+09,9E+09]]",  !servingArea);
   !yumiL.act:saveAreaPose("\"dropoff pose box area left\"", "[[-139.46,339.05,46.60],[0.0254128,0.713193,0.700505,-0.00142569],[0,-3,1,5],[141.943,9E+09,9E+09,9E+09,9E+09,9E+09]]",  !boxArea);
   !yumiL.act:saveAreaPose("\"dropoff pose sauce area left\"", "[[279.11,392.38,-55.00],[0.0174587,-0.99935,0.000278645,-0.0315538],[-1,0,-1,5],[-164.736,9E+09,9E+09,9E+09,9E+09,9E+09]]",  !sauceArea);

   !yumiR.act:saveCameraPose("\"camera pose cooktop\"", "[[54.93,-430.77, 355.00],[0.00063941,-0.999838,-0.0165616,-0.00704752],[0,-1,1,4],[-148.32,9E+09,9E+09,9E+09,9E+09,9E+09]]", !cooktopArea);
   !yumiR.act:saveCameraPose("\"camera pose hot plate\"", "[[438.67,-422.16,360.00],[0.0221556,0.999604,0.0171554,0.00266289],[0,0,0,0],[65.6809,9E+09,9E+09,9E+09,9E+09,9E+09]]" , !hotplateArea);
   !yumiR.act:saveCameraPose("\"camera pose prep area right\"", "[[357.48,-131.58,300.02],[6.82791E-05,0.0104291,0.999946,9.40719E-06],[0,2,1,4],[127.733,9E+09,9E+09,9E+09,9E+09,9E+09]]" ,!prepArea);
   !yumiR.act:saveCameraPose("\"camera pose serving area right\"", "[[400.47,92.17,300.02],[4.00877E-05,0.0104353,0.999946,-2.53899E-06],[2,2,0,4],[127.756,9E+09,9E+09,9E+09,9E+09,9E+09]]",!servingArea);

   !yumiR.act:saveAreaPose("\"dropoff pose serving area\"", "[[383.79,116.57,82.97],[0.00644578,-0.734494,0.678536,-0.00809856],[2,1,2,5],[162.166,9E+09,9E+09,9E+09,9E+09,9E+09]]", !servingArea);
   !yumiR.act:saveAreaPose("\"dropoff pose prep area\"","[[319.73,-87.62,-47.36],[0.0493485,0.711518,-0.700491,-0.0248732],[1,0,2,5],[150.909,9E+09,9E+09,9E+09,9E+09,9E+09]]", !prepArea);
   !yumiR.act:saveAreaPose("\"dropoff pose cooktop\"", "[[129.22,-403.38,-2.24],[0.00688705,0.725712,0.687864,0.0117789],[0,0,-1,4],[-146.666,9E+09,9E+09,9E+09,9E+09,9E+09]]",  !cooktopArea);
   !yumiR.act:saveAreaPose("\"dropoff pose hot plate\"", "[[418.88,-410.78,15.50],[0.0281348,-0.712138,-0.701463,0.00407477],[1,0,0,4],[121.19,9E+09,9E+09,9E+09,9E+09,9E+09]]",  !hotplateArea);

   //define ingredients
   act:defineIngredientHelper("\"bell pepper\"",!pantryArea,"detBellPepper");
   act:defineIngredientHelper("corn",!prepArea,"detCorn");
   act:defineIngredientHelper("carrot",!pantryArea,"detCarrot");
//   act:defineIngredientHelper("chicken",!pantryArea,"detChicken");

   act:defineIngredientHelper("servingBox",!servingArea,"detBox");
   act:defineIngredientHelper("\"chipotle sauce\"",!sauceArea,"detChipotle");

   !yumiR.act:setGraspPoint("\"bell pepper\"", "[[-8.13165, -2.23473, -321.049],[0.0206969, -0.629805, 0.775078, -0.0465877],[1, 2, 0, 4],[-168.617, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiR.act:setGraspPoint(carrot, "[[-16.7559, 5.54001, -316.32],[0.0402978, -0.678297, 0.732182, -0.0468918],[1, 2, -1, 4],[-108.713, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiR.act:setGraspPoint(corn,  "[[-5.42153, 2.64947, -318.484],[0.0161432, -0.695156, 0.717344, -0.0437575],[1, 2, -1, 4],[-134.688, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
//   !yumiR.act:setGraspPoint(chicken, "[[11.633, 5.5744, -309.718],[0.0410188, -0.741056, 0.668755, -0.0438137],[1, 2, -1, 4],[-110.298, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");

   !yumiL.act:setGraspPoint("servingBox", "[[11.8661, -187.919, -291],[0.0216773, 0.700944, -0.712725, 0.0151954],[0, 0, -1, 5],[160.682, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiL.act:setGraspPoint(aioli, "[[7.91812, -9.29623, -332.264],[0.0259458, 0.743849, -0.667522, 0.0207388],[-1, -2, 0, 5],[-158.425, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiL.act:setGraspPoint("\"chipotle sauce\"", "[[7.91812, -9.29623, -332.264],[0.0259458, 0.743849, -0.667522, 0.0207388],[-1, -2, 0, 5],[-158.425, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");

   //set up scene
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtLocation(!yumiL, !tableOneRef)");
   act:assertBelief(!tmp);
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtLocation(!yumiR, !tableOneRef)");
   act:assertBelief(!tmp);
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtLocation(!human, !pantryRef)");
   act:assertBelief(!tmp);

   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtArea(!yumiL, !noneArea)");
   act:assertBelief(!tmp);
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtArea(!yumiR, !noneArea)");
   act:assertBelief(!tmp);
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "agentAtArea(!human, !pantryArea)");
   act:assertBelief(!tmp);

   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "safe(!yumiL, !noneArea)");
   act:assertBelief(!tmp);
   !tmp = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "safe(!yumiR, !noneArea)");
   act:assertBelief(!tmp);
}

//TODO: actually replace or differentiate between "getIn" vs "getOn" - messes with LLM when going from "get X in the Y" to getOn(x,y)
() = getIn(Symbol ?item:physobj, Symbol ?destination:physobj) {
    goal:itemOn(?item, ?destination);
}