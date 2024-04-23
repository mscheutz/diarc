import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.fol.Term;

() = init["simulates environmental setup"]() {
   //robot identifiers
   Symbol !mobileYumi = "human:mobileyumi";
   Symbol !yumiL = "left:yumi";
   Symbol !yumiR = "right:yumi";

   //location reference local vars
   Symbol !pantryRef;
   Symbol !tableOneRef;
   Symbol !deliveryRef;

   //pose reference local vars
   Symbol !boxAreaRef;
   Symbol !sauceOneRef;
   Symbol !sauceTwoRef;
   Symbol !cookTopRef;
   Symbol !hotPlate;
   Symbol !ingredientPoseRef;
   
   Symbol !servingAreaRefLeft;
   Symbol !servingAreaRefRight;
   Symbol !prepAreaRefRight;
   Symbol !prepAreaRefMobile;

   Symbol !deliveryTableRef;

   !yumiL.act:calibrateGripper();
   !yumiR.act:calibrateGripper();

   //Save locations
   !pantryRef= !mobileYumi.act:saveLocationHelper("\"pantry location\"");
   !tableOneRef= !mobileYumi.act:saveLocationHelper("\"table one\"");

   //Save poses
   !prepAreaRefMobile= !mobileYumi.act:savePoseHelper("\"prep area\"", "" ,!tableOneRef);
   !ingredientPoseRef= !mobileYumi.act:savePoseHelper("pantry","" ,!pantryRef);

   !yumiL.act:savePoseHelper("carry", "[[34.08,394.92,650.80],[0.501986,-0.511092,-0.471294,-0.514468],[0,0,2,4],[91.832,9E+09,9E+09,9E+09,9E+09,9E+09]]", !tableOneRef);
   !yumiL.act:goToCameraPose(leftpose_0:leftpose);
   !yumiR.act:savePoseHelper("carry","[[180.50,-507.96,573.45],[0.00558492,-0.707077,-0.00557768,-0.707092],[0,0,0,4],[2.6532,9E+09,9E+09,9E+09,9E+09,9E+09]]", !tableOneRef);
   !yumiR.act:goToCameraPose(rightpose_0:rightpose);


   !servingAreaRefLeft = !yumiL.act:savePoseHelper("\"serving area left\"", "[[402.14,21.09,372.46],[0.0132955,0.706915,0.707031,0.0141785],[-1,-3,-1,5],[-151.303,9E+09,9E+09,9ECamera+09,9E+09,9E+09]]" ,!tableOneRef);
   !sauceOneRef =        !yumiL.act:savePoseHelper("\"sauce area one\"", "[[268.39,443.34,315.23],[0.00405957,-0.682573,0.730784,0.00571689],[0,-2,1,4],[-123.295,9E+09,9E+09,9E+09,9E+09,9E+09]]", !tableOneRef);
   !sauceTwoRef=         !yumiL.act:savePoseHelper("\"sauce area two\"", "[[414.95,443.34,315.22],[0.00405494,-0.682567,0.730789,0.00571011],[0,-2,1,4],[-123.296,9E+09,9E+09,9E+09,9E+09,9E+09]]", !tableOneRef);
   !boxAreaRef=          !yumiL.act:savePoseHelper("\"box area\"","[[-83.98,393.56,316.98],[0.00268627,0.705239,-0.708959,-0.00270396],[0,-1,1,4],[90.6635,9E+09,9E+09,9E+09,9E+09,9E+09]]", !tableOneRef);
   !yumiL.act:savePoseHelper("\"sauce pour pose\"","[[300.54,125.78,141.99],[0.00808252,0.0127641,-0.999846,0.00887834],[0,-2,1,0],[-95.0695,9E+09,9E+09,9E+09,9E+09,9E+09]]", !tableOneRef);

   !yumiL.act:addDropoffPose("\"left dropoff pose\"", "\"box area\"", "[[-139.46,339.05,46.60],[0.0254128,0.713193,0.700505,-0.00142569],[0,-3,1,5],[141.943,9E+09,9E+09,9E+09,9E+09,9E+09]]");
   !yumiL.act:addDropoffPose("\"front dropoff pose\"", "\"serving area left\"", "[[493.12,95.41,55.00],[0.00661954,-0.999227,-0.0322726,-0.0214439],[1,1,1,0],[-123.642,9E+09,9E+09,9E+09,9E+09,9E+09]]");
   !yumiL.act:addDropoffPose("\"sriracha put down\"", "\"sauce area one\"", "[[219.11,392.38,-55.00],[0.0174587,-0.99935,0.000278645,-0.0315538],[-1,0,-1,5],[-164.736,9E+09,9E+09,9E+09,9E+09,9E+09]]");
   !yumiL.act:addDropoffPose("\"chipotle put down\"", "\"sauce area two\"", "[[320.43,392.37,-55.00],[0.017426,-0.999353,0.000265536,-0.031448],[-1,-1,0,5],[-158.392,9E+09,9E+09,9E+09,9E+09,9E+09]]");


   !cookTopRef=          !yumiR.act:savePoseHelper("\"cook top\"", "[[54.93,-430.77, 355.00],[0.00063941,-0.999838,-0.0165616,-0.00704752],[0,-1,1,4],[-148.32,9E+09,9E+09,9E+09,9E+09,9E+09]]",!tableOneRef);
   !hotPlate=            !yumiR.act:savePoseHelper("\"hot plate\"", "[[438.67,-422.16,365.00],[0.0221556,0.999604,0.0171554,0.00266289],[0,0,0,0],[65.6809,9E+09,9E+09,9E+09,9E+09,9E+09]]" ,!tableOneRef);
   !prepAreaRefRight=    !yumiR.act:savePoseHelper("\"prep area right\"", "[[357.48,-131.58,300.02],[6.82791E-05,0.0104291,0.999946,9.40719E-06],[0,2,1,4],[127.733,9E+09,9E+09,9E+09,9E+09,9E+09]]" ,!tableOneRef,!prepAreaRefMobile);
   !servingAreaRefRight= !yumiR.act:savePoseHelper("\"serving area\"", "[[357.48,-131.58,300.02],[6.82791E-05,0.0104291,0.999946,9.40719E-06],[0,2,1,4],[127.733,9E+09,9E+09,9E+09,9E+09,9E+09]]" ,!tableOneRef,!servingAreaRefLeft);

    !yumiR.act:addDropoffPose("\"right rear put down\"", "\"cook top\"", "[[129.22,-403.38,-1.24],[0.00688705,0.725712,0.687864,0.0117789],[0,0,-1,4],[-146.666,9E+09,9E+09,9E+09,9E+09,9E+09]]");
    !yumiR.act:addDropoffPose("\"put down front\"", "\"serving area\"", "[[405.33,147.86,121.75],[0.000150876,-0.00241096,0.999981,-0.00572953],[0,-1,1,0],[138.263,9E+09,9E+09,9E+09,9E+09,9E+09]]");
    !yumiR.act:addDropoffPose("\"right front put down\"", "\"hot plate\"", "[[418.88,-410.78,21.36],[0.0281348,-0.712138,-0.701463,0.00407477],[1,0,0,4],[121.19,9E+09,9E+09,9E+09,9E+09,9E+09]]");

   //Define Ingredients

   act:defineIngredientHelper("\"bell pepper\"",!ingredientPoseRef,"none","detBellPepper");
   act:defineIngredientHelper("corn",!ingredientPoseRef,"none","detCorn");
   act:defineIngredientHelper("carrot",!ingredientPoseRef,"none","detCarrot");
   act:defineIngredientHelper("chicken",!ingredientPoseRef,"none","detChicken");

   op:log(warn, "[init] box are ref: !boxAreaRef");
   act:defineIngredientHelper("\"serving box\"",!boxAreaRef,"none","detBox");
   act:defineIngredientHelper("aioli",!sauceTwoRef,"sauce","detSriracha");
   act:defineIngredientHelper("chipotle sauce",!sauceOneRef,"sauce","detChipotle");


   !yumiR.act:setGraspPoint("\"bell pepper\"", "[[-42.077, 11.7806, -312.269],[0.00399305, -0.674119, 0.738594, -0.00515414],[1, 2, 1, 4],[117.493, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiR.act:setGraspPoint(carrot, "[[-40.0503, 18.6082, -312.167],[0.00929521, -0.644688, 0.764183, -0.0177475],[1, 3, 0, 4],[141.226, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiR.act:setGraspPoint(corn, "[[-44.4883, 7.18901, -312.167],[5.97197E-4, 0.68588, -0.727715, -3.85941E-4],[1, 3, 0, 4],[166.382, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiR.act:setGraspPoint(chicken, "[[-55.184, 7.14687, -310.706],[0.00779821, -0.736959, 0.675686, -0.0167067],[1, 3, 0, 4],[147.868, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");

   !yumiL.act:setGraspPoint("\"serving box\"", "[[11.8661, -187.919, -291],[0.0216773, 0.700944, -0.712725, 0.0151954],[0, 0, -1, 5],[160.682, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiL.act:setGraspPoint(aioli, "[[7.91812, -9.29623, -332.264],[0.0259458, 0.743849, -0.667522, 0.0207388],[-1, -2, 0, 5],[-158.425, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");
   !yumiL.act:setGraspPoint("\"chipotle sauce\"", "[[7.91812, -9.29623, -332.264],[0.0259458, 0.743849, -0.667522, 0.0207388],[-1, -2, 0, 5],[-158.425, 9.0E9, 9.0E9, 9.0E9, 9.0E9, 9.0E9]]");

}
