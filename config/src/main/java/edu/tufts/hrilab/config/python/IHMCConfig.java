package edu.tufts.hrilab.config.python;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.python.PythonWrapper;
import edu.tufts.hrilab.simspeech.SimSpeechProductionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.util.Util;

import javax.vecmath.Matrix4d;

public class IHMCConfig extends DiarcConfiguration {

  @Override
  public void runConfiguration() {

    createInstance(ListenerComponent.class);
    createInstance(TLDLParserComponent.class,
//            "-dict templatedict.dict " +
            "-dict mfawn/ihmc.dict"
    );
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class);
    createInstance(SimpleNLGComponent.class);
    createInstance(SimSpeechProductionComponent.class);
//    createInstance(MaryTTSComponent.class);

    createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture nadia_ros2.xml -cameraFrame ros2_left_optical -baseFrame base_footprint");

    createInstance(GoalManagerComponent.class,
            "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                    "-beliefinitfile agents/agents.pl "
                    + "-beliefinitfile demos.pl "
                    + "-beliefinitfile demos/ihmc.pl "
                    + "-asl core.asl "
                    + "-asl dialogue/nlg.asl "
                    + "-asl dialogue/nlu.asl "
                    + "-asl dialogue/handleSemantics.asl "
                    + "-asl demos/ihmc.asl "
                    + "-goal listen(self:agent) "
                    + "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector "
    );

//    createInstance(SimSpeechRecognitionComponent.class, "-speaker marlow -config ihmc.simspeech");

    PythonWrapper ihmc_wrapper = new PythonWrapper("ihmc.ihmc_wrapper", true);
    ihmc_wrapper.start();

    PythonWrapper ros2_tf2_wrapper = new PythonWrapper("ros2_tf2.ros2_tf2_wrapper", true);
    ros2_tf2_wrapper.start();

    Util.Sleep(10000);


//    try {
//      TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal").argTypes(Predicate.class));
//      service.call(Long.class, Factory.createPredicate("goto_primitive(self, Table1)"));
//    } catch (TRADEException e) {
//      throw new RuntimeException(e);
//    }

//    log.info("Attempting to call python getTransform service...");
//    try {
//      TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTransform").argTypes(String.class, String.class));
//      Matrix4d transform = service.call(Matrix4d.class, "ros2_left_optical", "base_link");
//      log.info("TF2 transform: {}", transform);
//    } catch (TRADEException e) {
//      log.error("getTransform exception", e);
//    } catch (Exception e) {
//      log.error("getTransform exception", e);
//    }

  }
}
