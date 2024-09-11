/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.temi.MockTemiV3Component;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;

public class TemiV3Mock extends DiarcConfiguration {
  protected static Logger log = LoggerFactory.getLogger(TemiV3Mock.class);
  public SimSpeechRecognitionComponent simspeech;
  public MockTemiV3Component temi;
  public TLDLParserComponent parser;
  public ReferenceResolutionComponent rr;
  public PragmaticsComponent prag;
  public DialogueComponent dialogue;
  public SimpleNLGComponent nlg;
  public GoalManagerComponent gm;

  @Override
  public void runConfiguration() {
    simspeech = createInstance(SimSpeechRecognitionComponent.class, "-nogui -speaker brad");

    temi = createInstance(MockTemiV3Component.class);
    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);
    parser = createInstance(TLDLParserComponent.class, "-dict temi.dict");
    rr = createInstance(ReferenceResolutionComponent.class);
    prag = createInstance(PragmaticsComponent.class, "-pragrules demos.prag");

    dialogue = createInstance(DialogueComponent.class);
    nlg = createInstance(SimpleNLGComponent.class);
    gm = createInstance(GoalManagerComponent.class,
//                "-editor " +
                "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                "-beliefinitfile demos.pl " +
                "-beliefinitfile agents/agents.pl " +
                "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector " +
                "-dbfile core.asl " +
                "-dbfile dialogue/nlg.asl " +
                "-dbfile dialogue/nlu.asl " +
                "-dbfile dialogue/handleSemantics.asl " +
                "-dbfile temi/temi.asl " +
                "-goal listen(self:agent)"
        );
    }


  public static void main(String[] args) {
    TemiV3Mock diarcConfig = new TemiV3Mock();
    diarcConfig.runConfiguration();
  }

}
