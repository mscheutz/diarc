package edu.tufts.hrilab.config.ava.llm;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.manager.KioskGoalManager;
import edu.tufts.hrilab.ava.MockAvaComponent;
import edu.tufts.hrilab.ava.gui.AvaGuiProvider;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.llm.serviceproviders.AvaLLMServiceProvider;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.listen.ListenerComponent;
import edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent;
import edu.tufts.hrilab.slug.parsing.llm.LLMParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import edu.tufts.hrilab.tts.MaryTTSComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//TODO: build proper LLMParserTest framework
public class AvaDemoLLMParserTest extends DiarcConfiguration {
  boolean useQueueEM = true;
  String avaArgs = "";
  String service = "llamahf";
  String model = "Llama-3.1-8B-Instruct";
  boolean usePrivateEndpoint = false; //Do not set to true, use arg
  private String locationRefs = "refs/locations.json";
  private String speakerId = "";
  LLMParserComponent llmParser;


  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("privateEndpoint").desc("run using privately hosted endpoint rather than free serverless endpoint").build());
    options.add(Option.builder("locationRefs").hasArg().desc("set location references file").build());
    options.add(Option.builder("speakerId").hasArg().desc("name of speaker to route local tts through").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("locationRefs")) {
      locationRefs = cmdLine.getOptionValue("locationRefs");
    }
    if (cmdLine.hasOption("speakerId")) {
      speakerId = cmdLine.getOptionValue("speakerId");
    }
    if (cmdLine.hasOption("privateEndpoint")) {
      log.warn("You have configured the system to use the privately hosted HuggingFace endpoint.");// THIS COSTS MONEY AND SHOULD NOT BE USED FOR FRIVOLOUS TESTING. If you are doing this intentionally, please enter 1 into the console");
      usePrivateEndpoint = true;
    }
  }
  public void runConfiguration() {
    llmParser = createInstance(LLMParserComponent.class, "");
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(DialogueComponent.class);
    String nlgArgs = "-simpleNLG -cache -cacheName ava-reverse -cachePersist";
    nlgArgs += " -llm";
    DiarcComponent.createInstance(HybridNLGComponent.class, nlgArgs);
    createInstance(ListenerComponent.class);

    createInstance(AvaLLMServiceProvider.class,"");
    String llmComponentArgs = String.format("-service %s -model %s -temperature 0.6", service, model);
    if (service.equals("llamahf")) {
      if (usePrivateEndpoint) {
        llmComponentArgs += " -endpoint https://cj3ymkcv5luz6soh.us-east-1.aws.endpoints.huggingface.cloud";
      }
    } else if (service.equals("azure")) {
      llmComponentArgs += " -endpoint https://tracs-southcentralus-cjavc.southcentralus.inference.ml.azure.com/score";
    }
    createInstance(LLMComponent.class, llmComponentArgs);

    createInstance(MaryTTSComponent.class, " -female -volume 0.0 -showMixers");
    String mapArgs = "";
    mapArgs += "-refs " + locationRefs;
    createInstance(MapComponent.class, mapArgs);

    createInstance(MockAvaComponent.class, avaArgs);

    String kioskArgs = "";

    createInstance(GoalManagerComponent.class,
            "-historylength 3600000 "
                    +   "-goal listen(self) monitorBattery(ava) " +//listenForWWandASR(ava) " +
                    "-beliefinitfile demos.pl agents/ava.pl"
                    + " -dbfile " +
                    "core.asl " +
                    "manipulation.asl " +
                    "dialogue/nlg.asl " +
                    "dialogue/nlu.asl " +
                    "dialogue/handleSemantics.asl " +
                    "vision.asl " +
                    "ava/ava.asl " +
                    "ava/scheduledGoals.asl " +
                    "ava/avaMedia.asl " +
                    "ava/avaASR.asl " +
                    "ava/avaTMCDemo.asl " +
                    "demos/queueEM.asl"
                    + " -priorityFile default.json avaPriority.json"
                    + (useQueueEM ? " -executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager" : ""));

    createInstance(KioskGoalManager.class, kioskArgs);

    createInstance(AvaGuiProvider.class);

    log.info("Finished Ava Demo Config");
    runLLMParserTest();
    System.exit(0);
  }

  private int testUtterance(String utterance, String desired, int num_runs) {
    int num_correct = 0;
    for (int i=0;i<num_runs;i++) {
      Utterance.Builder inputUtterance = new Utterance.Builder()
              .setSpeaker(Factory.createSymbol("eric"))
              .setAddressee(Factory.createSymbol("ava"))
              .setWords(Arrays.asList(utterance.split(" ")));


      Utterance parseResult = llmParser.parseUtterance(inputUtterance.build());

      String outcome = parseResult.toString();
      if (outcome.equals(desired)) {
        log.info("outcome: " + outcome);
        log.info("desired: " + desired + "\n");
      } else {
        log.error("outcome: " + outcome);
        log.error("desired: " + desired + "\n");
      }
      num_correct += outcome.equals(desired) ? 1 : 0;
    }
    log.info("{}/{}: {}", num_correct, num_runs, utterance);
    return num_correct;
  }

  private void runLLMParserTest() {
    boolean run_easy = true;
    boolean run_medium = true;
    boolean run_hard = true;
    int num_runs = 5;
    int easy_count = 0;
    int medium_count = 0;
    int hard_count = 0;

    if (run_easy) {
      int count_correct = 0;
      int count_total = 0;
      count_correct += testUtterance("follow me",
              "INSTRUCT(eric,ava,follow(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("save this location as room one",
              "INSTRUCT(eric,ava,saveLocation(ava,room one),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("stop following me",
              "INSTRUCT(eric,ava,stopFollow(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("go to the pavilion",
              "INSTRUCT(eric,ava,goToLocation(ava,VAR0:location),{pavilion(VAR0:location),DEFINITE(VAR0:location)})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("pause",
              "INSTRUCT(eric,ava,suspendSystemGoalsBlocking(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("resume",
              "INSTRUCT(eric,ava,resumeSystemGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("cancel your current task",
              "INSTRUCT(eric,ava,cancelSystemGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what is your current goal",
              "INSTRUCT(eric,ava,describeCurrentGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what are your pending goals",
              "INSTRUCT(eric,ava,describePendingGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what are your scheduled goals",
              "INSTRUCT(eric,ava,describeScheduledGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I will teach you a new task",
              "INSTRUCT(eric,ava,learnNewTask(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("the task name is greet people",
              "REPLY(eric,ava,taskName(greet people),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I am done teaching",
              "INSTRUCT(eric,ava,endActionLearning(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("where are you right now",
              "INSTRUCT(eric,ava,describeCurrentLocation(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what locations do you have saved",
              "INSTRUCT(eric,ava,describeAllKnownLocations(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what patrol paths do you have saved",
              "INSTRUCT(eric,ava,describeAllKnownPatrolPaths(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("add a new patrol path demo path",
              "INSTRUCT(eric,ava,definePatrolPath(ava,demo path),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("the pavilion",
              "REPLY(eric,ava,location(VAR0:location),{pavilion(VAR0:location),DEFINITE(VAR0:location)})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("that is all",
              "REPLY(eric,ava,location(none),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("add a new scheduled task",
              "INSTRUCT(eric,ava,scheduleGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("the task is to stand",
              "REPLY(eric,ava,taskPredicate(stand(ava)),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("in 30 minutes",
              "REPLY(eric,ava,relativeTime(30,minutes),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("yes",
              "REPLY(eric,ava,val(yes),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("no",
              "REPLY(eric,ava,val(no),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("modify a scheduled task",
              "INSTRUCT(eric,ava,modifyScheduledGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("the interval",
              "REPLY(eric,ava,timingOf(interval),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("the first execution",
              "REPLY(eric,ava,timingOf(execution),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("escort eric from the pavilion to reception",
              "INSTRUCT(eric,ava,escortFromToLocation(ava,eric,VAR0:location,VAR1:location),{pavilion(VAR0:location),reception(VAR1:location),DEFINITE(VAR0:location),DEFINITE(VAR1:location)})"
              ,num_runs
      );
      count_total += num_runs;
      easy_count = count_correct;

      log.info("EASY: {}/{}", count_correct, count_total);
    }

    if (run_medium) {
      int count_correct = 0;
      int count_total = 0;
      count_correct += testUtterance("come with me",
              "INSTRUCT(eric,ava,follow(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("save this as room one",
              "INSTRUCT(eric,ava,saveLocation(ava,room one),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("stop coming with me",
              "INSTRUCT(eric,ava,stopFollow(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("navigate your way to the pavilion",
              "INSTRUCT(eric,ava,goToLocation(ava,VAR0:location),{pavilion(VAR0:location),DEFINITE(VAR0:location)})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("hold on with what you are doing",
              "INSTRUCT(eric,ava,suspendSystemGoalsBlocking(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("continue with what you were doing",
              "INSTRUCT(eric,ava,resumeSystemGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("stop what you are doing",
              "INSTRUCT(eric,ava,cancelSystemGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what are you currently doing?",
              "INSTRUCT(eric,ava,describeCurrentGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what are you planning on doing next?",
              "INSTRUCT(eric,ava,describePendingGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what do you have scheduled",
              "INSTRUCT(eric,ava,describeScheduledGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I will teach you something new",
              "INSTRUCT(eric,ava,learnNewTask(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("it is called \"greet people\"",
              "REPLY(eric,ava,taskName(greet people),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("that is how you greet people",
              "INSTRUCT(eric,ava,endActionLearning(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("at what location do you find yourself",
              "INSTRUCT(eric,ava,describeCurrentLocation(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what locations are you aware of",
              "INSTRUCT(eric,ava,describeAllKnownLocations(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what patrol paths are you aware of",
              "INSTRUCT(eric,ava,describeAllKnownPatrolPaths(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I'm going to teach you a new patrol path demo path",
              "INSTRUCT(eric,ava,definePatrolPath(ava,demo path),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("pavilion",
              "REPLY(eric,ava,location(VAR0:location),{pavilion(VAR0:location),DEFINITE(VAR0:location)})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("no location",
              "REPLY(eric,ava,location(none),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I want to schedule a new task",
              "INSTRUCT(eric,ava,scheduleGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("the scheduled task will be to stand",
              "REPLY(eric,ava,taskPredicate(stand(ava)),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("about 30 minutes from now",
              "REPLY(eric,ava,relativeTime(30,minutes),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("of course",
              "REPLY(eric,ava,val(yes),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("don't think so",
              "REPLY(eric,ava,val(no),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I want to change the timing of a scheduled task",
              "INSTRUCT(eric,ava,modifyScheduledGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("The time between executions",
              "REPLY(eric,ava,timingOf(interval),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("when it is executed",
              "REPLY(eric,ava,timingOf(execution),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("bring eric to reception, he's at the pavilion",
              "INSTRUCT(eric,ava,escortFromToLocation(ava,eric,VAR0:location,VAR1:location),{pavilion(VAR0:location),reception(VAR1:location),DEFINITE(VAR0:location),DEFINITE(VAR1:location)})"
              ,num_runs
      );
      count_total += num_runs;
      medium_count = count_correct;

      log.info("MEDIUM: {}/{}", count_correct, count_total);
    }

    if (run_hard) {
      int count_correct = 0;
      int count_total = 0;
      count_correct += testUtterance("stay with me",
              "INSTRUCT(eric,ava,follow(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("remember room one is here",
              "INSTRUCT(eric,ava,saveLocation(ava,room one),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("you can stay behind now",
              "INSTRUCT(eric,ava,stopFollow(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("i want you under the pavilion",
              "INSTRUCT(eric,ava,goToLocation(ava,VAR0:location),{pavilion(VAR0:location),DEFINITE(VAR0:location)})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("wait a second",
              "INSTRUCT(eric,ava,suspendSystemGoalsBlocking(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("okay, you can go again",
              "INSTRUCT(eric,ava,resumeSystemGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("drop what you're doing",
              "INSTRUCT(eric,ava,cancelSystemGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what are you up to?",
              "INSTRUCT(eric,ava,describeCurrentGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what are you doing after this?",
              "INSTRUCT(eric,ava,describePendingGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what are your daily tasks?",
              "INSTRUCT(eric,ava,describeScheduledGoals(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("Let me show you how to do something",
              "INSTRUCT(eric,ava,learnNewTask(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I'm going to show you how to greet people",
              "REPLY(eric,ava,taskName(greet people),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("you should be all set to greet people now if you followed all that",
              "INSTRUCT(eric,ava,endActionLearning(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("where you at?",
              "INSTRUCT(eric,ava,describeCurrentLocation(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("where can you go?",
              "INSTRUCT(eric,ava,describeAllKnownLocations(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("what can you patrol?",
              "INSTRUCT(eric,ava,describeAllKnownPatrolPaths(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I'll show you the test path",
              "INSTRUCT(eric,ava,definePatrolPath(ava,test path),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("the shaded area outside",
              "REPLY(eric,ava,location(VAR0:location),{pavilion(VAR0:location),DEFINITE(VAR0:location)})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("I forget where",
              "REPLY(eric,ava,location(none),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("Can you do something for me later?",
              "INSTRUCT(eric,ava,scheduleGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("it'll be to stand up",
              "REPLY(eric,ava,taskPredicate(stand(ava)),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("Let's say right after lunch? I usually only eat for around 30 minutes",
              "REPLY(eric,ava,relativeTime(30,minutes),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("let me check with my manager, but that sounds good",
              "REPLY(eric,ava,val(yes),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("Maybe another day, sorry",
              "REPLY(eric,ava,val(no),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("Could I actually get you to do that another time?",
              "INSTRUCT(eric,ava,modifyScheduledGoal(ava),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("how often it happens",
              "REPLY(eric,ava,timingOf(interval),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("when it happens next",
              "REPLY(eric,ava,timingOf(execution),{})"
              ,num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("can you find eric and show him to reception? i think he's by the pavilion.",
              "INSTRUCT(eric,ava,escortFromToLocation(ava,eric,VAR0:location,VAR1:location),{pavilion(VAR0:location),reception(VAR1:location),DEFINITE(VAR0:location),DEFINITE(VAR1:location)})"
              ,num_runs
      );
      count_total += num_runs;
      hard_count = count_correct;

      log.info("HARD: {}/{}", count_correct, count_total);
    }

    int count_correct = 0;
    int count_total = 0;
    count_correct += testUtterance("go to the atrium elevators",
            "INSTRUCT(eric,ava,goToLocation(ava,VAR0:location),{atrium three elevators(VAR0:location),DEFINITE(VAR0:location)})"
            ,num_runs
    );
    count_total += num_runs;
    count_correct += testUtterance("go to the basement",
            "INSTRUCT(eric,ava,goToLocation(ava,VAR0:location),{atrium 3 elevators(VAR0:location),DEFINITE(VAR0:location)})"
            ,num_runs
    );
    count_total += num_runs;
    log.info("CURRENT: {}/{}", count_correct, count_total);
    if (run_easy) {
      log.info("EASY: {}/{}", easy_count, 28*num_runs);
    }
    if (run_medium) {
      log.info("MEDIUM: {}/{}", medium_count, 28*num_runs);
    }
    if (run_hard) {
      log.info("HARD: {}/{}", hard_count, 28*num_runs);
    }
  }
}


