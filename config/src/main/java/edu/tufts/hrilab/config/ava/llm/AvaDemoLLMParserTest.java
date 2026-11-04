package edu.tufts.hrilab.config.ava.llm;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.manager.KioskGoalManager;
import edu.tufts.hrilab.ava.MockAvaComponent;
import edu.tufts.hrilab.ava.gui.AvaGuiProvider;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.llm.serviceproviders.tr.AvaLLMServiceProvider;
import edu.tufts.hrilab.llm.serviceproviders.tr.PickAndPlace2024LLMServiceProvider;
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
  String service = "openai";
  String model = "gaunernst/gemma-3-12b-it-int4-awq";
  String openaiEndpoint = "http://100.113.66.111:8000";
  boolean usePrivateEndpoint = false; //Do not set to true, use arg
  private String locationRefs = "refs/locations.json";
  private String speakerId = "";
  LLMParserComponent llmParser;
  List<Float> inf_times = new ArrayList();
  boolean run_japanese_tests = false; //Holding off on adding corresponding prompts for translations

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

    if (run_japanese_tests) {
      createInstance(PickAndPlace2024LLMServiceProvider.class, "-japanese");
    } else {
      createInstance(AvaLLMServiceProvider.class,"");
    }
    String llmComponentArgs = String.format("-service %s -model %s -temperature 0.6", service, model);
    if (service.equals("llamahf")) {
      if (usePrivateEndpoint) {
        llmComponentArgs += " -endpoint https://cj3ymkcv5luz6soh.us-east-1.aws.endpoints.huggingface.cloud";
      }
    } else if (service.equals("azure")) {
      llmComponentArgs += " -endpoint https://tracs-southcentralus-cjavc.southcentralus.inference.ml.azure.com/score";
    } else if (service.equals("openai") && !openaiEndpoint.isEmpty()) {
      llmComponentArgs += " -endpoint " + openaiEndpoint;
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
      Utterance.Builder inputUtterance;
      if (run_japanese_tests) {
        inputUtterance = new Utterance.Builder()
                .setSpeaker(Factory.createSymbol("brad"))
                .setAddressee(Factory.createSymbol("robotone"))
                .setWords(Arrays.asList(utterance.split(" ")));
      } else {
         inputUtterance = new Utterance.Builder()
                .setSpeaker(Factory.createSymbol("eric"))
                .setAddressee(Factory.createSymbol("ava"))
                .setWords(Arrays.asList(utterance.split(" ")));
      }


      long startTime = System.currentTimeMillis();
      Utterance parseResult = llmParser.parseUtterance(inputUtterance.build());
      float inf_time = (System.currentTimeMillis() - startTime) / 1000f;
      inf_times.add(inf_time);

      String outcome = parseResult.toString();
      if (outcome.equals(desired)) {
        log.info("outcome: " + outcome);
        log.info("desired: " + desired + "\n");
      } else {
        log.error("outcome: " + outcome);
        log.error("desired: " + desired + "\n");
      }
      log.info("inf time: {}", inf_time);
      num_correct += outcome.equals(desired) ? 1 : 0;
    }
    log.info("{}/{}: {}", num_correct, num_runs, utterance);
    return num_correct;
  }

  private void runLLMParserTest() {
    boolean run_easy = false;
    boolean run_medium = false;
    boolean run_hard = false;
    boolean run_japanese_documented = false;
    boolean run_japanese_translated = false;
    if (run_japanese_tests) {
      run_japanese_documented = true;
      run_japanese_translated = true;
    } else {
      run_easy = true;
      run_medium = true;
      run_hard = true;
    }

    int num_runs = 1; //TODO: see if we can force clearing cached context. Seems to return the same result every time if senf in succession
    int easy_count = 0;
    int medium_count = 0;
    int hard_count = 0;
    int japanese_documented_count = 0;
    int japanese_translated_count = 0;
    List<Float> easy_inf_times = new ArrayList<>();
    List<Float> medium_inf_times = new ArrayList<>();
    List<Float> hard_inf_times = new ArrayList<>();
    List<Float> japanese_documented_inf_times = new ArrayList<>();
    List<Float> japanese_translated_inf_times = new ArrayList<>();

    if (run_easy) {
      inf_times = new ArrayList<>();
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
              "INSTRUCT(eric,ava,stopFollowing(ava),{})"
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
              "INSTRUCT(eric,ava,startSchedulingTask(ava),{})"
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
              "INSTRUCT(eric,ava,escortFromLocationToLocation(ava,eric,VAR0:location,VAR1:location),{pavilion(VAR0:location),reception(VAR1:location),DEFINITE(VAR0:location),DEFINITE(VAR1:location)})"
              ,num_runs
      );
      count_total += num_runs;
      easy_count = count_correct;
      easy_inf_times = new ArrayList<>(inf_times);

      log.info("EASY: {}/{}", count_correct, count_total);
    }

    if (run_medium) {
      int count_correct = 0;
      int count_total = 0;
      inf_times = new ArrayList<>();
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
              "INSTRUCT(eric,ava,stopFollowing(ava),{})"
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
              "INSTRUCT(eric,ava,startSchedulingTask(ava),{})"
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
              "INSTRUCT(eric,ava,escortFromLocationToLocation(ava,eric,VAR0:location,VAR1:location),{pavilion(VAR0:location),reception(VAR1:location),DEFINITE(VAR0:location),DEFINITE(VAR1:location)})"
              ,num_runs
      );
      count_total += num_runs;
      medium_count = count_correct;
      medium_inf_times = new ArrayList<>(inf_times);

      log.info("MEDIUM: {}/{}", count_correct, count_total);
    }

    if (run_hard) {
      int count_correct = 0;
      int count_total = 0;
      inf_times = new ArrayList<>();
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
              "INSTRUCT(eric,ava,stopFollowing(ava),{})"
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
              "INSTRUCT(eric,ava,startSchedulingTask(ava),{})"
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
              "INSTRUCT(eric,ava,escortFromLocationToLocation(ava,eric,VAR0:location,VAR1:location),{pavilion(VAR0:location),reception(VAR1:location),DEFINITE(VAR0:location),DEFINITE(VAR1:location)})"
              ,num_runs
      );
      count_total += num_runs;
      hard_count = count_correct;
      hard_inf_times = new ArrayList<>(inf_times);

      log.info("HARD: {}/{}", count_correct, count_total);
    }
    
    if (run_japanese_documented) {
      int count_correct = 0;
      int count_total = 0;
      inf_times = new ArrayList<>();
      count_correct += testUtterance("ポーズ2番にしてください",
              "INSTRUCT(brad,robotone,goToPose(robotone,VAR0:pose),{location two(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("グリッパーを開いてください",
              "INSTRUCT(brad,robotone,openGripper(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("グリッパーを閉めてください",
              "INSTRUCT(brad,robotone,closeGripper(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("青いブロックをみつけてください",
              "INSTRUCT(brad,robotone,detectObject(robotone,VAR0:physobj),{blue block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("物体を拾ってください",
              "INSTRUCT(brad,robotone,pickUp(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを拾ってください\n",
              "INSTRUCT(brad,robotone,pickUp(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("物体を置いてください",
              "INSTRUCT(brad,robotone,putDown(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを置いてください",
              "INSTRUCT(brad,robotone,putDown(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを場所1から場所2に移動してください",
              "INSTRUCT(brad,robotone,moveObjectTo(robotone,VAR0:physobj,VAR1:pose,VAR2:pose),{red block(VAR0:physobj),location one(VAR1:pose),location two(VAR2:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose),DEFINITE(VAR2:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを場所1に置いてください",
              "INSTRUCT(brad,robotone,placeObjectDownAt(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("物体を場所1番に置いてください",
              "INSTRUCT(brad,robotone,placeObjectDownAt(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを場所1にしてください",
              "INSTRUCT(brad,robotone,getObjectTo(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("swap the red block and the yellow block",
              "INSTRUCT(brad,robotone,swapObjects(robotone,VAR0:physobj,VAR1:physobj),{red block(VAR0:physobj),yellow block(VAR1:physobj),DEFINITE(VAR0:physobj),DEFINITE(VAR1:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("現在のゴールを一時停止してください",
              "INSTRUCT(brad,robotone,suspendSystemGoals(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("前ゴールを再採用してください",
              "INSTRUCT(brad,robotone,resumeSystemGoals(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("現在のゴールを中止してください",
              "INSTRUCT(brad,robotone,cancelSystemGoals(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("すべてのゴールを中止してください",
              "INSTRUCT(brad,robotone,cancelAllCurrentGoals(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックは場所1にあります",
              "INSTRUCT(brad,robotone,believeObjectAt(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックはもう場所1にありません",
              "STATEMENT(brad,robotone,not(at(VAR0:physobj,VAR1:pose)),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

//        count_correct += testUtterance("eric is a supervisor",
//                "STATEMENT(brad,robotone,supervisor(eric),{})",
//                "brad",
//                "robotone"
//        ));
//        count_correct += testUtterance("eric is not a supervisor",
//                "STATEMENT(brad,robotone,not(supervisor(eric)),{})",
//                "brad",
//                "robotone"
//        ));
//        count_correct += testUtterance("eric is an admin",
//                "STATEMENT(brad,robotone,admin(eric),{})",
//                "brad",
//                "robotone"
//        ));
//        count_correct += testUtterance("eric is not an admin",
//                "STATEMENT(brad,robotone,not(admin(eric)),{})",
//                "brad",
//                "robotone"
//        ));
//        count_correct += testUtterance("eric is trusted",
//                "STATEMENT(brad,robotone,trusted(eric),{})",
//                "brad",
//                "robotone"
//        ));
//        count_correct += testUtterance("eric is not trusted",
//                "STATEMENT(brad,robotone,not(trusted(eric)),{})",
//                "brad",
//                "robotone"
//        ));

      count_correct += testUtterance("赤いブロックを場所1にしてくださいはできますか",
              "QUESTION(brad,robotone,knowHowTo(robotone,getObjectTo(robotone,VAR0:physobj,VAR1:pose)),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックは見えますか",
              "QUESTION(brad,robotone,see(VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("場所1にブロックはありますか",
              "QUESTION(brad,robotone,objectsAt(VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("場所1に赤いブロックはありますか",
              "QUESTION(brad,robotone,at(VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックはどこにありますか",
              "INSTRUCT(brad,robotone,describeObjectPose(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("位置を把握している物体をすべて教えてください",
              "INSTRUCT(brad,robotone,describeObjectPoses(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("場所1にどのブロックがありますか",
              "INSTRUCT(brad,robotone,describeObjectsAt(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("場所1に物体は何個ありますか",
              "INSTRUCT(brad,robotone,describeNumberOfObjectsAt(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("where are you",
              "INSTRUCT(brad,robotone,describeCurrentPose(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("現在のゴールは何でしょうか",
              "INSTRUCT(brad,robotone,describeCurrentGoal(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("並んでいるゴールを教えてください",
              "INSTRUCT(brad,robotone,describePendingGoals(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロック",
              "REPLY(brad,robotone,objectType(VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("場所1",
              "REPLY(brad,robotone,pose(VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("何でもないです",
              "REPLY(brad,robotone,abort(question),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("場所なし",
              "REPLY(brad,robotone,pose(none),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("オブジェクトがありません",
              "REPLY(brad,robotone,objectType(none),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("はい",
              "REPLY(brad,robotone,val(yes),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("いいえ",
              "REPLY(brad,robotone,val(no),{})",
              num_runs
      );
      count_total += num_runs;
      japanese_documented_count = count_correct;
      japanese_documented_inf_times = new ArrayList<>(inf_times);

      log.info("JAPANESE DOCUMENTED: {}/{}", count_correct, count_total);
    }

    if (run_japanese_translated) {
      int count_correct = 0;
      int count_total = 0;
      inf_times = new ArrayList<>();

      //Japanese input here simply taken from google translate - proper translations present in prompts
      count_correct += testUtterance("ロボット1、場所1へ移動",
              "INSTRUCT(brad,robotone,goToPose(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("場所 2 に移動します",
              "INSTRUCT(brad,robotone,goToPose(robotone,VAR0:pose),{location two(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("ロボット1、赤いブロックが見えることを確認してください",
              "INSTRUCT(brad,robotone,perceiveObject(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      //count_correct += testUtterance("赤いブロックを場所1に移動させる",
      //        "INSTRUCT(brad,robotone,getObjectTo(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
      //        "brad",
      //        "robotone"
      //));

      count_correct += testUtterance("あなたのグリッパーを位置 1 に移動します",//"アームのグリッパーを位置 1 に移動します。",
              "INSTRUCT(brad,robotone,goToPose(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("オープングリッパー",
              "INSTRUCT(brad,robotone,openGripper(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("グリッパーを開く",
              "INSTRUCT(brad,robotone,openGripper(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("グリッパーを開けて",
              "INSTRUCT(brad,robotone,openGripper(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("クローズグリッパー",
              "INSTRUCT(brad,robotone,closeGripper(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("グリッパーを閉じてください",
              "INSTRUCT(brad,robotone,closeGripper(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("グリッパーを閉じる",
              "INSTRUCT(brad,robotone,closeGripper(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを探してください",
              "INSTRUCT(brad,robotone,perceiveObject(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを探せ",
              "INSTRUCT(brad,robotone,perceiveObject(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを1つ探す",
              "INSTRUCT(brad,robotone,perceiveObject(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを検出します",
              "INSTRUCT(brad,robotone,perceiveObject(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを観察するk",
              "INSTRUCT(brad,robotone,perceiveObject(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを見つける",
              "INSTRUCT(brad,robotone,perceiveObject(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("ブロックをピックアップ",
              "INSTRUCT(brad,robotone,pickUp(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ブロックをつかむ",
              "INSTRUCT(brad,robotone,pickUp(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ブロックを手に入れる",
              "INSTRUCT(brad,robotone,pickUp(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ブロックを拾う",
              "INSTRUCT(brad,robotone,pickUp(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ブロックを1つ拾う",
              "INSTRUCT(brad,robotone,pickUp(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを拾う",
              "INSTRUCT(brad,robotone,pickUp(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックをつかむ",
              "INSTRUCT(brad,robotone,pickUp(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを取得する",
              "INSTRUCT(brad,robotone,pickUp(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを拾う",
              "INSTRUCT(brad,robotone,pickUp(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを1つ拾う",
              "INSTRUCT(brad,robotone,pickUp(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("ブロックを下ろす",
              "INSTRUCT(brad,robotone,putDown(robotone),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ブロックを下に置きます",
              "INSTRUCT(brad,robotone,putDown(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを場所1から場所2に移動する",
              "INSTRUCT(brad,robotone,moveObjectTo(robotone,VAR0:physobj,VAR1:pose,VAR2:pose),{red block(VAR0:physobj),location one(VAR1:pose),location two(VAR2:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose),DEFINITE(VAR2:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを場所 1 から場所 2 に輸送する",
              "INSTRUCT(brad,robotone,moveObjectTo(robotone,VAR0:physobj,VAR1:pose,VAR2:pose),{red block(VAR0:physobj),location one(VAR1:pose),location two(VAR2:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose),DEFINITE(VAR2:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを場所 1 から場所 2 まで取得します",
              "INSTRUCT(brad,robotone,moveObjectTo(robotone,VAR0:physobj,VAR1:pose,VAR2:pose),{red block(VAR0:physobj),location one(VAR1:pose),location two(VAR2:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose),DEFINITE(VAR2:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを場所 1 と場所 2 の間で移動します",
              "INSTRUCT(brad,robotone,moveObjectTo(robotone,VAR0:physobj,VAR1:pose,VAR2:pose),{red block(VAR0:physobj),location one(VAR1:pose),location two(VAR2:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose),DEFINITE(VAR2:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックを場所1と場所2の間で輸送する",
              "INSTRUCT(brad,robotone,moveObjectTo(robotone,VAR0:physobj,VAR1:pose,VAR2:pose),{red block(VAR0:physobj),location one(VAR1:pose),location two(VAR2:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose),DEFINITE(VAR2:pose)})",
              num_runs
      );
      count_total += num_runs;
      //Not sure if we want this to parse or not
      count_correct += testUtterance("場所1と場所2の間の赤いブロックを取得します",
              "INSTRUCT(brad,robotone,moveObjectTo(robotone,VAR0:physobj,VAR1:pose,VAR2:pose),{red block(VAR0:physobj),location one(VAR1:pose),location two(VAR2:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose),DEFINITE(VAR2:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックを位置1に置く",
              "INSTRUCT(brad,robotone,placeObjectDownAt(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("場所1に赤いブロックを置く",
              "INSTRUCT(brad,robotone,placeObjectDownAt(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("ブロックを位置1に置く",
              "INSTRUCT(brad,robotone,placeObjectDownAt(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ブロックを場所 1 に置きます",
              "INSTRUCT(brad,robotone,placeObjectDownAt(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("場所 1 に赤いブロックが欲しい",
              "INSTRUCT(brad,robotone,getObjectTo(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("場所1の赤いブロックが欲しい",
              "INSTRUCT(brad,robotone,getObjectTo(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックが位置1にあるようにします",
              "INSTRUCT(brad,robotone,getObjectTo(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;


      //TODO
      //count_correct += testUtterance("I want the red, blue, and green blocks at location one",
      //        "",
      //        "brad",
      //        "robotone"
      //));

      count_correct += testUtterance("赤いブロックは位置1にあります",
              "INSTRUCT(brad,robotone,believeObjectAt(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックは位置1に移動しました",
              "INSTRUCT(brad,robotone,believeObjectAt(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックは場所2から場所1に移動しました",
              "INSTRUCT(brad,robotone,believeObjectAt(robotone,VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックはもう位置1には存在しません",
              "STATEMENT(brad,robotone,not(at(VAR0:physobj,VAR1:pose)),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("赤いブロックは位置1から移動しました",
              "STATEMENT(brad,robotone,not(at(VAR0:physobj,VAR1:pose)),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;


      count_correct += testUtterance("ericは監督者です",
              "STATEMENT(brad,robotone,supervisor(eric),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ericは監督者ではありません",
              "STATEMENT(brad,robotone,not(supervisor(eric)),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ericは管理者です",
              "STATEMENT(brad,robotone,admin(eric),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ericは管理者ではありません",
              "STATEMENT(brad,robotone,not(admin(eric)),{})",
              num_runs
      );
      count_total += num_runs;

      //TODO: issue somewhere with knowHow parse matching early when more args are present and overloaded entries exist which take in different numbers of args
      //count_correct += testUtterance("do you know how to get the red block to location one",
      count_correct += testUtterance("赤いブロックを場所1に移動する方法を知っていますか",
              "QUESTION(brad,robotone,knowHowTo(robotone,moveObjectTo(robotone,VAR0:physobj,VAR1:pose)),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("赤いブロックが見えますか",
              "QUESTION(brad,robotone,see(VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      //count_correct += testUtterance("do you know where the red block is",
      //        "QUESTION(brad,robotone,know(robotone,at(VAR0:physobj,X)),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
      //        "brad",
      //        "robotone"
      //));
      count_correct += testUtterance("赤いブロックはどこですか",
              "INSTRUCT(brad,robotone,describeObjectPose(robotone,VAR0:physobj),{red block(VAR0:physobj),DEFINITE(VAR0:physobj)})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("作業領域にあるブロックは何ですか",
              "INSTRUCT(brad,robotone,describeObjectPoses(robotone),{})",
              num_runs
      );
      count_total += num_runs;

      count_correct += testUtterance("位置1にあるブロックは何か",
              "INSTRUCT(brad,robotone,describeObjectsAt(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("場所1にはブロックがいくつあるか",
              "INSTRUCT(brad,robotone,describeNumberOfObjectsAt(robotone,VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("位置 1 にブロックはありますか",
              "QUESTION(brad,robotone,objectsAt(VAR0:pose),{location one(VAR0:pose),DEFINITE(VAR0:pose)})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("場所1に赤いブロックがありますか",
              "QUESTION(brad,robotone,at(VAR0:physobj,VAR1:pose),{red block(VAR0:physobj),location one(VAR1:pose),DEFINITE(VAR0:physobj),DEFINITE(VAR1:pose)})",
              num_runs
      );
      count_total += num_runs;

      //count_correct += testUtterance("どのような種類のブロックを検出できますか",
      //        "INSTRUCT(brad,robotone,describeObjectTypes(robotone),{})",
      //        "brad",
      //        "robotone"
      //));

      count_correct += testUtterance("ericは監督者ですか",
              "QUESTION(brad,robotone,know(robotone,supervisor(eric)),{})",
              num_runs
      );
      count_total += num_runs;
      count_correct += testUtterance("ericは管理者ですか",
              "QUESTION(brad,robotone,know(robotone,admin(eric)),{})",
              num_runs
      );
      count_total += num_runs;
      japanese_translated_count = count_correct;
      japanese_translated_inf_times = new ArrayList<>(inf_times);

      log.info("JAPANESE TRANSLATED: {}/{}", count_correct, count_total);
    }

    int count_correct = 0;
    int count_total = 0;
    count_correct += testUtterance("go to the atrium elevators",
            "INSTRUCT(eric,ava,goToLocation(ava,VAR0:location),{atrium three elevators(VAR0:location),DEFINITE(VAR0:location)})"
            ,num_runs
    );
    count_total += num_runs;
    count_correct += testUtterance("go to the basement",
            "UNKNOWN(eric,ava,[go, to, the, basement],{})"
            ,num_runs
    );
    count_total += num_runs;
    log.info("CURRENT: {}/{}", count_correct, count_total);
    if (run_easy) {
      log.info("EASY: {}/{}", easy_count, 28*num_runs);
      log.info("EASY inf: {}", easy_inf_times);
    }
    if (run_medium) {
      log.info("MEDIUM: {}/{}", medium_count, 28*num_runs);
      log.info("MEDIUM inf: {}", medium_inf_times);
    }
    if (run_hard) {
      log.info("HARD: {}/{}", hard_count, 28*num_runs);
      log.info("HARD inf: {}", hard_inf_times);
    }
    if (run_japanese_documented) {
      log.info("JAPANESE DOCUMENTED: {}/{}", japanese_documented_count, 37*num_runs);
      log.info("JAPANESE DOCUMENTED inf: {}", japanese_documented_inf_times);
    }
    if (run_japanese_translated) {
      log.info("JAPANESE TRANSLATED: {}/{}", japanese_translated_count, 60*num_runs);
      log.info("JAPANESE TRANSLATED inf: {}", japanese_translated_inf_times);

    }
  }
}


