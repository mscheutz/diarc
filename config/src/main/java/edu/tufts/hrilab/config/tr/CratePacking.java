package edu.tufts.hrilab.config.tr;

import ai.thinkingrobots.mtracs.interfaces.CR800ComponentInterface;
import com.google.gson.Gson;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.llm.hf.PickAndPlace2024LLMServiceProvider;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.swing.*;
import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.stream.Collectors;

public class CratePacking extends DiarcConfiguration {
    private boolean mock = false;
    private CR800ComponentInterface assista;
    private CR800ComponentInterface rv4fr;
    private SimSpeechRecognitionComponent simspeech;
    boolean useLLMParser = true;
    boolean useLLMNLG = true;
    String service = "openai";
    String model = "Qwen/Qwen3-14B-AWQ";
    String openaiEndpoint = "http://dashboard.thinkingrobots.ai:8000/qwen";
    boolean japanese = false;
    boolean usePrivateEndpoint = false; //DO NOT CHANGE THIS TO TRUE
    String languageGenerationASLFile = "config/edu/tufts/hrilab/action/asl/demos/pickAndPlace/pickAndPlaceLanguageGeneration.asl";
    private boolean cachePersist = false;
    private String configFilesPrefixPath = "/home/pete/dev/mtracs/crate_packing/";
    boolean use4fr = true;
    boolean useAssista = true;

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("mock").desc("run in mock").build());
        options.add(Option.builder("noLLM").desc("run without LLM for language").build());
        options.add(Option.builder("japanese").desc("run in mock").build());
        options.add(Option.builder("privateEndpoint").desc("run using privately hosted endpoint rather than free serverless endpoint").build());
        options.add(Option.builder("configFilesPrefixPath").desc("prefix path for config files").hasArg().argName("path").build());
        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        if (cmdLine.hasOption("noLLM")) {
            log.info("Running without LLM for language");
            useLLMParser = false;
            useLLMNLG = false;
        }
        if (cmdLine.hasOption("mock")) {
            log.info("Starting with mock...");
            mock = true;
        }
        if (cmdLine.hasOption("japanese")) {
            log.info("Configuring to run in japanese");
            japanese = true;
        }
        if (cmdLine.hasOption("privateEndpoint")) {
            log.warn("You have configured the system to use the privately hosted HuggingFace endpoint. THIS COSTS MONEY AND SHOULD NOT BE USED FOR TESTING. If you are doing this intentionally, please enter 1 into the console");
            System.out.println("Enter: ");
            Scanner reader = new Scanner(System.in);
            int in = reader.nextInt();
            if (in == 1) {
                log.warn("using private endpoint");
                usePrivateEndpoint = true;
            } else {
                log.warn("not using private endpoint");
            }
        }
        if (cmdLine.hasOption("configFilesPrefixPath")) {
            configFilesPrefixPath = cmdLine.getOptionValue("configFilesPrefixPath");
        }
    }

    @Override
    public void runConfiguration() {
        loadPickAndPlaceSettings(configFilesPrefixPath + "settings.json");

        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);
        createInstance(edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent.class, "");
        createInstance(edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent.class, "-pragrules demos.prag");
        createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class, "");
        createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class, "");

        String nlgArgs = "-simpleNLG -cache -cacheName pickAndPlace2024-reverse";
        if (useLLMNLG) {
            nlgArgs += " -llm";
        }
        if (cachePersist) {
            nlgArgs += " -cachePersist";
        }
        createInstance(edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent.class, nlgArgs);

        String parserArgs = "-tldl cratePacking.dict -cache -cacheName cratePacking";
        if (useLLMParser) {
            parserArgs += " -llm";
        }
        if (cachePersist) {
            parserArgs += " -cachePersist";
        }
        createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, parserArgs);

        if (useLLMNLG || useLLMParser) {
            String serviceProviderArgs = "";
            if (japanese) {
                serviceProviderArgs += " -japanese";
            }
            createInstance(PickAndPlace2024LLMServiceProvider.class, serviceProviderArgs);
            String llmComponentArgs = String.format("-service %s llamahf -model %s Qwen/Qwen3-14B -temperature 0.6 0.6", service, model);
            if (service.equals("llamahf")) {
                if (usePrivateEndpoint) {
                    llmComponentArgs += " -endpoint https://z6xvb4wfqwn59s1r.us-east-1.aws.endpoints.huggingface.cloud";
                }
            } else if (service.equals("azure")) {
                llmComponentArgs += " -endpoint https://tracs-southcentralus-lezah.southcentralus.inference.ml.azure.com/score";
            } else if (service.equals("openai") && !openaiEndpoint.isEmpty()) {
                llmComponentArgs += " -endpoint " + openaiEndpoint;
            }
            createInstance(LLMComponent.class, llmComponentArgs);
        }

        String locationsPath = configFilesPrefixPath + "locations.json";
        String objectsPath = configFilesPrefixPath + "objects.json";
        String assistaConfigPath = configFilesPrefixPath + "5AS.json"; //todo this should be passed in as a command line arg or should be configurable in the settings file?
        String rv4frConfigPath = configFilesPrefixPath + "RV4FR.json";

        if (mock) {
            if (useAssista) {
                assista = createInstance(ai.thinkingrobots.mtracs.mock.MockCR800Component.class, "-config " + assistaConfigPath + " -poses " + locationsPath + " -objects " + objectsPath + " -groups agent:robotone:agent");
            }
            if (use4fr) {
                rv4fr = createInstance(ai.thinkingrobots.mtracs.mock.MockCR800Component.class, "-config " + rv4frConfigPath + " -poses " + locationsPath + " -objects " + objectsPath + " -groups agent:robottwo:agent");
            }
        } else {
            if (useAssista) {
                assista = createInstance(ai.thinkingrobots.mtracs.CR800Component.class, "-config " + assistaConfigPath + " -poses " + locationsPath + " -objects " + objectsPath + " -groups agent:robotone:agent");
            }
            if (use4fr) {
                rv4fr = createInstance(ai.thinkingrobots.mtracs.CR800Component.class, "-config " + rv4frConfigPath + " -poses " + locationsPath + " -objects " + objectsPath + " -groups agent:robottwo:agent");
            }
        }
        if (!mock) {
          createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture rsense.xml -groups agent:robotone:agent -rotX -67.5 -height 1.1");
          createInstance(edu.tufts.hrilab.grasp.GraspComponent.class, "-groups agent:robotone:agent");
        }

        if (japanese) {
            languageGenerationASLFile = "config/edu/tufts/hrilab/action/asl/demos/pickAndPlace/pickAndPlaceLanguageGenerationJapanese.asl";
        }
        String goalManagerArgs=
                "-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                "-beliefinitfile demos.pl " +
                        "-beliefinitfile agents/onRobotScrewingAgents.pl " +
                        "-beliefinitfile demos/onRobotScrewing.pl " +
                        "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector " +
//                        "-selector edu.tufts.hrilab.action.selector.UtilitarianActionSelector " +

                        "-dbfileDir / " +
                        "-asl config/edu/tufts/hrilab/action/asl/core.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/handleSemantics.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/nlu.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/dialogue/nlg.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/cognex.asl " +

                        //todo: should move these asl files and sort into ones that are/are not specific to screwing most/all of this is actually general to the robot arm demos w the cognex and M arms
                        "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/assistaCognex.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/cognexArmActions.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/demos/cratePacking/cratePackingDemo.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/cratePacking/cratePackingInit.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/demos/cratePacking/cratePackingInit.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/cognexArmUtil.asl " +
                        "-asl " + languageGenerationASLFile + " " +

                        "-asl config/edu/tufts/hrilab/action/asl/mtracs/cr800Base.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/vision.asl " +

                        //TODO:brad this has to come after the agents file
                        "-goal listen(self) ";
        if (useAssista) {
            goalManagerArgs += "-goal initCratePacking(self) ";
        }

        //goalManagerArgs +="-editor";

        createInstance(edu.tufts.hrilab.action.GoalManagerComponent.class, goalManagerArgs);

        if (japanese) {
            if (useAssista) {
                simspeech = createInstance(SimSpeechRecognitionComponent.class, "-speaker たたら -addressee robotone -config pickAndPlace.simspeech");
            }
            if (use4fr) {
                simspeech = createInstance(SimSpeechRecognitionComponent.class, "-speaker たたら -addressee robottwo -config pickAndPlace.simspeech");
            }
        } else {
            if (useAssista) {
                simspeech = createInstance(SimSpeechRecognitionComponent.class, "-speaker pete -addressee robotone -config cratePacking.simspeech");
            }
            if (use4fr) {
                simspeech = createInstance(SimSpeechRecognitionComponent.class, "-speaker pete -addressee robottwo -config pickAndPlace.simspeech");
            }
        }
        this.buildAndDisplayGui();
    }


    private void buildAndDisplayGui(){
        JFrame frame = new JFrame("TRACS");
        buildContent(frame);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setVisible(true);
    }

    private void buildContent(JFrame aFrame){
        JPanel panel = new JPanel();
        panel.add(new JLabel("TRACS RUNNING"));
        aFrame.getContentPane().add(panel);
    }

    private void loadPickAndPlaceSettings(String path) {
        Gson gson = new Gson();
        StringBuilder json = new StringBuilder();
        InputStream is;
        BufferedReader br = null;
        try {
            is = new FileInputStream(path);
            br = new BufferedReader(new InputStreamReader(is));
        } catch (FileNotFoundException e) {
            log.warn("[loadLocationConfig] error loading location configuration from file at path" + path);
        }
        List<String> jsonList= br.lines().collect(Collectors.toList());
        for (String s : jsonList)
            json.append(s);
        PickAndPlaceSettings settings = gson.fromJson(json.toString(), PickAndPlaceSettings.class);
        japanese = settings.useJapanese;
        cachePersist = settings.persistCache;
        useAssista = settings.assista;
        use4fr = settings.rv4fr;
    }

    private class PickAndPlaceSettings {

      public boolean useJapanese;
      public boolean persistCache;
      public boolean assista;
      public boolean rv4fr;
    }
}
