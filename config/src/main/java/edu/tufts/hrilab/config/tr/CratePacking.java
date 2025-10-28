package edu.tufts.hrilab.config.tr;

import ai.thinkingrobots.mtracs.TRACSBackendClient;
import ai.thinkingrobots.mtracs.interfaces.CR800ComponentInterface;
import ai.thinkingrobots.mtracs.util.Util;
import com.google.gson.Gson;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.llm.LLMComponent;
import edu.tufts.hrilab.llm.serviceproviders.CratePackingLLMServiceProvider;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent;
import edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.*;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.stream.Collectors;

public class CratePacking extends DiarcConfiguration {


  private static final String TIER = "2 robot; action learning;";
  private final boolean production = true;

  private boolean mock = false;
    private CR800ComponentInterface assista;
    private SimSpeechRecognitionComponent simspeech;
    boolean useLLMParser = true;
    boolean useLLMNLG = true;
    String service = "openai";
    String model = "Qwen/Qwen3-14B-AWQ";
    String openaiEndpoint = "http://dashboard.thinkingrobots.ai:8000/qwen";
    boolean japanese = false;
    boolean usePrivateEndpoint = false; //DO NOT CHANGE THIS TO TRUE
    private boolean cachePersist = false;
    private String configFilesPrefixPath = "";
    boolean use4fr = true;
    boolean useAssista = true;
    private HybridNLGComponent hybridNLGComponent;
    private HybridParserComponent hybridParserComponent;
    private CratePackingLLMServiceProvider serviceProvider;
  private TRACSBackendClient backendClient;
  private JFrame frame;

  private boolean licenseValidated = false;
  private JTextField authKeyField;
  private String licenseKey = UNINITIALIZED_LICENSE_KEY;
  private final String liceseKeyFileName = "keyCache";

  private static final String UNINITIALIZED_LICENSE_KEY = "UNKNOWN";

  protected static final Insets buttonInsets = new Insets(10, 10, 0, 10);

  @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("mock").desc("run in mock").build());
        options.add(Option.builder("noLLM").desc("run without LLM for language").build());
        options.add(Option.builder("japanese").desc("run in Japanese").build());
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
      String currentPath;
      if (configFilesPrefixPath.equals("")) {
        currentPath = System.getProperty("user.dir");
      } else {
        currentPath = configFilesPrefixPath;
      }

      log.error(currentPath);

      loadPickAndPlaceSettings(String.valueOf(Paths.get(currentPath, "settings.json")));
      if (production) {
        licenseAndLaunch();
      }

      loadPickAndPlaceSettings(configFilesPrefixPath + "settings.json");

        createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);
        createInstance(edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent.class, "");
        createInstance(edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent.class, "-pragrules demos.prag");
        createInstance(edu.tufts.hrilab.slug.dialogue.DialogueComponent.class, "");
        createInstance(edu.tufts.hrilab.simspeech.SimSpeechProductionComponent.class, "");

        String nlgArgs = "-simpleNLG -cache -cacheName cratePacking-reverse";
        if (useLLMNLG) {
            nlgArgs += " -llm";
        }
        if (cachePersist) {
            nlgArgs += " -cachePersist";
        }
        hybridNLGComponent = createInstance(edu.tufts.hrilab.slug.nlg.hybrid.HybridNLGComponent.class, nlgArgs);

        String parserArgs = "-tldl cratePacking.dict -cache -cacheName cratePacking";
        if (useLLMParser) {
            parserArgs += " -llm";
        }
        if (cachePersist) {
            parserArgs += " -cachePersist";
        }
        hybridParserComponent = createInstance(edu.tufts.hrilab.slug.parsing.hybrid.HybridParserComponent.class, parserArgs);
        if (japanese) {
          hybridParserComponent.getCacheParser().setLanguage("Japanese");
          hybridNLGComponent.getCacheNLG().setLanguage("Japanese");
        }

        if (useLLMNLG || useLLMParser) {
            String serviceProviderArgs = "";
            if (japanese) {
                serviceProviderArgs += " -japanese";
            }
            serviceProvider = createInstance(CratePackingLLMServiceProvider.class, serviceProviderArgs);
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
        } else {
            if (useAssista) {
                assista = createInstance(ai.thinkingrobots.mtracs.CR800Component.class, "-config " + assistaConfigPath + " -poses " + locationsPath + " -objects " + objectsPath + " -groups agent:robotone:agent");
            }
        }


        String cratePackingComponentArgs =  "-crateL 560 -crateW 360 -crateH 200 " +
                "-itemL 230 -itemW 160 -itemH 60 " +
//        String cratePackingComponentArgs =  " " +
                "-tolerance 20 " +
                "-cameraAngle 0.0 -cameraHeightMM 325 -imageWidth 320 -imageHeight 260 " +
                "-groups agent:robotone:agent";
        if (mock) {
          cratePackingComponentArgs += " -mock";
        }

        //IMPORTANT: PackingVisionComponent _must_ be started before CratePackingComponent. There is a good way to do this and I'm not bothering right now (Pete)
      createInstance(edu.tufts.hrilab.packing.vision.PackingVisionComponent.class, "");
        createInstance(edu.tufts.hrilab.packing.CratePackingComponent.class, cratePackingComponentArgs);

//        if (mock) {
//          createInstance(edu.tufts.hrilab.vision.MockVisionComponent.class, "-groups agent:robotone:agent -rotX -67.5 -height 1.14");
//          createInstance(edu.tufts.hrilab.grasp.MockGraspComponent.class, "-groups agent:robotone:agent");
//        } else {
//          createInstance(edu.tufts.hrilab.vision.VisionComponent.class, "-capture rsense.xml -groups agent:robotone:agent -rotX -67.5 -height 1.14");
    //      SwingUtilities.invokeLater(() -> new CameraCalibrationTool().setVisible(true));
          //createInstance(edu.tufts.hrilab.grasp.GraspComponent.class, "-groups agent:robotone:agent");
//        }
  //      createInstance(edu.tufts.hrilab.packing.BoxOBBComponent.class,"-lowerThreshold 42 -upperThreshold 86");

        String goalManagerArgs=
                //"-executionManagerType edu.tufts.hrilab.action.manager.QueueExecutionManager " +
                "-executionManagerType edu.tufts.hrilab.action.manager.SingleGoalExecutionManager " +
                "-beliefinitfile demos.pl " +
                        "-beliefinitfile agents/onRobotScrewingAgents.pl " +
                        "-beliefinitfile demos/onRobotScrewing.pl " +
//                        "-selector edu.tufts.hrilab.action.selector.GoalPlanningActionSelector " +
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
                        "-asl config/edu/tufts/hrilab/action/asl/demos/cratePacking/cratePackingDialogue.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/demos/cratePacking/cratePackingInit.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/demos/onRobotScrewing/cognexArmUtil.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/mtracs/cr800Base.asl " +
                        "-asl config/edu/tufts/hrilab/action/asl/vision.asl " +

                        "-asl config/edu/tufts/hrilab/action/asl/demos/queueEM.asl " +

                        "-priorityFile default.json cratePackingPriority.json " +
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
        } else {
            if (useAssista) {
                simspeech = createInstance(SimSpeechRecognitionComponent.class, "-speaker pete -addressee robotone -config cratePacking.simspeech");
            }
        }
    }

    //Should be one of "English" or "Japanese" (case matters)
    public void setLanguage(String language) {
      log.info("Setting language to: {}", language);
      hybridNLGComponent.getCacheNLG().setLanguage(language);
      hybridParserComponent.getCacheParser().setLanguage(language);
      if (serviceProvider != null) {
        serviceProvider.setLanguage(language);
      }
      if (useAssista) {
        List<String> currentPoseNames;
        List<String> newPoseNames;
        if (language.equals("Japanese")) {
          currentPoseNames = assista.getEnglishPoseDescriptors();
          newPoseNames = assista.getJapanesePoseDescriptors();
        } else {
          currentPoseNames = assista.getJapanesePoseDescriptors();
          newPoseNames = assista.getEnglishPoseDescriptors();
        }
        for (int i=0; i<currentPoseNames.size(); i++) {
          assista.updatePoseName(Factory.createSymbol(currentPoseNames.get(i),"pose"), Factory.createSymbol(newPoseNames.get(i),"pose"));
        }
      }
    }

    private void toggleLanguage() {
      japanese = !japanese;
      setLanguage(japanese ? "Japanese" : "English");
    }

    private void buildContent(JFrame aFrame){
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        //panel.add(new JLabel("TRACS RUNNING"));
        JPanel innerPanel = new JPanel();
        JCheckBox languageToggle = new JCheckBox();
        JLabel languageToggleLabel = new JLabel("Set Language to Japanese: ");
        languageToggle.addActionListener(e -> toggleLanguage());
        innerPanel.add(languageToggleLabel);
        innerPanel.add(languageToggle);
        panel.add(innerPanel);
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

    //for distribution

    private void licenseAndLaunch() {
      this.backendClient = new TRACSBackendClient();
      this.buildAndDisplayGui();
      while (!licenseValidated) {
        try {
          Thread.sleep(500);
        } catch (InterruptedException e) {
          throw new RuntimeException(e);
        }
      }
    }


    private void buildAndDisplayGui() {
      frame = new JFrame("TRACS Launcher");
      frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);

      frame.addWindowListener(new WindowAdapter() {
        public void windowClosing(WindowEvent e) {
          frame.setVisible(false);
          try {
            backendClient.uploadLogs(licenseKey);
          } catch (IOException ex) {
            log.error("Unable to upload logs");
          }
          frame.dispose();
          System.exit(0);
        }
      });

      Container container = frame.getContentPane();

      JPanel mainPanel = new JPanel();
      mainPanel.setLayout(new BoxLayout(mainPanel, BoxLayout.Y_AXIS));

      JPanel entryPanel = new JPanel();
      entryPanel.setLayout(new GridBagLayout());

      int gridY = 0;

      JLabel authKeyLabelString = new JLabel("Product Key");
      addComponent(entryPanel, authKeyLabelString, 0, gridY, 1, 1, buttonInsets, GridBagConstraints.LINE_START, GridBagConstraints.HORIZONTAL);

      authKeyField = new JTextField(20);
      addComponent(entryPanel, authKeyField, 1, gridY++, 1, 1, buttonInsets, GridBagConstraints.LINE_START, GridBagConstraints.HORIZONTAL);

      mainPanel.add(entryPanel);

      JPanel buttonPanel = new JPanel();
      buttonPanel.setLayout(new FlowLayout());

      String licenseKeyPath;
      try {
        String diarcPath = Util.ensureDiarcDirPath();
        licenseKeyPath = String.valueOf(Paths.get(diarcPath, liceseKeyFileName));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        licenseKey = readCachedLicenseKey(licenseKeyPath);
        if (!licenseKey.equals(UNINITIALIZED_LICENSE_KEY)) {
          authKeyField.setText(licenseKey);
        }
      } catch (IOException e) {
        log.error("Failed to read cached license key value"); //todo: should this pop up a dialogue window?
      }

      JButton okButton = new JButton("OK");
      okButton.setAlignmentX(JButton.RIGHT_ALIGNMENT);
      okButton.addActionListener(new ActionListener() {
        @Override
        public void actionPerformed(ActionEvent event) {
          String rawKeyFromField = authKeyField.getText();
          licenseKey = validateLicenseKey(rawKeyFromField, TIER);
          if (!licenseKey.equals(UNINITIALIZED_LICENSE_KEY)) {
            licenseValidated = true;
            try {
              cacheLicenseKey(licenseKey, licenseKeyPath);
            } catch (IOException e) {
              log.error("Failed to cache the license key value"); //todo: should this pop up a dialogue window?
            }
            authKeyField.setEnabled(false);
            okButton.setEnabled(false);
          }
        }
      });

      buttonPanel.add(okButton);

      mainPanel.add(buttonPanel);

      container.add(mainPanel);
      frame.pack();
      frame.setVisible(true);
      okButton.requestFocusInWindow();
    }

    private void uploadLogFileOnWindowClose(String licenseKey) {
      try {
        backendClient.uploadLogs(licenseKey);
      } catch (IOException e) {
        log.error("Failed to upload logs"); //todo: should this pop up a dialogue window?
      }
    }

    private void cacheLicenseKey(String licenseKey, String filePath) throws IOException {
      try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
        writer.write(licenseKey);
        writer.newLine();
      }
    }

    private String readCachedLicenseKey(String filePath) throws IOException {
      File f = new File(filePath);
      String toReturn = null;
      if (f.isFile()) {
        try (BufferedReader reader = new BufferedReader(new FileReader(f))) {
          toReturn = reader.readLine();
        }
      }
      if (toReturn != null) {
        return toReturn;
      } else {
        return UNINITIALIZED_LICENSE_KEY;
      }
    }

    private void addComponent(Container container, Component component,
                              int gridx, int gridy, int gridwidth, int gridheight, Insets insets,
                              int anchor, int fill) {
      GridBagConstraints gbc = new GridBagConstraints(gridx, gridy,
              gridwidth, gridheight, 1.0D, 1.0D, anchor, fill, insets, 0, 0);
      container.add(component, gbc);
    }

    private String validateLicenseKey(String key, String tier) {
      String sanitizedKey = key.trim().toLowerCase();
      boolean valid = false;
      try {
        valid = backendClient.validateLicense(sanitizedKey, tier);
      } catch (IOException e) {
        JOptionPane.showMessageDialog(frame, "Failed to validate license. Please verify license key and network connection.");
      }
      if (!valid) {
        JOptionPane.showMessageDialog(frame, "Error: Invalid license key.");
        return UNINITIALIZED_LICENSE_KEY;
      } else {
        return sanitizedKey;
      }
    }
}
