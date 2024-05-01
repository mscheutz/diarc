/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.nao;

import com.aldebaran.qimessaging.Application;
import com.aldebaran.qimessaging.Object;
import com.aldebaran.qimessaging.CallError;
import com.aldebaran.qimessaging.Future;
import com.aldebaran.qimessaging.Session;
import edu.tufts.hrilab.diarc.DiarcComponent;

import java.util.ArrayList;
import java.util.List;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;


import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class NaoComponent extends DiarcComponent implements NaoInterface {

  final protected Logger log = LoggerFactory.getLogger(this.getClass());

  protected com.aldebaran.qimessaging.Object asr;
  protected com.aldebaran.qimessaging.Object audioPlayer;
  protected com.aldebaran.qimessaging.Object soundDetect;
  protected com.aldebaran.qimessaging.Object memory;
  protected com.aldebaran.qimessaging.Object tts;
  protected com.aldebaran.qimessaging.Object sonar;
  protected com.aldebaran.qimessaging.Object motion;
  protected com.aldebaran.qimessaging.Object posture;
  protected com.aldebaran.qimessaging.Object battery;
  protected com.aldebaran.qimessaging.Object world;
  private com.aldebaran.qimessaging.Tuple tuple;

  private com.aldebaran.qimessaging.Object animSpeech;
  private com.aldebaran.qimessaging.Object leds;
  private CallBack callback;

  private Application application;
  private Session session;

  protected String naoURL = "tcp://192.168.0.195:9559";
  private List<String> testVocabulary;

  private double curTV = 0;
  private double curRV = 0;
  private double defTV = 0.1;
  private double defRV = 0.3;

  private boolean useAP = false;
  private boolean useSR = false;
  private boolean useSD = false;
  private boolean useTTS = true;
  private boolean useSONAR = true;

  private boolean unsafeMove = false; // if true, allow unsafe moves (i.e., ignore collision protection)

  private boolean wakeUp = true;

  private float MAXSONARLENGTH = 3.0f; // 2.5m, for Nao V4 and below, 5m for Nao V5
  private float MINSONARLENGTH = 0.5f; // 0.5m

  private boolean sim = false;
  private boolean isSpeaking = false;

  private final double maxTV = 0.3;
  private final double maxRV = 0.5;

  private String voice;
  //@Override
  //public boolean goToPosture(String targetPosture) {
  //  log.trace("goToPosture()method called");
  //  try {
  //    posture.call("goToPosture", targetPosture, 0.5);
  //  } catch (CallError ce) {
  //    log.error("Call Error ", ce);
  //    return false;
  //  }

  //  return true;
  //}

  // constructor
  public NaoComponent() {
    super();
  }

  @Override
  protected void init() {

    // start the Nao application
    application = new Application(null);
    session = new Session();
    try {
      Future<Void> fut = session.connect(naoURL);
      fut.get();
    } catch (Exception ie) {
      throw new RuntimeException("error connecting to session", ie);
    }

    try {
      log.trace("Loading modules.");

      posture = session.service("ALRobotPosture");
      log.trace("ALRobotPosture loaded.");

      motion = session.service("ALMotion");
      log.trace("ALMotion loaded.");

      battery = session.service("ALBattery");
      log.trace("ALBattery loaded.");

      world = session.service("ALWorldRepresentation");
      log.trace("ALWorldRepresentation loaded.");

      // initialize motion system
      //	ALMotion.wakeUp() is the following:
      //
      // If the robot is already awake when you call ALMotion.wakeUp(),
      // it only stiffens its motors. It is equivalent to
      // ALMotion.setStiffnesses("Body", 1.0).
      //
      // If the robot is not already awake and it is standing (two feet flat on the ground,
      // and body above the feet), it stiffens up and goes to posture Stand.
      //
      // If the robot is not already awake and it is not standing, it only stiffens up.
      if (wakeUp) {
        try {
          motion.call("wakeUp");
        } catch (CallError ce) {
          throw new RuntimeException("CallError during wakeUp ", ce);
        }
      }
      if (useAP) {
        audioPlayer = session.service("ALAudioPlayer");
        log.trace("ALAudioPlayer loaded");
        String naoRawURL = naoURL.substring(6).split(":")[0];
        String tmpdircommand = "ssh nao@" + naoRawURL + " mkdir /tmp/audio";
        Runtime.getRuntime().exec(tmpdircommand);
        log.trace("creating /tmp/audio on robot");
        log.debug(tmpdircommand);
      }
      if (useSR) {
        asr = session.service("ALSpeechRecognition");
        asr.call("setLanguage", "English");
        testVocabulary = new ArrayList<String>();
        testVocabulary.add("yes");
        testVocabulary.add("no");
        testVocabulary.add("please");
        testVocabulary.add("walk forward");
        testVocabulary.add("can you walk forward");
        testVocabulary.add("can you walk backwards");
        testVocabulary.add("raise your arms");
        //testVocabulary.add("walk");

        asr.call("setVocabulary", this.testVocabulary, Boolean.valueOf(true));
        //asr.call("setParameter", "- NbHypotheses", new Float(3));
        //asr.call("compile","testgrammar.txt","testgrammar.lcf","English");
        //asr.call("addContext", "testgrammar.lcf","mainContext");
        //asr.call("activateRule", "mainContext", "cmd");
        //asr.call("activateAllRules", "mainContext");
        asr.call("unsubscribe", "Test_ASR");
        //asr.call("subscribe", "Test_ASR");
        log.trace("ALSpeechRecognition loaded.");
      }
      if (useSD) {
        soundDetect = session.service("ALSoundDetection");
        log.trace("ALSoundDetection loaded.");
      }
      if (useTTS) {
        tts = session.service("ALTextToSpeech");
        log.trace("ALTextToSpeech loaded.");
      }

      animSpeech = session.service("ALAnimatedSpeech");
      leds = session.service("ALLeds");

      if (unsafeMove) {
        setExternalCollisionProtectionEnabled(false);
      }

      // initialize the Nao's callback function
      callback = new CallBack();

      memory = session.service("ALMemory");
      log.trace("ALMemory loaded.");
      Object subscriber = memory.<Object>call("subscriber", "SpeechDetected").get();
      subscriber.connect("signal::(m)", "speechDetected::(m)", callback);

      subscriber = memory.<Object>call("subscriber", "SoundDetected").get();
      subscriber.connect("signal::(m)", "soundDetected::(m)", callback);

      subscriber = memory.<Object>call("subscriber", "WordRecognized").get();
      subscriber.connect("signal::(m)", "wordRecognized::(m)", callback);

      if (useSONAR) {
        sonar = session.service("ALSonar");
        sonar.call("subscribe", "Test_sonar");
        log.trace("useSONAR");

        try {
          // not sure this is right, i.e., that the future is a "List"
          Future<Float> leftsonar = memory.call("getData", "Device/SubDeviceList/US/Left/Sensor/Value");
          Future<Float> rightsonar = memory.call("getData", "Device/SubDeviceList/US/Right/Sensor/Value");
          // assuming that the returned list has as its first element the reading...
          float left = (float) leftsonar.get();
          float right = (float) rightsonar.get();

          log.trace("SONAR OK: " + left + " " + right);
        } catch (InterruptedException e) {
          throw new RuntimeException("SONAR READING GOT INTERRUPTED...", e);
        }
      }
      //sonar.call("unsubscribe", "Test_sonar");
      //sonar.call("subscribe", "Test_sonar");

      //soundDetect.call("subscribe","test_sub");
      // initialize standing
      if (wakeUp) {
        try {
          posture.call("goToPosture", "Stand", 0.5);
        } catch (CallError ce) {
          throw new RuntimeException("CallError during goToPosture ", ce);
        }
      }

      new Thread() {
        public void run() {
          application.run();
        }
      }.start();
    } catch (Exception e) {
      throw new RuntimeException("Exception during initialization... ", e);
    }
  }

  @Override
  public void shutdownComponent() {
    log.info("Closing NaoComponent session for {} ...", getMyGroups());
    session.close();
    log.info("... session closed for {}.", getMyGroups());
  }

  public class CallBack {

    public void wordRecognized(java.lang.Object o) {
      log.trace("wordRecognized()!");
      if (o instanceof String) {
        String w = (String) o;
        log.debug("w = " + w);
      } else if (o instanceof List) {
        List list = (List) o;
        log.debug("words", list);
      } else {
        log.debug("o class = " + o.getClass().toString());
      }
    }

    public void speechDetected(java.lang.Object o) {
      log.trace("speechDetected()!");
    }

    public void soundDetected(java.lang.Object o) {
      log.trace("soundDetected()!");
    }
  }

  public void parseArgs(CommandLine cmdLine) {
      if (cmdLine.hasOption("url")) {
        naoURL = cmdLine.getOptionValue("url");
      }
      if (cmdLine.hasOption("unsafeMove") || cmdLine.hasOption("unsafe")) {
        unsafeMove = true;
      } 
      if (cmdLine.hasOption("doNotWakeUp")) {
        wakeUp = false;
      } 
      if (cmdLine.hasOption("sim")) {
        sim = true;
      }
      if (cmdLine.hasOption("ap")) {
        useAP = true;
      }
      if (cmdLine.hasOption("voice")) {
        voice = cmdLine.getOptionValue("voice");
      }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("url").longOpt("naourl").hasArg().argName("URL").desc("specify the Nao port, e.g., tcp://192.168.0.195:9559").build());
    options.add(Option.builder("unsafe").longOpt("unsafeMove").desc("allow move commands to ignore collision protection").build());
    options.add(Option.builder("doNotWakeUp").desc("do not wake up the nao (i.e. activate motors and stand up)").build());
    options.add(Option.builder("sim").desc("use sim audio player").build());
    options.add(Option.builder("ap").desc("use audio player").build());
    options.add(Option.builder("doNotWakeUp").desc("o not wake/stand up").build());
    options.add(Option.builder("voice").hasArg().argName("low/high").desc("set low(er) voice or high(er) voice").build());
    return options;
  }

  @Override
  public ArrayList<String> getPostureList() {
    ArrayList<String> postures = null;
    if (posture != null) {
      try {
        postures = (ArrayList<String>) (posture.call("getPostureList").get());
      } catch (InterruptedException | CallError e) {
        throw new RuntimeException("couldn't get postures.", e);
      }
    }
    return postures;
  }

  @Override
  public String getPosture() {
    if (posture == null) {
      return null;
    }
    try {
      return (String) (posture.call("getPosture").get());
    } catch (CallError | InterruptedException e) {
      throw new RuntimeException("getPosture failed", e);
    }
  }

  public int getBatteryCharge() {
    if (battery == null) {
      return Integer.MIN_VALUE;
    } else {
      try {
        return (int) (battery.call("getBatteryCharge").get());
      } catch (CallError | InterruptedException e) {
        throw new RuntimeException("getBatteryCharge failed: ", e);
      }
    }
  }

  public double[] takeSonarReadings() {
    double numReadings = 5.0;
    double[] retData = {0.0, 0.0};
    try {
      for (int i = 0; i < numReadings; i++) {
        // not sure this is right, i.e., that the future is a "List"
        Future<Float> leftsonar = memory.call("getData", "Device/SubDeviceList/US/Left/Sensor/Value");
        Future<Float> rightsonar = memory.call("getData", "Device/SubDeviceList/US/Right/Sensor/Value");
        // assuming that the returned list has as its first element the reading...
        float left = (float) leftsonar.get();
        float right = (float) rightsonar.get();
        retData[0] += left;
        retData[1] += right;
      }

    } catch (InterruptedException | CallError e) {
      throw new RuntimeException("Sonar reading interrupted.", e);
    }
    retData[0] /= numReadings;
    retData[1] /= numReadings;
    return retData;
  }

  //*************************************************************
  // Public API methods
  //*************************************************************
  @Override
  public boolean isMoving() {
    try {
      return (Boolean) (motion.call("moveIsActive").get());
    } catch (CallError | InterruptedException e) {
      throw new RuntimeException("Call Error ", e);
    }
  }

  @Override
  public void playFile(String tmpfilename) {
    if (!sim) {
      //	    String filename = tmpfilename.substring(tmpfilename.lastIndexOf("/")+1);
      String remotefilename = "/home/nao/audio/" + tmpfilename;
      try {
        // 	    	log.debug("scpCommand: "+scpCommand);
        // 	    	Runtime.getRuntime().exec(scpCommand);
        // 	    	Thread.sleep(400);
        log.debug("Playing " + remotefilename);
        audioPlayer.call("playFile", remotefilename);
      } catch (CallError e) {
        throw new RuntimeException("Call Error", e);
      }
    } else {
      log.info("Would play " + tmpfilename);
    }
  }

  @Override
  public boolean stopUtterance() {
    return false;
  }

  @Override
  public boolean sayToFile(String text, String filename) {
    System.out.println("JNaoComponent::sayToFile()");
    try {
      if (voice.equalsIgnoreCase("low")) {
        tts.call("sayToFile", "\\VCT=70\\ \\RSPD=90\\" + text, filename);
      } else if (voice.equalsIgnoreCase("high")) {
        tts.call("sayToFile", "\\VCT=140\\" + text, filename);
      } else {
        tts.call("sayToFile", text, filename);
      }

    } catch (CallError ex) {
      throw new RuntimeException("sayToFile failed: ", ex);
    }
    return true;
  }

  @Override
  public boolean sayText(String text) {
    return sayText(text, true);
  }

  @Override
  public boolean sayText(String text, boolean wait) {
    blink();

    if (text == null) {
      log.warn("Text is null!");
      return false;
    } else if (tts == null) {
      log.debug("TTS HOOK is NULL!");
      return false;
    }
    try {
      isSpeaking = true;
      //if (naoASR != null && naoASR.isReady()) {
      //  naoASR.call("disableASR", void.class);
      //}

      if (voice.equalsIgnoreCase("low")) {
        tts.call("say", "\\VCT=70\\ \\RSPD=90\\" + text);
      } else if (voice.equalsIgnoreCase("high")) {
        tts.call("say", "\\VCT=140\\" + text);
      } else {
        tts.call("say", text);
      }

      isSpeaking = false;
      //if (naoASR != null && naoASR.isReady()) {
      //  naoASR.call("enableASR", void.class);
      //}
    } catch (CallError e) {
      throw new RuntimeException("sayText failed: ", e);
    }

    return true;
  }

  @Override
  public boolean isSpeaking() {
    return isSpeaking;
  }

  private void setExternalCollisionProtectionEnabled(boolean flag) {
    try {
      log.trace("setExternalCollisionProtectionEnabled: " + flag);
      motion.call("setExternalCollisionProtectionEnabled", "Move", flag);

      //HACK: because naoqi takes a little bit for the setting to work
      Boolean protectionEnabled = !flag;
      while (protectionEnabled != flag) {
        Future<Boolean> future = motion.call("getExternalCollisionProtectionEnabled", "Move");
        protectionEnabled = future.get();
        if (log.isTraceEnabled()) {
          log.trace("Post getExternalCollisionProtectionEnabled: " + protectionEnabled);
        }
      }
    } catch (CallError | InterruptedException e) {
      throw new RuntimeException("setExternalCollisionProtectionEnabled", e);
    }
  }

  // ********************************************************************
  // *** Nao specific
  // ********************************************************************
  //not-blocking
  @Override
  public boolean moveTo(float x, float y, float theta) {
    try {
      // initialize the move
      motion.call("moveInit");
      motion.call("moveTo", x, y, theta);
    } catch (CallError e) {
      throw new RuntimeException("moveTo failed: ", e);
    }
    return true;
  }

  //blocking
  @Override
  public boolean moveToBlocking(float x, float y, float theta) {
    try {
      // initialize the move
      motion.call("moveInit");
      Future<Void> future = motion.call("moveTo", x, y, theta);
      future.get();
    } catch (CallError | java.lang.InterruptedException e) {
      throw new RuntimeException("moveToBlocking failed: ", e);
    }
    return true;
  }


  //blocking
  @Override
  public void angleInterpolation(String name, float angle, float time, boolean isAbsolute) {
    try {
      Future<Void> future = motion.call("angleInterpolation", name, angle, time, isAbsolute);
      future.get();
    } catch (CallError | java.lang.InterruptedException e) {
      throw new RuntimeException("angleInterpolation failed: ", e);
    }
  }

  //blocking, takes a single time value for all joints
  @Override
  public void angleInterpolation(List<String> names, List<Float> angles, float time, boolean isAbsolute) {
    try {
      Future<Void> future = motion.call("angleInterpolation", names, angles, time, isAbsolute);
      future.get();
    } catch (CallError | java.lang.InterruptedException e) {
      throw new RuntimeException("angleInterpolation failed: ", e);
    }
  }

  //blocking, each joint has its own time value
  @Override
  public void angleInterpolation(List<String> names, List<Float> angles, List<Float> times, boolean isAbsolute) {
    try {
      Future<Void> future = motion.call("angleInterpolation", names, angles, times, isAbsolute);
      future.get();
    } catch (CallError | java.lang.InterruptedException e) {
      throw new RuntimeException("angleInterpolation failed: ", e);
    }
  }

  @Override
  public float getAngle(String name) {
    try {
      Future<ArrayList<Float>> future = motion.call("getAngles", name, false);
      return future.get().get(0);
    } catch (CallError | InterruptedException e) {
      throw new RuntimeException("getAngle failed: ", e);
    }
  }

  @Override
  public boolean pointTo(float x, float y, float z) {
    // determine the hand to point with
    String chainName;
    String handName;
    String shoulderName;
    if (y < 0) {
      chainName = "RArm";
      handName = "RHand";
      shoulderName = "RShoulderPitch"; //RShoulderRoll
    } else {
      chainName = "LArm";
      handName = "LHand";
      shoulderName = "LShoulderPitch"; //LShoulderRoll
    }

    //get current shoulder location in base frame
    Matrix4d shoulderTransform = getTransform(2, shoulderName);
    Point3d shoulderLocation = new Point3d(shoulderTransform.m03, shoulderTransform.m13, shoulderTransform.m23);
    log.debug(String.format("[pointTo] shoulderLocation %s. shoulderTransform %s.", shoulderLocation.toString(), shoulderTransform.toString()));

    //calc unit vector pointing from shoulder to target
    Point3d targetLocation = new Point3d(x, y, z);
    Vector3d dirVec = new Vector3d(targetLocation);
    dirVec.sub(shoulderLocation);
    double targetDist = dirVec.length();  //from shoulderLoc to targetObject
    dirVec.normalize();

    //calc goal location of hand
    //scale length based on distance of object
    double armReach = 0.7 * targetDist;
    armReach = (armReach > 0.2) ? 0.2 : armReach; //meters
    Point3d goalLocation = new Point3d();
    goalLocation.x = shoulderLocation.x + armReach * dirVec.x;
    goalLocation.y = shoulderLocation.y + armReach * dirVec.y;
    goalLocation.z = shoulderLocation.z + armReach * dirVec.z;

    //calc orientation
//    //in x-y plane (in base frame)
//    Vector3d xyzRot = new Vector3d(Math.PI/2.0,0,0); //0,0,0
//    Vector3d tmpDirVec = new Vector3d(dirVec);
//    tmpDirVec.z = 0;
//    tmpDirVec.normalize();
//    xyzRot.y = Math.asin(tmpDirVec.y);  //xyzRot.z
//
//    //in x-z plane (in base frame)
//    tmpDirVec = new Vector3d(dirVec);
//    tmpDirVec.y = 0;
//    tmpDirVec.normalize();
//    xyzRot.z = Math.asin(tmpDirVec.z); //xyzRot.y = -(stuff);
//
//    log.debug(String.format("[pointTo] xyz rotation: %s.", xyzRot.toString()));
//    Quat4d orientation = RotationHelpers.xyzRotationsToQuaternion(xyzRot);
    //Quat4d orientation = new Quat4d(0, 0, 0, 1);
    int frame = 2;//motion.FRAME_ROBOT;
    boolean useSensor = false; // Get the current position of the chainName in the same frame
    float fractionMaxSpeed = 0.5f;
    //AXIS_MASK_X 1
    //AXIS_MASK_Y 2
    //AXIS_MASK_Z 4
    //AXIS_MASK_WX 8
    //AXIS_MASK_WY 16
    //AXIS_MASK_WZ 32
//    int axisMask = 7; // just control position
    int axisMask = 15; // control position and orientation along x axis
//    int axisMask = 63; // control position and orientation
    try {
//      Future<ArrayList<Float>> currPosFuture = motion.call("getPosition", chainName, frame, useSensor);
//      ArrayList<Float> current = currPosFuture.get();
//      log.debug("current position: " + current);

      ArrayList<Float> target = new ArrayList<>(6);
      target.add(0, (float) goalLocation.x);
      target.add(1, (float) goalLocation.y);
      target.add(2, (float) goalLocation.z);
      if (y < 0) {
        target.add(3, 1.57f);
      } else {
        target.add(3, -1.57f);
      }
      target.add(4, 0.0f);
      target.add(5, 0.0f);
      // Turn on motors
      setStiffness("LHipPitch", 1.0f);
      setStiffness("RHipPitch", 1.0f);

      if (y < 0) {
        setStiffness("RShoulderRoll", 1.0f);
        setStiffness("RShoulderPitch", 1.0f);
        setStiffness("RElbowRoll", 1.0f);
        setStiffness("RElbowYaw", 1.0f);
        setStiffness("RHand", 1.0f);
      } else {
        setStiffness("LShoulderRoll", 1.0f);
        setStiffness("LShoulderPitch", 1.0f);
        setStiffness("LElbowRoll", 1.0f);
        setStiffness("LElbowYaw", 1.0f);
        setStiffness("LHand", 1.0f);
      }

      motion.call("setPositions", chainName, frame, target, fractionMaxSpeed, axisMask);
      motion.call("openHand", handName);
      log.debug("target position: " + target);
    } catch (CallError e) {
      throw new RuntimeException("pointTo failed: ", e);
    }
    return true;
  }

  @Override
  public boolean restLeftArm() {
    try {
      pointTo(0.12f, 0.02f, 0.12f);
      closeHand("LHand");

      Thread.sleep(1500); // Wait for arms to move

      setStiffness("LShoulderRoll", 0.0f);
      setStiffness("LShoulderPitch", 0.0f);
      setStiffness("LElbowRoll", 0.0f);
      setStiffness("LElbowYaw", 0.0f);
      setStiffness("LHand", 0.0f);

      setStiffness("LHipPitch", 0.0f); // Disable hip motors only if knees are soft
      setStiffness("RHipPitch", 0.0f);

    } catch (InterruptedException e) {
      throw new RuntimeException("Error calling restLeftArm.", e);
    }

    return true;
  }

  @Override
  public boolean restRightArm() {
    try {
      pointTo(0.12f, -0.02f, 0.12f);
      closeHand("RHand");

      Thread.sleep(1500); // Wait for arms to move

      setStiffness("RShoulderRoll", 0.0f);
      setStiffness("RShoulderPitch", 0.0f);
      setStiffness("RElbowRoll", 0.0f);
      setStiffness("RElbowYaw", 0.0f);
      setStiffness("RHand", 0.0f);

      setStiffness("LHipPitch", 0.0f); // Disable hip motors only if knees are soft
      setStiffness("RHipPitch", 0.0f);

    } catch (InterruptedException e) {
      throw new RuntimeException("Error calling restRightArm.", e);
    }

    return true;
  }

  @Override
  public boolean pointHeadTo(float x, float y, float z) {

    try {
      // transform goal location into head frame (translation only, don't care about current head orientation)
      Point3d targetLocation = getTargetLocation(x, y, z);

      // calc head yaw and pitch
      double pitch = -Math.atan2(targetLocation.z, targetLocation.x);
      double yaw = Math.atan2(targetLocation.y, targetLocation.x);

      ArrayList<String> jointNames = new ArrayList<>(2);
      ArrayList<Float> jointAngles = new ArrayList<>(2);
      jointNames.add("HeadPitch");
      jointNames.add("HeadYaw");
      jointAngles.add((float) pitch);
      jointAngles.add((float) yaw);
      float fractionMaxSpeed = 0.05f;

      setStiffness("HeadPitch", 1.0f);
      setStiffness("HeadYaw", 1.0f);

      //      motion.call("setAngles", jointNames, jointAngles, fractionMaxSpeed);
      ArrayList<Float> times = new ArrayList<>(2);
      times.add(2.0f);
      times.add(2.0f);
      Future<Void> future = motion.call("angleInterpolation", jointNames, jointAngles, times, true);
      future.get();

      setStiffness("HeadPitch", 0.0f);
      setStiffness("HeadYaw", 0.0f);
      log.debug("head angles: " + jointAngles);
    } catch (CallError | InterruptedException e) {
      throw new RuntimeException("Error calling pointHeadTo", e);
    }
    return true;
  }

  private Point3d getTargetLocation(float x, float y, float z) throws CallError, InterruptedException {
    // get current head location in base frame
    Future<ArrayList<Float>> currHeadPosFuture = motion.call("getPosition", "Head", 2, false);
    ArrayList<Float> currHeadPos = currHeadPosFuture.get();
    Point3d headLocation = new Point3d(currHeadPos.get(0), currHeadPos.get(1), currHeadPos.get(2));

    // transform goal location into head frame (translation only, don't care about current head orientation)
    Point3d targetLocation = new Point3d(x - headLocation.x, y - headLocation.y, z - headLocation.z);

    return targetLocation;
  }

  public boolean sonarCheckFloorSupport() {

    double[] readings = takeSonarReadings();

    float left = (float) readings[0];
    float right = (float) readings[1];
    log.debug("SONAR readings " + left + " " + right);

    // check for floor support, i.e., if both sonars are less than maxlenght
    // this is assuming that the robot is standing properly
    if (left >= MAXSONARLENGTH || right >= MAXSONARLENGTH) {
      // no support in front
      log.debug("No floor support in front " + left + " " + right);
      return false;
    }

    return true;
  }

  @Override
  public synchronized boolean checkFloorSupport() {
    // sonarCheckFloorSupport is worthless - basically a coin toss
    // return sonarCheckFloorSupport();
    return true;
  }


  @Override
  public synchronized boolean checkObstacle() {

    double[] readings = takeSonarReadings();

    float left = (float) readings[0];
    float right = (float) readings[1];
    log.debug("SONAR readings " + left + " " + right);

    if ((left <= MINSONARLENGTH && left > 0) || (right <= MINSONARLENGTH && right > 0)) {
      log.debug("SONAR indicates obstacle: " + left + " " + right);
      return true;
    }

    return false;
  }

  @Override
  public boolean goToPosture(String targetPosture) {
    log.trace("goToPosture()method called");
    try {
      posture.call("goToPosture", targetPosture, 0.5);
    } catch (CallError ce) {
      throw new RuntimeException("error calling goToPosture", ce);
    }

    return true;
  }

  @Override
  public boolean openHand(String name) {
    try {
      motion.call("openHand", name);
    } catch (CallError ce) {
      throw new RuntimeException("error calling openHand", ce);
    }
    return true;
  }

  @Override
  public boolean closeHand(String name) {
    try {
      motion.call("closeHand", name);
    } catch (CallError ce) {
      throw new RuntimeException("error calling closeHand", ce);
    }
    return true;
  }

  /**
   * Helper transform method.
   *
   * @param refFrame FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
   * @param dstFrame
   * @return
   */
  private Matrix4d getTransform(int refFrame, String dstFrame) {
    Matrix4d transform = null;
    try {
      boolean useSensorValues = true;
      Future<ArrayList<Float>> future = motion.call("getTransform", dstFrame, refFrame, useSensorValues);
      ArrayList<Float> data = future.get();
      if (data.size() == 16) {
        transform = new Matrix4d(data.get(0), data.get(1), data.get(2), data.get(3),
                data.get(4), data.get(5), data.get(6), data.get(7),
                data.get(8), data.get(9), data.get(10), data.get(11),
                data.get(12), data.get(13), data.get(14), data.get(15));

        // if requesting camera frame, rotate into standard camera orientation (x-right, y-down, z-forward)
        // aldebaran apparently uses robot standards (x-forward, y-left, z-up) for camera frames...sigh
        if (dstFrame.equals("CameraTop") || dstFrame.equals("CameraBottom")) {
          Matrix4d cameratransform = new Matrix4d();
          cameratransform.m02 = 1;
          cameratransform.m10 = -1;
          cameratransform.m21 = -1;
          cameratransform.m33 = 1;

          log.trace("cameratransform: " + cameratransform);
          transform.mul(cameratransform);
        }

      }
    } catch (CallError | InterruptedException e) {
      throw new RuntimeException("getTransform failed:", e);
    }
    return transform;
  }

  @Override
  public Point3d getPixelPositionInRobotFrame(int pX, int pY, int width, int height, int camID) {
    Matrix4d mat = getTransform(2, "Head");
    double camHeight = mat.m23;
    double yaw = Math.asin(mat.m01);
    double pitch = 0.0;
    double cam_tx = 0.0;

    switch (camID) {
      case 0:
        pitch = Math.asin(mat.m02) + Math.toRadians(1.2); // Camera 0 is looking down by 1.2deg
        cam_tx = 0.058; // Camera 0 position in head coordinate frame
        break;
      case 1:
        pitch = Math.asin(mat.m02) + Math.toRadians(39.7); // Camera 1 is looking down by 39.7deg
        cam_tx = 0.05; // Camera 1 position in head coordinate frame
        break;
      default:
        log.error("Camera does not exist");
    }

    // Compute position of pixel
    yaw += ((pX - (width / 2)) / (double) width) * Math.toRadians(60.97); // 60.97deg = Horizontal field of view
    pitch += ((pY - (height / 2)) / (double) height) * Math.toRadians(47.64); // 47.64deg = Vertical field of view

    double x = camHeight * Math.tan(-(pitch - Math.toRadians(90))) + cam_tx;
    double y = Math.sqrt(x * x + camHeight * camHeight) * Math.tan(-yaw);
    double z = -0.1;

    //System.out.println("x:" + x + "  y:" + y + "  z:" + z);
    return new Point3d(x, y, z);
  }

  // ********************************************************************
  // *** VelocityComponent interface
  // ********************************************************************
  @Override
  public boolean setVels(double tv, double rv) {
    log.trace("setVels: " + "rv: " + rv + " tv: " + tv);

    // safety checks
    if (tv > maxTV) {
      tv = maxTV;
    }
    if (rv > maxRV) {
      rv = maxRV;
    }

    try {
      log.trace("calling move");
      motion.call("move", tv, 0.0, rv);
    } catch (CallError e) {
      throw new RuntimeException("setVels failed: ", e);
    }

    // if move call was successful
    curTV = tv;
    curRV = rv;
    return true;
  }

  @Override
  public boolean setRV(double v) {
    log.trace("setRV: " + v);
    return setVels(0.0, v);
  }

  @Override
  public boolean setTV(double tv) {
    log.trace("setTV: " + tv);
    return setVels(tv, 0.0);
  }

  @Override
  public double[] getDefaultVels() {
    return new double[]{defTV, defRV};
  }

  @Override
  public double[] getVels() {
    return new double[]{curTV, curRV};
  }

  @Override
  public double getRV() {
    return curRV;
  }

  @Override
  public double getTV() {
    return curTV;
  }

  /**
   * Stop the motors.
   */
  @Override
  public void stop() {
    try {
      curTV = 0.0;
      curRV = 0.0;
      motion.call("move", 0.0, 0.0, 0.0);
    } catch (CallError ce) {
      throw new RuntimeException("stop failed: ", ce);
    }
  }

  // ********************************************************************
  // *** VelocityComponent interface
  // ********************************************************************


  @Override
  public boolean setStiffness(String name, float stiffness) {
    try {
      motion.call("setStiffnesses", name, stiffness);
      return true;
    } catch (CallError ce) {
      throw new RuntimeException("setStiffness failed: ", ce);
    }
  }

  @Override
  public boolean rest() {
    try {
      motion.call("rest");
      return true;
    } catch (CallError ce) {
      throw new RuntimeException("rest failed: " + ce, ce);
    }
  }

  // Animated Speech

  @Override
  public boolean sayAnimated(String annotatedText) {
    try {
      animSpeech.call("setBodyLanguageModeFromStr", "contextual");
      animSpeech.call("say", annotatedText);
      return true;
    } catch (CallError ce) {
      throw new RuntimeException("error calling sayAnimated", ce);
    }
  }

  // EAK: this method shouldn't be in the nao component. Only
  // core functionality for the nao should go in here. See the
  // nao tower component or wizard component for examples
  // of experiment specific code.
  @Override
  public boolean welcome() {
    try {
      blink();
      pointTo(0.2f, -0.1f, 0.15f);
      pointTo(0.2f, 0.1f, 0.15f);
      blink();
      Thread.sleep(500);
      tts.call("say", "Welcome");
      Thread.sleep(500);
      restRightArm();
      blink();
      Thread.sleep(200);
      pointTo(0.15f, 0.0002f, 0.35f);
      closeHand("LHand");
      Thread.sleep(1500);
      tts.call("say", "My name is Shafer");
      blink();
      Thread.sleep(2000);
      //restLeftArm();
      return true;
    } catch (CallError | InterruptedException e) {
      throw new RuntimeException("error calling sayAnimated: " + e, e);
    }
  }

  public boolean setBreathEnabled(String par, boolean enable) {
    try {
      motion.call("setBreathEnabled", par, enable);
      return true;
    } catch (CallError ce) {
      throw new RuntimeException("setBreathEnabled failed: " + ce, ce);
    }
  }

  public boolean ledOn(String name) {
    try {
      leds.call("on", name);
      return true;
    } catch (CallError ce) {
      throw new RuntimeException("ledOn failed: " + ce, ce);
    }
  }

  public boolean ledOff(String name) {
    try {
      leds.call("off", name);
      return true;
    } catch (CallError ce) {
      throw new RuntimeException("ledOff failed: " + ce, ce);
    }
  }

  @Override
  public boolean blink() {
    try {
      leds.call("off", "FaceLedsTop");
      Thread.sleep(20);
      leds.call("off", "FaceLedsExternal");
      leds.call("off", "FaceLedsInternal");
      Thread.sleep(100);

      leds.call("on", "FaceLedsInternal");
      leds.call("on", "FaceLedsExternal");
      Thread.sleep(20);
      leds.call("on", "FaceLedsTop");

      return true;
    } catch (CallError | InterruptedException e) {
      throw new RuntimeException("blink failed: " + e, e);
    }
  }
}
