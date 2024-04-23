/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.Gson;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.mtracs.consultant.pose.MPoseConsultant;
import edu.tufts.hrilab.mtracs.consultant.pose.MPoseReference;
import edu.tufts.hrilab.mtracs.consultant.vision.CognexConsultant;
import edu.tufts.hrilab.mtracs.consultant.vision.CognexReference;
import edu.tufts.hrilab.mtracs.gson.MPoseGson;
import edu.tufts.hrilab.mtracs.util.CognexJob;
import edu.tufts.hrilab.mtracs.util.CognexResult;
import edu.tufts.hrilab.mtracs.util.MPose;
import edu.tufts.hrilab.mtracs.gson.CR800Config;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.vecmath.Point3d;
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class MockCR800Component extends DiarcComponent implements CR800ComponentInterface {

    //+++ Config Supplied Information +++
    //-- Config file info
    protected String configFilePath = "";
    //-- Connection Settings
    //-- Gripper Switching
    protected Map<Symbol, MPose> dropoffs;
    //-- Compilation info
    protected List<String> additionalHeader;
    protected String modelCode;
    //-- Vision
    protected String defaultCognexJob;
    //-- Manipulation
    protected Point3d cognexOffset;
    protected Map<Symbol, MPose> tcps;

    //+++ Sub-components +++
    protected MPoseConsultant mxtPoseConsultant;
    protected CognexConsultant cognexConsultant;

    //+++ Universal Constants +++
    //-- Gripper
    protected final Symbol DEFAULT_GRIPPER = new Symbol("gripper");
    protected final int GRIPPER_POLL_SLEEP = 100;
    protected final int GRIPPER_FINISH_SLEEP = 3000;
    protected final int GRIPPER_CLOSED = 0;
    protected final int GRIPPER_OPEN = 1;
    //-- Cognex
    protected final Point3d COGNEX = new Point3d(0.07, 0, 0.12);
    //-- End effector empirically found orientations
    //Gripper fingers facing in negative z axis direction towards work surface
    //Note: This is also the presumed orientation prior to any cognex job
    protected final Point3d DOWN = new Point3d(3.14, 0, 3.14);

    //-- Robot Settings
    protected String myAgentName;

    protected int SIM_SLEEP_T = 0;


    public MockCR800Component() {
        super();
        this.cognexOffset = new Point3d(0, 0, 0);

    }

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = super.additionalUsageInfo();
        options.add(Option.builder("config").numberOfArgs(1).argName("filepath").desc("set CR800 configuration").build());
        options.add(Option.builder("sleepTime").numberOfArgs(1).argName("duration").desc("set sleep time (in ms) for movement actions for testing purposes").build());
        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        if (cmdLine.hasOption("config")) {
            configFilePath = cmdLine.getOptionValue("config");
        }
        if (cmdLine.hasOption("sleepTime")) {
            SIM_SLEEP_T = Integer.parseInt(cmdLine.getOptionValue("sleepTime"));
        }
    }

    /**
     * Helper method which loads a JSON formatted config file to describe the
     * robot that is connected to this CR800 controller. Examples can be seen
     * in resources/config/
     *
     * @param path The String representation of the path to the desired config
     *             file relative to the /resources folder (must be preceded with /).
     * @return The configuration as a CR800Config object.
     */
    private CR800Config loadConfigFile(String path) {
        try {
            Gson gson = new Gson();
            StringBuilder json = new StringBuilder();
            InputStream is = getClass().getResourceAsStream(path);
            BufferedReader br = new BufferedReader(new InputStreamReader(is));
            List<String> jsonList = br.lines().collect(Collectors.toList());
            for (String s : jsonList)
                json.append(s);
            return gson.fromJson(json.toString(), CR800Config.class);
        } catch (Exception e) {
            log.error("exception loading config file", e);
            return null;
        }
    }

    /**
     * Helper fuction to initialize all configurable data in <code>this</code>
     * CR800 component to values stored within the given config object.
     *
     * @param config The configuration as a CR800Config object.
     */
    private void initFromConfig(CR800Config config) {
        if (config == null) {
            log.warn("CR800 Config was not properly loaded, no config information will be set!");
            return;
        }
        //Initialize compilation configuration
        //Todo: Will: must wrap this in a constructor or there's no List implementation information
        additionalHeader = new ArrayList<>(Arrays.asList(config.mHeader));
        modelCode = config.modelCode;
        defaultCognexJob = config.defaultCognexJob;
        //Initialize connection configuration
        //Initialize robot and area configuration
        tcps = new HashMap<>();
        for (Map.Entry<String, MPoseGson> e : config.tcps.entrySet()) {
            this.tcps.put(new Symbol(e.getKey()), new MPose(e.getValue()));
        }
        dropoffs = new HashMap<>();
        for (Map.Entry<String, MPoseGson> e : config.dropoffs.entrySet()) {
            this.dropoffs.put(new Symbol(e.getKey()), new MPose(e.getValue()));
        }
        if (tcps.containsKey(DEFAULT_GRIPPER)) {
            //add default gripper as a line in the melfa header for compiled program functionality
            MPose defaultTCP = tcps.get(DEFAULT_GRIPPER);
            //Calculate cognex offset based on TCP and COGNEX mounting point
            cognexOffset = this.tcps.get(DEFAULT_GRIPPER).getTranslation();
            cognexOffset.sub(COGNEX);
        } else {
            log.error("No default gripper TCP set, this will likely lead to manipulation errors!");
        }
    }

    @Override
    /**
     * Starts this component, and initializes all values.
     */
    protected void init() {
        //Config Fetching
        if (!configFilePath.equals("")) {
            initFromConfig(loadConfigFile(configFilePath));
        }
        //Diarc Setup
        myAgentName = "self:agent";
        if (!getMyGroups().isEmpty()) {
            //Todo: Will: This is a very specific hack for the format "agent:robot_one", can we be more general?
            //Todo: Will: Do we even need to support this anymore? Not sure the agent name is a problem with typed symbols anymore
            myAgentName = getMyGroups().get(0).split(":")[1];
        }
        //Sub-component Setup

        cognexConsultant = new CognexConsultant(this.getMyGroups());
        List<String> cognexGroups = this.getMyGroups();
        cognexGroups.add(cognexConsultant.getKBName());
        mxtPoseConsultant = new MPoseConsultant(MPoseReference.class, "pose");
        List<String> poseGroups = this.getMyGroups();
        poseGroups.add(mxtPoseConsultant.getKBName());
        try {
            TRADE.registerAllServices(cognexConsultant, cognexGroups);
            TRADE.registerAllServices(mxtPoseConsultant, poseGroups);
        } catch (TRADEException e) {
            log.error("Error registering with trade ", e);
        }
    }

    @Override
    public List<CognexResult> getCameraData(String jobName) {

        List<CognexResult> results = new ArrayList<>();
        CognexResult r1;
        CognexResult r2;
        switch (jobName) {
            case "holeDet":
                //TODO:brad: figure out how to make a left/right distinction between these two.
                //Todo: Will:
                //right is positive y
                r1 = new CognexResult("+63.475s(-0.58,+3.39,+0.00,+0.00,+0.00,+0.10,+0.00,+0.00)(0,0)\n", new MPose());
                //left is negative y
                r2 = new CognexResult("+63.475s(+0.58,+3.39,+0.00,+0.00,+0.00,+0.10,+0.00,+0.00)(0,0)\n", new MPose());
                results.add(r1);
                results.add(r2);
                break;
            case "holeDeep":
                //TODO:brad: figure out how to make a left/right distinction between these two.
                //Todo: Will:
                //right is positive y
                r1 = new CognexResult("+63.475s(-0.58,+3.39,+0.00,+0.00,+0.00,+0.10,+0.00,+0.00)(0,0)\n", new MPose());
                //left is negative y
                r2 = new CognexResult("+63.475s(+0.58,+3.39,+0.00,+0.00,+0.00,+0.10,+0.00,+0.00)(0,0)\n", new MPose());
                results.add(r1);
                results.add(r2);
                break;
            case "holeM3":
                //TODO:brad: figure out how to make a left/right distinction between these two.
                //Todo: Will:
                //right is positive y
                r1 = new CognexResult("+63.475s(-0.58,+3.39,+0.00,+0.00,+0.00,+0.10,+0.00,+0.00)(0,0)\n", new MPose());
                //left is negative y
                r2 = new CognexResult("+63.475s(+0.58,+3.39,+0.00,+0.00,+0.00,+0.10,+0.00,+0.00)(0,0)\n", new MPose());
                results.add(r1);
                results.add(r2);
                break;
            case "nvDet":
            case "cbDet":
            case "feedrDet":
            case "pillDet"://pill bottle
            case "ioCDet": //IO card for PLC
            case "antisDet": //IO card for PLC
            case "propDet": //IO card for PLC
            case "sBoxDet": //screw box
                r1 = new CognexResult("+63.475s(+0.58,+3.39,+0.00,+0.00,+0.00,+0.10,+0.00,+0.00)(0,0)\n", new MPose());
                results.add(r1);
                break;
            default:
                log.error("[getCameraData] MockCR800 component does not support jobName: " + jobName);
        }
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return results;
    }

    @Override
    public boolean moveToCognexTarget(List<CognexResult> results, int jobIndex) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        if (results == null) {
            log.error("No cognex results!");
            return false;
        }
        CognexResult target = results.get(jobIndex);
        if (target.getConfidenceScore() == 0.0) {
            log.error("Cognex target confidence score too low!");
            return false;
        }
        //Todo: Will: While we are calling another primitive, this is a non annotated primitive for the time being
        return moveAbove(target.getCoords());
    }

    @Override
    public boolean moveToCognexTarget(Symbol refID) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Symbol typedRef = Factory.createSymbol(refID.getName(), "physobj");
        CognexReference r = cognexConsultant.getReference(typedRef);
        if (r != null) {
            CognexResult target = r.result;
            if (target == null) {
                log.error("[moveToCognexTarget] CognexReference not bound to a CognexResult");
                return false;
            }
            if (target.getConfidenceScore() == 0.0) {
                log.error("[moveToCognexTarget] Cognex target confidence score too low!");
                return false;
            }
            return moveAbove(target.getCoords());//Todo:Will: return for automate stuff later or eval for screws //&& mitsubishiEEOrientTo(0, 0, target.getTheta());
        }
        log.error("[moveToCognexTarget] Reference ID supplied not bound to a CognexReference: " + refID);
        return false;
    }

    @Override
    public MPose getCurrentPose() {
        return new MPose();
    }

    @TRADEService
    public void connectToController(){
       return;
    }

    @Override
    public Symbol recordPose(Symbol poseName, Symbol surfaceHeight) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        log.info("Recording: " + poseName.getName() + " with surface height: " + surfaceHeight.getName() + "mm!");
        MPose currentPose = getCurrentPose();
        currentPose.setSurfaceHeight(Float.parseFloat(surfaceHeight.getName()) / 1000.0f);
        return addPose(poseName.getName(), currentPose);
    }

    @Override
    public Symbol recordPose(Symbol poseName, MPose pose, Symbol surfaceHeight) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        log.info("Recording: " + poseName.getName() + " with surface height: " + surfaceHeight.getName() + "mm!");
        pose.setSurfaceHeight(Float.parseFloat(surfaceHeight.getName()) / 1000.0f);
        return addPose(poseName.getName(), pose);
    }

    /**
     * Helper method used to create a new entry in our pose consultant for the given pose name and pose data.
     *
     * @param poseName The string representation of the human understandable pose name.
     * @param pose     The MPose representing the pose to save to our pose consultant.
     */
    protected Symbol addPose(String poseName, MPose pose) {
        //add pose name to properties consultant can handle
        List<Term> props = new ArrayList<>();
        props.add(Factory.createPredicate(poseName, "X:pose"));
        boolean novel=mxtPoseConsultant.addPropertiesHandled(props);

        if (novel) {
            //Get a new refId from consultant
            List<Variable> vars = new ArrayList<>();
            Variable var = Factory.createVariable("X");
            vars.add(var);
            Map<Variable, Symbol> refIds = mxtPoseConsultant.createReferences(vars);

            //Notify consultant we have an instance of new location type
            mxtPoseConsultant.assertProperties(refIds.get(var), props);

            //add location name to created reference
            MPoseReference poseReference = mxtPoseConsultant.getReference(refIds.get(var));
            poseReference.setPose(pose);

            return poseReference.refId;
        }else {
            return mxtPoseConsultant.updateReference(props.get(0),pose);
        }
    }

    @Override
    public MPose getPoseFromSymbol(Symbol poseName) {
        //TODO: refs coming out of the planner need to have types
        Symbol p = new Symbol(poseName.getName(), "pose");
        MPoseReference poseRef = mxtPoseConsultant.getReference(p);
        if (poseRef == null) {
            log.error("Pose not found: " + poseName.getName());
            return null;
        }
        return poseRef.getPose();
    }

    @Override
    public boolean reconnect() {
        return true;
    }

    @Override
    public boolean moveInPlane(Symbol x, Symbol y) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean moveAbove(Symbol s) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return moveInPlane(((Predicate) (s)).getArgs().get(0), ((Predicate) (s)).getArgs().get(1));
    }

    @Override
    public boolean moveAwayFromJointLimit() {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return eeOrientTo(new Point3d(0, 0, Math.PI / -2.0));
    }

    @Override
    public boolean undoCognexOffset() {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MPose goal = new MPose();
        goal.setXMeters((float) cognexOffset.x);
        goal.setYMeters((float) cognexOffset.y);
        return moveToRelative(goal);
    }

    @Override
    public boolean doCognexOffset() {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MPose goal = new MPose();
        goal.setXMeters((float) cognexOffset.x);
        goal.setYMeters((float) cognexOffset.y);
        return moveToRelative(goal);
    }

    @Override
    public boolean moveAndOrientToCognexTarget(Symbol refID) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Symbol typedRef = new Symbol(refID.getName(), "physobj");
        CognexReference r = cognexConsultant.getReference(typedRef);
        if (r != null) {
            CognexResult target = r.result;
            if (target == null) {
                log.error("[moveToCognexTarget] CognexReference not bound to a CognexResult");
                return false;
            }
            if (target.getConfidenceScore() == 0.0) {
                log.error("[moveToCognexTarget] Cognex target confidence score too low!");
                return false;
            }
            return moveAbove(target.getCoords()) && eeOrientTo(new Point3d(0, 0, target.getTheta()));
        }
        log.error("[moveToCognexTarget] Reference ID supplied not bound to a CognexReference: " + refID);
        return false;
    }

    @Override
    public void bindCognexResult(CognexReference ref, CognexResult result, int indexIntoCognexResult) {
        log.debug("Binding: " + ref.refId + " to cognex result index: " + indexIntoCognexResult);
        ref.setResult(result);
        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name( "updateFOC")).call(void.class, ref.refId);
        } catch (TRADEException e) {
            log.error("[bindCognexResult] Exception calling updateFOC: ", e);
        }
    }

    @Override
    public CognexResult getMatchingResult(CognexReference toReBind, List<CognexResult> results) {
        //TODO:brad: this is not as complete as it should be.
        if (toReBind.properties.stream().anyMatch(p -> p.getName().equals("right"))) {
            results.sort(Comparator.comparingDouble(CognexResult::getYAbs));
        }
        if (toReBind.properties.stream().anyMatch(p -> p.getName().equals("left"))) {
            results.sort(Comparator.comparingDouble(CognexResult::getYAbs).reversed());
        }
        if (toReBind.properties.stream().anyMatch(p -> p.getName().equals("bottom"))) {
            results.sort(Comparator.comparingDouble(CognexResult::getXAbs));
        }
        if (toReBind.properties.stream().anyMatch(p -> p.getName().equals("top"))) {
            results.sort(Comparator.comparingDouble(CognexResult::getXAbs).reversed());
        }

        return results.get(0);
    }

    @Override
    public List<Term> getEmptyProps() {
        return new ArrayList<>();
    }

    @Override
    public CognexReference createCogRefWithProps(CognexJob j, List<Term> additionalProperties) {
        CognexReference ref = cognexConsultant.createCognexRef(j, additionalProperties);
        if (!additionalProperties.isEmpty()) {
            //TODO:brad:what happens if these are duplicate?
            cognexConsultant.assertProperties(ref.refId, additionalProperties);
        }
        return ref;
    }

    @Override
    public CognexJob getCognexJobForCognexReference(CognexReference ref) {
        return ref.cognexJob;
    }

    @Override
    public CognexReference getCognexReferenceForID(Symbol refId) {
        CognexReference ret = cognexConsultant.getReference(refId);

        //If the reference isn't in this component's consultant, remove it from remote and add it to local
        if (ret == null) {
            TRADEServiceConstraints additionalConstraints = new TRADEServiceConstraints().notInGroups(this.getMyGroups().get(0)); //not working
            Collection<TRADEServiceInfo> kbNameServices = TRADE.getAvailableServices(additionalConstraints.name("getKBName"));
            for (TRADEServiceInfo kbNameService : kbNameServices) {
                try {
                    String kbname = kbNameService.call(String.class);
                    if (kbname.equals("physobj") && Collections.disjoint(this.getMyGroups(), kbNameService.getGroups())) { //this can be removed with proper filtering
                        CognexReference ref = TRADE.getAvailableService(new TRADEServiceConstraints().inGroups(kbNameService.getGroups().toArray(new String[0])).name( "removeReference")).call(CognexReference.class, refId);
                        ref.setResult(null);
                        if (cognexConsultant.insertReference(refId, ref)) {
                            return ref;
                        }
                        log.error("Error transferring reference from remote to local");
                        return null;
                    }
                } catch (TRADEException e) {
                    log.error("[getCognexReferenceForID]",e);
                }
            }
            //make trade call for getCognexReferenceForID

            log.warn("[getCognexReferenceForID] null reference found for id:" + refId);
        }
        return ret;
    }


    @Override
    public Symbol getDescriptorForID(Symbol refId) {
        CognexReference ref = cognexConsultant.getReference(refId);
        if(ref == null) {
            log.warn("[getDescriptorForID] reference: "+refId+" not found in consultant: "+this.cognexConsultant.getKBName());
            return Factory.createSymbol("none");
        }
        List<Term> props = ref.properties;
        if(!props.isEmpty()) {
            return Factory.createSymbol(props.get(0).getName());
        } else{
            log.warn("[getDescriptorForID] no properties found for reference: "+refId);
            return Factory.createSymbol("none");
        }
    }

    @Override
    public CognexJob getCognexJobForDescriptor(Symbol descriptor) {
        return cognexConsultant.getJobForDescriptor(descriptor.getName());
    }

    @Override
    public boolean eeOrientTo(Point3d g) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public List<String> getMHeader() {
        List<String> mHeader = new ArrayList<>();
        Map<String, String> constants = getConstantMappings();
        for (String constant : constants.keySet()) {
            mHeader.add(constant + " = " + constants.get(constant));
        }
        //The X and Y of the cognex are actually flipped, hence the strange notations
        mHeader.add("Def FNPTRANS(PCOG) = SetPos(PCOG.Y + P_Curr.X + PCOGOFF.X" +
                ", (PCOG.X * -1) + P_Curr.Y + PCOGOFF.Y" +
                ", P_Curr.Z, P_Curr.A, P_Curr.B, P_Curr.C + ((PCOG.C * -3.14159)/180))");
        mHeader.add("Def FNPTRANZ(PTAKEN, PCOG) = SetPos(PCOG.Y + PTAKEN.X + PCOGOFF.X" +
                ", (PCOG.X * -1) + PTAKEN.Y + PCOGOFF.Y" +
                ", PTAKEN.Z, PTAKEN.A, PTAKEN.B, PTAKEN.C + ((PCOG.C * -3.14159)/180))");
        mHeader.add("Servo On");
        mHeader.add("If M_NvOpen(3)<>1 Then");
        mHeader.add("NVOpen \"COM3:\" As #3");
        mHeader.add("EndIf");
        mHeader.add("Wait M_NvOpen(3)=1");
        //Arrays to handle and serve as global accessors for cognex results
        mHeader.add("Dim PVS(10)");
        mHeader.add("Dim MD#(10)");
        return mHeader;
    }

    @Override
    public Map<String, String> getConstantMappings() {
        Map<String, String> constants = new HashMap<>();
        constants.put("PCOGOFF", "("
                + (cognexOffset.x * 1000)
                + "," + (cognexOffset.y * 1000)
                + "," + (cognexOffset.z * 1000) + ",0,0,0)(7,0)");
        //Cognex confidence scores are usually truncated at the job level, so our confidence threshold can afford to be
        //quite low here
        constants.put("MCONF", "" + 0.1);
        return constants;
    }

    @Override
    public String getModelType() {
        return modelCode;
    }

    @Override
    public boolean delay(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            log.error("Failed to sleep mitsubishi arm!");
            return false;
        }
        return true;
    }


    @Override
    public boolean openGripper() {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean closeGripper() {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    //todo: all of the screwdriver related primitives should actually get feedback instead of blindly executing
    @Override
    public boolean pickupScrew() {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean tightenScrew(Symbol screwType) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean shankOut() {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean shankIn() {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean goToPose(MPose goal) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean goToPoseLong(MPose goal) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public MPose adjustPoseToCameraHeight(MPose goal, Symbol cameraHeight) {
        return goal;
    }

    @Override
    public void bindToSurface(MPose goal, Symbol surfaceHeight) {
        List<Map<Variable, Symbol>> beliefs = null;
        try {
            beliefs = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Term.class)).call(List.class, new Predicate("offset", new Symbol(myAgentName), surfaceHeight, new Variable("X")));

            if (beliefs == null || beliefs.size() == 0) {
                log.warn("Could not find any matching offset in belief for: " + surfaceHeight + ", setting breaking value!!");
                //Because of some close calls with failing to properly teach a surface and then going to a camera pose with the wrong value,
                //we inject a value which will cause the robot controller to fail if this condition is reached and a go to camera pose is attempted
                goal.setSurfaceHeight(10000);
                return;
            }
        } catch (TRADEException e) {
            log.error("[bindToSurface] Exception while trying to call queryBelief.", e);
            return;
        }
        Symbol numHeight = beliefs.get(0).get(new Variable("X"));

        goal.setSurfaceHeight(Float.parseFloat(numHeight.getName()) / 1000.0f);
    }

    @Override
    public MPose getTCPForEE(Symbol eeType) {
        return tcps.get(eeType);
    }

    @Override
    public boolean alternateEE(MPose newTCP) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean acceptEE() {
        return true;
    }

    @Override
    public boolean ejectEE() {
        return true;
    }

    @Override
    public MPose getGripDropoff(Symbol s) {
        if (this.dropoffs.containsKey(s)) {
            return dropoffs.get(s);
        }
        log.error("[getGripDropoff]: unable to find gripper dropoff for" + s.getName() + "; check config!");
        return null;
    }

    @Override
    public boolean rotateForToolRack() {
        return eeOrientTo(new Point3d(0, 0, Math.PI));
    }

    @Override
    public boolean moveZRelative(double dist) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    @Override
    public boolean moveXRelative(double dist) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MPose updated = new MPose();
        updated.setXMeters((float) dist);
        return moveToRelative(updated);
    }

    @Override
    public boolean moveToRelative(MPose g) {
        try {
            Thread.sleep(SIM_SLEEP_T);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }

    //todo:(pete) should this be done here? it feels like there should be a better way to force a reference to no longer be salient?
    @Override
    public boolean removeCognexReferenceWithID(Symbol refID) {
        return cognexConsultant.removeReference(refID) != null;
    }
}
