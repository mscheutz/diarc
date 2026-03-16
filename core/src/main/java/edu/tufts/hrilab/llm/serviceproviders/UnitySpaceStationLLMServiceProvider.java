package edu.tufts.hrilab.llm.serviceproviders;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import com.google.gson.Gson;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.consultant.pose.PoseReferenceJson;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.LLMServiceProvider;
import edu.tufts.hrilab.llm.Prompt;
import edu.tufts.hrilab.llm.Prompts;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.slug.parsing.llm.AlternateResponse;
import edu.tufts.hrilab.slug.parsing.llm.Descriptor;
import edu.tufts.hrilab.slug.parsing.llm.ParserResponse;
import edu.tufts.hrilab.slug.parsing.llm.Proposition;
import edu.tufts.hrilab.slug.parsing.llm.Referent;
import edu.tufts.hrilab.unity.space_station.llm.LlamaSpaceStationResponse;
import edu.tufts.hrilab.util.resource.Resources;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class UnitySpaceStationLLMServiceProvider extends LLMServiceProvider {
    static private Logger log = LoggerFactory.getLogger(UnitySpaceStationLLMServiceProvider.class);

    private static String refsDirectoryPath = "config/edu/tufts/hrilab/map";
    Pattern agentPattern = Pattern.compile("agent:");
    Matcher agentMatcher = agentPattern.matcher("");
    Prompt nluPrompt = Prompts.getPrompt("spaceStationFinetunedLlama");

    private String refsFile;
    HashMap<String, Symbol> locationRefs;

    /**
     * Simplified Pose Reference class only used for reading/writing JSON files.
     */
    class PoseRefJson {
        public List<PoseReferenceJson> poseReferences;
    }

    @Override
    protected void init () {
        Gson gson = new Gson();
        String[] parts;
        String refId;
        List<String> properties;

        String filename = Resources.createFilepath(UnitySpaceStationLLMServiceProvider.refsDirectoryPath, refsFile);

        InputStream stream = UnitySpaceStationLLMServiceProvider.class.getResourceAsStream(filename);
        if (stream == null) {
            log.error("Resource not found: " + filename);
            return;
        }

        BufferedReader reader = new BufferedReader((new InputStreamReader(stream)));
        UnitySpaceStationLLMServiceProvider.PoseRefJson loadedPoseRefs = gson.fromJson(reader, UnitySpaceStationLLMServiceProvider.PoseRefJson.class);
        locationRefs = new HashMap<>();
        for (PoseReferenceJson poseRef : loadedPoseRefs.poseReferences) {
            refId = poseRef.getRefId();
            properties = poseRef.getProperties();
            parts = properties.get(0).split("\\(");
            locationRefs.put(parts[0], Factory.createSymbol(refId));
        }
    }

    @Override
    public Utterance convertSemanticsToTextLLMService(Utterance utterance) {
        //TODO: when looking back at lab branch, set this up correctly if exists (assuming stylizeUnitySpeak is filling in for this some way)
        log.warn("NLG for UnitySpaceStation not set up through LLMServiceProvider");
        return null;
    }

    @Override
    public boolean inExtendedInteraction(Symbol user, Symbol agentTeam) {
        return false;
    }

    @Override
    public Utterance parseUtteranceLLMService(Utterance utterance) {
        Chat chat = new Chat(nluPrompt);
        chat.addUserMessage(utterance.getWordsAsString());

        Completion llmResponse = callChatEndpoint(chat);

        String jsonString = llmResponse.toString();
        LlamaSpaceStationResponse lssRes = null;
        if (!jsonString.contains("}")) {
            jsonString += "}";
        }
        log.debug("LLM response object: " + jsonString);
        try {
            lssRes = LlamaSpaceStationResponse.fromString(jsonString);
        } catch (Exception e) {
            log.error("Error parsing response: ", e);
            return null;
        }

        Symbol addressee = null;
        AlternateResponse altResponse = new AlternateResponse(lssRes.toParserResponse(), lssRes.toAddressee());
        ParserResponse response = altResponse.response;
        if (altResponse.addressee != null) {
            addressee = altResponse.addressee;
            log.debug("Got addressee from alternate LLM service: " + addressee.toString());
        }

        if (response == null) {
            log.error("Error: Failed to get response");
            return null;
        } else {
            log.info("Got response: " + response.toString());
        }

        Utterance.Builder output = new Utterance.Builder(utterance);
        output.setUtteranceType(UtteranceType.valueOf(response.intention.intent.toUpperCase()));

        // collect referent info by variable name
        Map<String, Referent> referentMap = new HashMap<>();
        List<Variable> variables = new ArrayList<>();
        for (Referent referent : response.referents) {
            referentMap.put(referent.variable_name, referent);
            variables.add(Factory.createVariable(referent.variable_name, referent.type));
        }

        // populate semantics with semantically typed variables
        Proposition prop = response.intention.proposition;
        Predicate semantics;
        if (UtteranceType.valueOf(response.intention.intent.toUpperCase()) == UtteranceType.INSTRUCT) {
            List<Symbol> args = new ArrayList<>();
            if (addressee != null) {
                args.add(addressee);
                output.setAddressee(addressee);
            } else {
                args.add(utterance.getAddressee());
                output.setAddressee(utterance.getAddressee());
            }
            Arrays.stream(prop.arguments).forEach(arg -> args.add(Factory.createFOL(arg)));
            semantics = Factory.createPredicate(prop.text, args);
        } else {
            semantics = Factory.createPredicate(prop.text, prop.arguments);
        }
        output.setSemantics(semantics.copyWithNewVariableTypes(variables));

        // populate supplemental semantics
        for (Descriptor descriptor : response.descriptors) {
            Predicate descriptorPred = Factory.createPredicate(descriptor.text, descriptor.arguments);
            output.addSupplementalSemantics(descriptorPred.copyWithNewVariableTypes(variables));
        }

        // add tier assignments to supplemental semantics
        variables.forEach(var -> output.addTierAssignment(var, Factory.createSymbol(referentMap.get(var.getName()).toString().toUpperCase())));
        //    variables.forEach(var -> output.addTierAssignment(var, Factory.createSymbol(referentMap.get(var.getName()).cognitive_status.toUpperCase())));

        return output.build();
    }

    private String getAgent () {
        List<String> groups = getMyGroups();
        for (String group : groups) {
            agentMatcher.reset(group);
            if (agentMatcher.find()) {
                return group.split(":")[1];
            }
        }
        return "robot1";
    }

    public static String toSpoken (String str) {
        return String.join(" ", str.split("(?<=[a-z])(?=[A-Z])"));
    }

    public static String capitalize (String str) {
        return str.substring(0, 1).toUpperCase() + str.substring(1);
    }

    public static Symbol toFullTube (Symbol area, Symbol side, Symbol number) {
        return Factory.createSymbol( area.toString() + capitalize(side.toString()) + capitalize(number.toString()) );
    }

    public static Symbol toTubeNumber (Symbol side, Symbol number) {
        return Factory.createSymbol( capitalize(side.toString()) + capitalize(number.toString()) );
    }

    public static Symbol getTubeSide (Symbol tubeCombined) {
        String[] parts = tubeCombined.toString().split("(?=[A-Z])");
        return Factory.createSymbol( parts[0].toLowerCase() );
    }

    public static Symbol getTubeNumber (Symbol tubeCombined) {
        String[] parts = tubeCombined.toString().split("(?=[A-Z])");
        return Factory.createSymbol( parts[1].toLowerCase() );
    }

    public static List<Symbol> fromFullTube (Symbol fullTube) {
        List <Symbol> symbolList = new ArrayList<>();
        for (String str : fullTube.toString().split("(?<!(^|[A-Z]))(?=[A-Z])|(?<!^)(?=[A-Z][a-z])")) {
            symbolList.add( Factory.createSymbol(str.toLowerCase()) );
        }
        return symbolList;
    }

    //TODO: rewrite this?
    @TRADEService
    @Action
    public void stylizeUnitySpeak (Symbol original) {
        Prompt prompt = Prompts.getPrompt("spaceStationSpeechStyle");
        String model = "gpt-3.5-turbo";
        Symbol actor = Factory.createSymbol("robot1");
        Chat chat = new Chat(prompt);
        Completion oResponse = null;

        chat.addUserMessage(original.toString());

        try {
            oResponse = TRADE.getAvailableService(new TRADEServiceConstraints().name("chatCompletion").argTypes(Symbol.class,Chat.class)).call(Completion.class, chat, model);
        } catch (TRADEException e) {
            log.error("Error getting a completion.", e);
        }

        Completion response = oResponse;
        String strResponse = response.toString().trim();
        Symbol statement = Factory.createSymbol(strResponse);

        log.info("Response: " + strResponse);

        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(Symbol.class,String.class)).call(void.class, actor, statement);
        } catch (TRADEException e) {
            log.error("Error calling sayText.", e);
        }
    }

    @TRADEService
    @Action
    public Symbol resolveReferenceLLM (Symbol location) {
        if (locationRefs.containsKey(location.toString())) {
            return locationRefs.get(location.toString());
        }
        return null;
    }
}
