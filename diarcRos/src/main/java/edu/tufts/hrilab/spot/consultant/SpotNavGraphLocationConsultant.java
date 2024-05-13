/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.consultant;

import edu.tufts.hrilab.consultant.Consultant;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.util.resource.Resources;

import java.io.*;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class SpotNavGraphLocationConsultant extends Consultant<SpotNavGraphLocationReference> {


    private static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();

    public SpotNavGraphLocationConsultant(Class<SpotNavGraphLocationReference> refClass, String kbName) {
        super(refClass, kbName);
    }

    public SpotNavGraphLocationConsultant(Class<SpotNavGraphLocationReference> refClass, String kbName, List<String> properties) {
        super(refClass, kbName, properties);
    }

    @Override
    protected <U> U localConvertToType(Symbol refId, Class<U> type) {
        return null;
    }

    @Override
    protected <U> U localConvertToType(Symbol refId, Class<U> type, List<? extends Term> constraints) {
        return null;
    }

    public SpotNavGraphLocationReference getRefWithId(Symbol refId) {
        return references.get(refId);
    }

    //todo: consolidate json and reference-loading with functionality in PoseConsultant?
    class SpotMissionLocationRefJson {
        public List<SpotNavGraphLocationReferenceJson>  references;
    }

    /**
     * Load pre-defined references from file via JSON.
     * @param dir
     * @param filename
     */
    public void loadReferencesFromFile(String dir, String filename) {
        String filepath = Resources.createFilepath(dir, filename);
        try {
            Reader reader = new BufferedReader((new InputStreamReader(
                    Objects.requireNonNull(SpotNavGraphLocationConsultant.class.getResourceAsStream(filepath)))));
            SpotMissionLocationRefJson loadedPoseRefs = gson.fromJson(reader, SpotMissionLocationRefJson.class);
            log.debug("Parsed pose references from json:\n" + gson.toJson(loadedPoseRefs));



            for (SpotNavGraphLocationReferenceJson ref : loadedPoseRefs.references) {
                //get the variables from each of the properties, extract for the constructor, ang ensure they're all the same
                addReference(
                        new SpotNavGraphLocationReference(
                                getNextReferenceId(),
                                Factory.createVariable(ref.getVariable()),
                                ref.getProperties().stream().map(Factory::createPredicate).collect(Collectors.toList()),
                                ref.getNodeName(),
                                ref.getGraphId())
                );
            }
        } catch (NullPointerException e) {
            log.error("[loadReferencesFromFile] Error trying to read mission references from json file: " + filepath, e);
        }
    }
}
