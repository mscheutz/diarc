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
import edu.tufts.hrilab.fol.Variable;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point2d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.stream.Collectors;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;

public class SpotLocationConsultant extends Consultant<SpotLocationReference> {

    final double DISTANCE_EPSILON = 0.01;

    private static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();

    public SpotLocationConsultant(String kbName) {
        super(SpotLocationReference.class, kbName);
    }

    public SpotLocationConsultant(Class<SpotLocationReference> refClass, String kbName, List<String> properties) {
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

    public SpotLocationReference getRefWithId(Symbol refId) {
        return references.get(refId);
    }

    public Symbol getClosestLocationToPoint(Pair<Point2d, Quat4d> pose) {
        Point2d position = pose.getLeft();
        Quat4d orientation = pose.getRight();

        double min = Double.MAX_VALUE;
        double minAngle = Double.MAX_VALUE;
        Symbol toReturn = null;
        for (Symbol refId : references.keySet()) {
            SpotLocationReference ref = references.get(refId);
            if (ref.position == null || ref.orientation == null) {
                log.warn("[getClosestLocationToPoint] skipping reference with refId " + refId + " since its x,y location has not been initialized");
            } else {
                double distance = ref.position.distanceSquared(position);
                if (distance < min || abs(distance - min) < DISTANCE_EPSILON) {
                    Quat4d diff =  new Quat4d(orientation);
                    diff.inverse();
                    diff.mul(ref.orientation);
                    double angle = abs(2 * atan2(new Vector3d(diff.x, diff.y, diff.z).length(), diff.w)); //todo: i think this is right?
                    if (angle < minAngle) {
                        min = distance;
                        minAngle = angle;
                        toReturn = refId;
                    }
                }
            }
        }
        return toReturn;
    }

    //todo: consolidate json and reference-loading with functionality in PoseConsultant?
    class SpotMissionLocationRefJson {
        public List<SpotLocationReferenceJson>  references = new ArrayList<>();
    }

    /**
     * Load pre-defined references from file via JSON.
     * @param filename
     */
    public void loadReferencesFromFile(String filename) {
        InputStream stream = SpotLocationConsultant.class.getResourceAsStream(filename);
        if (stream == null) {
            log.error("Resource not found: " + filename);
            return;
        }

        try {
            Reader reader = new BufferedReader(new InputStreamReader(Objects.requireNonNull(stream)));
            SpotMissionLocationRefJson loadedPoseRefs = gson.fromJson(reader, SpotMissionLocationRefJson.class);
            log.debug("Parsed pose references from json:\n" + gson.toJson(loadedPoseRefs));

            for (SpotLocationReferenceJson ref : loadedPoseRefs.references) {
                //get the variables from each of the properties, extract for the constructor, ang ensure they're all the same
                addReference(
                        new SpotLocationReference(
                                getNextReferenceId(),
                                Factory.createVariable(ref.getVariable()),
                                ref.getProperties().stream().map(Factory::createPredicate).collect(Collectors.toList()),
                                ref.getGraphId())
                );
            }
        } catch (NullPointerException e) {
            log.error("[loadReferencesFromFile] Error trying to read mission references from json file: " + filename, e);
        }
    }

    public boolean writeReferencesToFile(String absolutePath) {
        SpotMissionLocationRefJson json = new SpotMissionLocationRefJson();
        List<SpotLocationReference> sortedReferences = references.entrySet().stream().sorted(
                Comparator.comparing(a -> a.getKey().getName())
        ).map(Map.Entry::getValue).toList();
        for (SpotLocationReference ref : sortedReferences) {
            json.references.add(new SpotLocationReferenceJson(ref));
        }
        String jsonString = gson.toJson(json);
        File file = new File(absolutePath);
        try (FileOutputStream out = new FileOutputStream(file)) {
            out.write(jsonString.getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            log.error("[writeReferencesToFile] error writing references to file: " + e);
            return false;
        }
        return true;
    }

    public void createReferenceWithWaypointId(String waypointId, Symbol property) {
        log.info("Creating reference with waypointId: {} and property: {}", waypointId, property);
        Variable var = Factory.createVariable("VAR0", kbName);
        SpotLocationReference ref = createReference(var, List.of(Factory.createPredicate(property.getName(), var)));
        ref.setWaypointId(waypointId);
    }
}
