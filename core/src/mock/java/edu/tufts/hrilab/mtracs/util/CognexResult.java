/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs.util;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * CognexResult represents the data from a single Cognex Tool,
 * as defined by the format we have selected for our Cognex Jobs.
 * Each Cognex Result has a confidence score (or other number expressed as a Double), and a location described
 * by an x, y, and theta value, where theta represents a rotation around the z axis.
 * The ethernet communication string of a CognexJob must deliver the results of CognexTools in the order described above
 * in order to work with our system.
 */
public class CognexResult {
    //Members
    private double xOff;
    private double xAbs;
    private double yOff;
    private double yAbs;
    private double theta;
    private double confidenceScore;

    @Override
    public String toString() {
        return "CognexResult{" +
                "xOff=" + xOff +
                ", xAbs=" + xAbs +
                ", yOff=" + yOff +
                ", yAbs=" + yAbs +
                ", theta=" + theta +
                ", confidenceScore=" + confidenceScore +
                ", COGNEX_Y_IND=" + COGNEX_Y_IND +
                ", COGNEX_X_IND=" + COGNEX_X_IND +
                ", COGNEX_T_IND=" + COGNEX_T_IND +
                '}';
    }

    private final int COGNEX_Y_IND = 1;
    private final int COGNEX_X_IND = 0;
    private final int COGNEX_T_IND = 5;

    //Misc
    private Logger log = LoggerFactory.getLogger(this.getClass());

    /**
     * Constructs a CognexResult given the ethernet communication string representation of a Cognex Tool result, the pose that the picture
     * was taken at, and a 3 dimensional point representing the offset from the end effector to the Cognex.
     *
     * @param oneResult The String representation of one Cognex Tool result from the ethernet communication string.
     * @param takenAt The Pose that the arm was in when the Cognex Job was triggered.
     */
    public CognexResult(String oneResult, MPose takenAt) {
        //0 will be score, 1 will be coords
        //Constants
        log.debug("[CognexResult] raw cognex string: " + oneResult);
        String SCORE_SEPARATOR = "s";
        String[] scoreAndCoords = oneResult.split(SCORE_SEPARATOR, 0);
        if (scoreAndCoords.length != 2) {
            throw new IllegalArgumentException("Unable to parse cognex score - are you sure your job is configured properly?");
        }
        confidenceScore = Double.parseDouble(scoreAndCoords[0]);
        //Half 0 is coordinate data, 1 is structural flags (we ignore these for now)
        String HALF_SEPARATOR = "\\)";
        String[] halves = scoreAndCoords[1].split(HALF_SEPARATOR, 0);
        if (halves.length != 3) {
            throw new IllegalArgumentException("Unable to parse cognex halves - are you sure your job is configured properly?");
        }
        String COORDS_SEPARATOR = ",";
        String[] coords = halves[0].substring(1).replace("(", "").split(COORDS_SEPARATOR, 0);
        if (coords.length != 8) {
            throw new IllegalArgumentException("Unable to parse cognex coords - are you sure your job is configured properly?");
        }
        log.info("[CognexResult] pose cognex image was taken at: " + takenAt);
        StringBuilder toLog = new StringBuilder("[ ");
        toLog.append("x: " + coords[COGNEX_X_IND]);
        toLog.append(" y: " + coords[COGNEX_Y_IND]);
        toLog.append(" z: " + coords[COGNEX_T_IND]);
        toLog.append(" ]\n");
        log.debug("[CognexResult] " + toLog);

        //Coordinates come in as millimeters - we need to convert
        if (Math.abs(takenAt.getC() - 1.5707) < 0.01) {
            //todo: no. bad logging.
            log.debug("[CognexResult] robot conveyor orientation. Robot C value: " + takenAt.getC());
            xOff = (Double.parseDouble(coords[COGNEX_Y_IND]) / 1000.0);
            yOff = (Double.parseDouble(coords[COGNEX_X_IND]) / -1000.0);
        } else {
            log.debug("[CognexResult] robot work area etc. orientation. Robot C value: " + takenAt.getC());
            xOff = (Double.parseDouble(coords[COGNEX_X_IND]) / 1000.0);
            yOff = (Double.parseDouble(coords[COGNEX_Y_IND]) / 1000.0);
        }
        xAbs = xOff + takenAt.getXMeters();
        yAbs = yOff + takenAt.getYMeters();
        log.debug("[CognexResult] xAbs: "+xAbs+" yAbs: "+yAbs);
        theta = Double.parseDouble(coords[COGNEX_T_IND]);
        if (Math.abs(theta) > 180) {
            log.warn("WARNING: Cognex result parsed with an over 180 degree rotation - currently not implemented - TODO: Will");
        }
        theta = -Math.toRadians(theta);
        log.debug("[CognexResult] xAbs: "+xAbs+" yAbs: "+yAbs+" theta: "+theta);
    }

    /**
     * Returns the displacement on the x axis of the target from the center of the camera.
     *
     * @return the x displacement of the target in meters.
     */
    public double getXOffset() {
        return xOff;
    }

    /**
     * Returns the absolute x value of the target in the robot base frame.
     *
     * @return the absolute x value of the target in meters.
     */
    public double getXAbs() {
        return xAbs;
    }

    /**
     * Returns the displacement on the y axis of the target from the center of the camera.
     *
     * @return the y displacement of the target in meters.
     */
    public double getYOffset() {
        return yOff;
    }

    /**
     * Returns the absolute y value of the target in the robot base frame.
     *
     * @return the absolute y value of the target in meters.
     */
    public double getYAbs() {
        return yAbs;
    }

    /**
     * Returns the rotation about the z axis of the target from the center of the camera.
     *
     * @return the z rotation of the target in radians.
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Returns the confidence score or other numerical data about the target.
     *
     * @return the double representing the confidence of the target from 0 - 1.
     */
    public double getConfidenceScore() {
        return confidenceScore;
    }

    /**
     * Returns a convenient predicate representation of <code>this</code> CognexResult
     *
     * @return the x, y, and rotation values of <code>this</code> CognexResult as a <code>coords</code> predicate.
     */
    public Symbol getCoords() {
        return new Predicate(
                "coords",
                String.valueOf(getXAbs()),
                String.valueOf(getYAbs()),
                String.valueOf(getTheta()));
    }

    /**
     * Returns a pose representation of <code>this</code> CognexResult
     *
     * @return the x, y, and rotation values of <code>this</code> CognexResult as a MPose.
     */
    public MPose getPose() {
        return new MPose((float)(getXAbs()/1000), (float)(getYAbs()/1000.0f), 0.0f, 0.0f, 0.0f, (float)getTheta());
    }
}
