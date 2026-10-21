/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.message.response;
/*
{
        "bearings" : [],
        "beginAngle" : 0,
        "endAngle" : 4.1015238761901855,
        "maxRange" : 10,
        "minRange" : 0.029999999329447746,
        "pose" : {
            "heading" : -2.0507619380950928,
            "x" : 0.18000000715255737,
            "y" : 0,
            "z" : 0.19844500720500946
        },
        "ranges" : []
}*/

public class LidarResponse {
    public double bearings[];
    public double beginAngle;
    public double endAngle;
    public double maxRange;
    public double minRange;
    public PoseResponse pose;
    public double ranges[];
}