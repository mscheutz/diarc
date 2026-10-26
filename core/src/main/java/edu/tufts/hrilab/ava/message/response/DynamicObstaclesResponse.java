package edu.tufts.hrilab.ava.message.response;

/*
{
    "obstacles":
    [
        {
            "position":
            {
                "x": "-1.808923",
                "y": "0.4762408",
                "theta": "-3.11544"
            },
            "velocity":
            {
                "x": "-0.006712626",
                "y": "-0.001358689",
                "theta": "0"
            },
            "size":
            {
                "x": "0.2653018",
                "y": "0.08931273"
            },
            "type": "0"
        }
    ]
}
 */

public class DynamicObstaclesResponse {
  public Obstacle[] obstacles;
}
