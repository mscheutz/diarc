/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.ava.message.response;

import edu.tufts.hrilab.ava.message.response.PowerStatus;

/*
● health (object) - Status of individual robot subsystems. The property names are the
names of the subsystem, and the values are all one of the following strings, depending on the
health of that subsystem:
○ INIT: Subsystem is trying to initialize
○ OK: Subsystem is OK
○ OK_WARN: Subsystem is OK but has a warning
○ DEGRADED: Subsystem has degraded performance
○ FAULT: Subsystem has a fault
○ MISSING: Subsystem is missing
○ DISABLED: Subsystem is externally disabled
○ RECOVERING: Subsystem is trying to recover from an internal fault
 */



public class HealthResponse {
    public String overallHealth; //OK, OK_WARN, FAULT
    public Health health;
    public RMSDetail detail;
    public PowerStatus powerStatus;
    public String batteryCharge;
}