package edu.tufts.hrilab.ava;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

public class FakeMockAvaComponent extends AvaComponent {
    private double batteryLevel = 90.0; // Initial battery level
    private boolean dropped = false;
    private boolean droppedMore = false;
    private boolean startedCharging = false;
    private long startTime;
    private long startChargingTime;

    public FakeMockAvaComponent() {
        super();  // Call the constructor of AvaComponent
        this.startTime = System.currentTimeMillis();
    }

    /**
     * Mock getBatteryLevel function to test out battery monitoring and goals
     *
     * @return Current fake mock battery level from 0-100 (in %)
     */
    @Override
    @Action
    @TRADEService
    public double getBatteryLevel() {
        double xThreshold = -10; // meters
        double testingBatteryLevel = 19.0; // %
        double testingLowBatteryLevel = 9.0; // %
        double timeTil2ndDrop = 10; // seconds
        double timeToCharge = 30; // seconds

        double currX = getPosition().x;
        long currentTime = System.currentTimeMillis();

        if (currX <= xThreshold && !dropped) {
            // Check if the ava is a certain distance and the drop the battery
            log.info("[FakeMockAvaComponent] Dropping battery");
            batteryLevel = testingBatteryLevel;
            dropped = true;
            startTime = System.currentTimeMillis();
        } else if (dropped && !droppedMore && (currentTime - startTime) / 1000 > timeTil2ndDrop) {
            log.info("[FakeMockAvaComponent] Really dropping battery");
            // After certain amount of time drop the battery again
            batteryLevel = testingLowBatteryLevel;
            droppedMore = true;
        } else if (!startedCharging && (dropped || droppedMore) && currX >= -2) {
            log.info("[FakeMockAvaComponent] Starting charging");
            // Start Charging
            startedCharging = true;
            startChargingTime = System.currentTimeMillis();
        } else if (startedCharging && (currentTime - startChargingTime) / 1000 > timeToCharge) {
            // If it is charging for 30 seconds bring it back to normal
            log.info("[FakeMockAvaComponent] Done charging");
            dropped = false;
            droppedMore = false;
            startedCharging = false;
            batteryLevel = 51.0;
        } else if (!dropped && !droppedMore && !startedCharging) {
            // Run normally
            return super.getBatteryLevel();
        }

        log.info("[FakeMockAvaComponent] Battery: {}%", batteryLevel);
        return batteryLevel;
    }
}
