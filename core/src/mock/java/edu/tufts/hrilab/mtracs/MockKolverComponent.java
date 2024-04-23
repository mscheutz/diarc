/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;

import java.util.HashMap;
import java.util.Map;

public class MockKolverComponent extends DiarcComponent implements KolverComponentInterface {

    protected Map<Symbol, Integer> symbolIdMap = new HashMap<>();
    protected Integer idCounter;

    @Override
    /**
     * Starts this component, and initializes all values.
     */
    protected void init() {
        idCounter = 1;
    }

    @Override
    public int incrementId() {
        int temp = idCounter;
        this.idCounter++;
        return temp;
    }

    /**
     * Sends the given information about one param in a screwdriving rundown program, and
     * writes it to the Kolver controller over MODBUS.
     *
     * @return True if the screwdriver program was successfully configured,
     * false if otherwise.
     */
    @Override
    public boolean configureScrewdriverParam(Term screwType, Symbol val) {
        log.debug("[configureParam]");

        if (symbolIdMap.containsKey(screwType.getArgs().get(0))) {
            switchProgram(symbolIdMap.get(screwType.getArgs().get(0)));
        } else {
            log.error("Unable to configure parameter for untaught program");
            return false;
        }

        switch (screwType.getName()) {
            case "targetTorque":
                break;
            case "maxTorque":
                break;
            case "angleMax":
                break;
            default:
                log.error("Unknown param: " + screwType);
                return false;
        }

        return true;
    }

    @Override
    public int getScrewdriverProgramIdFromSymbol(Symbol s) {
        if (s == null || !symbolIdMap.containsKey(s)) {
            log.warn("[getScrewdriverProgramIdFromSymbol] no screwdriver program id found for symbol: " + s);
            return -1;
        }
        return symbolIdMap.get(s);
    }

    /**
     * Sends the given information about a screwdriving rundown program, and
     * writes it to the Kolver controller over MODBUS.
     *
     * @return True if the screwdriver program was successfully configured,
     * false if otherwise.
     */
    @Override
    public boolean configureScrewdriverProgram(Symbol screwType, Symbol torqueTarget, Symbol torqueMax,
                                               Symbol angleMax) {

        //lookup
        if (symbolIdMap.containsKey(screwType)) {
            switchProgram(symbolIdMap.get(screwType));
        } else {
            int newId = incrementId();
            log.info("Configuring screwdriver program: " + newId);
            symbolIdMap.put(screwType, newId);
            switchProgram(newId);
        }

        return true;
    }


    /**
     * Starts a program on the Kolver screwdriver controller over MODBUS.
     *
     * @return True if the screwdriver program succeeded, false if otherwise.
     */
    @Override
    public boolean runScrewdriverProgram(int programID) {
        log.debug("[runProgram]");

        return true;
        //Todo: Will: Close connection via some method
    }

    //Todo: Will: Replace the below by instead checking coils 3/4???

    /**
     * Switches the currently queued program on the Kolver controller over MODBUS
     *
     * @param id The program id to switch to as an int.
     * @return True if the program switch succeeded, false if otherwise.
     */
    @Override
    public boolean switchProgram(int id) {
        log.debug("[switchProgram]: Switched to " + id);

        //Todo: Will: is there some sort of way to determine the success of this operation?
        return true;
    }

    //Todo: Will: Why doesn't this work? For now, the above also counts as a reset.

    /**
     * Clears the current screwing status of the Kolver controller. Used to ensure
     * results don't bleed over between rundowns.
     *
     * @return True if the reset succeeded, false if otherwise.
     */
    @Override
    public boolean resetScrewdriver() {
        log.debug("[resetScrewdriver]");
        //Todo: Will: is there some sort of way to determine the success of this operation?
        return true;
    }

}
