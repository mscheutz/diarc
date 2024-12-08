package edu.tufts.hrilab.vla;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.gui.GuiProvider;
import edu.tufts.hrilab.vla.gui.VLAAdapter;
import edu.tufts.hrilab.vla.util.Util;

import javax.annotation.Nonnull;

public class VLAComponent extends DiarcComponent implements GuiProvider {
    //==========================================================================
    // Implement methods | GuiProvider
    //==========================================================================
    @Nonnull
    @Override
    public String[] getAdapterClassNames() {
        return new String[]{
                VLAAdapter.class.getName()
        };
    }

    /**
     * Returns True is beliefs were sucessfully updated and False otherwise
    */
    public boolean updateBeliefs(ArrayList<Integer> objectRelations, ArrayList<Integer> actionStates) {
        if (objectRelations == null || actionStates == null) {
            return false;
        }

        String[] symbolicStates = Util.getSymbolicStates();
        Map<String, Integer> symbolicStateMap = new HashMap<>();
        for (int i = 0; i < objectRelations.size(); i++) {
            Integer relation = objectRelations.get(i);
            String state = symbolicStates.get(i);
            symbolicStateMap.put(state, relation);
        }

        String[] actionStates = Util.getActionStates();
        Map<String, Integer> actionStateMap = new HashMap<>();
        for (int j = 0; j < actionStates.size(); j++) {
            Integer state = actionStates.get(j);
            String state = symbolicStates.get(i);
            symbolicStateMap.put(state, relation);
        }

        Map<String, String> symbolicBeliefs = Util.reformatInputData(symbolicStateMap)
        Map<String, String> actionBeliefs = Util.reformatInputData(actionStateMap)

        return true;
    }

    /**
     * Array of symbolic functions (ex. rightof(obj3, obj4)
    */
    public List<String> retrieveSymbolicBeliefs() {
        return new ArrayList<>(symbolicBeliefs.values());
    }

    /**
     * Array of action functions (ex. grasping(obj2)
    */
    public List<String> retrieveActionBeliefs() {
        return new ArrayList<>(actionBeliefs.values());
    }

    /**
     * Mapping of object name -> object number used in belief arrays
     * (ex. glazed_rim_procelain_ramekin -> obj2)
    */
    public static Map<String, String> retrieveObjectMapping() {
        return Util.getObjectMapping();
    }
}
