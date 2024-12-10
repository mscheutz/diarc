package edu.tufts.hrilab.vla;

import com.google.gson.Gson;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.gui.GuiProvider;
import edu.tufts.hrilab.vla.gui.VLAAdapter;
import edu.tufts.hrilab.vla.util.Util;

import javax.annotation.Nonnull;
import java.util.*;

public class VLAComponent extends DiarcComponent implements GuiProvider {
    private VLAAdapter adapter;
    private Map<String, String> symbolicBeliefs = new HashMap<>();
    private Map<String, String> actionBeliefs = new HashMap<>();

    public VLAComponent(VLAAdapter adapter) {
        this.adapter = adapter;
    }

    // Implement methods | GuiProvider
    @Nonnull
    @Override
    public String[] getAdapterClassNames() {
        return new String[]{
                VLAAdapter.class.getName()
        };
    }

    // Call with prepareBeliefsForDisplay() as the input
    public void sendMessageToGui(String message) {
        if (adapter != null) {
            adapter.sendToViewer(message);
        } else {
            System.out.println("Adapter is not connected.");
        }
    }

    /**
     * Updates beliefs using provided relations and actions.
     *
     * @return true if beliefs updated successfully, false otherwise.
     */
    public boolean updateBeliefs(ArrayList<Integer> objectRelations, ArrayList<Integer> actionStates) {
        if (objectRelations == null || actionStates == null) {
            System.err.println("Input lists cannot be null.");
            return false;
        }

        try {
            String[] symbolicStates = Util.getSymbolicStates();
            Map<String, Integer> symbolicStateMap = new HashMap<>();
            for (int i = 0; i < objectRelations.size(); i++) {
                symbolicStateMap.put(symbolicStates[i], objectRelations.get(i));
            }

            String[] actionStateLabels = Util.getActionStates();
            Map<String, Integer> actionStateMap = new HashMap<>();
            for (int j = 0; j < actionStates.size(); j++) {
                actionStateMap.put(actionStateLabels[j], actionStates.get(j));
            }

            symbolicBeliefs = Util.reformatInputData(symbolicStateMap);
            actionBeliefs = Util.reformatInputData(actionStateMap);

            return !symbolicBeliefs.isEmpty() && !actionBeliefs.isEmpty();
        } catch (Exception e) {
            System.err.println("Error updating beliefs: " + e.getMessage());
            return false;
        }
    }

    /**
     * Array of symbolic functions (ex. rightof(obj3, obj4))
     */
    public List<String> retrieveSymbolicBeliefs() {
        return new ArrayList<>(symbolicBeliefs.values());
    }

    /**
     * Array of action functions (ex. grasping(obj2))
     */
    public List<String> retrieveActionBeliefs() {
        return new ArrayList<>(actionBeliefs.values());
    }

    /**
     * Convert beliefs to JSON for WebSocket transmission
     */
    public String prepareBeliefsForDisplay() {
        List<String> symbolic = retrieveSymbolicBeliefs();
        List<String> action = retrieveActionBeliefs();

        BeliefDisplayData displayData = new BeliefDisplayData(symbolic, action);
        return new Gson().toJson(displayData);
    }

    // Inner class to represent data structure for beliefs
    private static class BeliefDisplayData {
        private List<String> symbolicBeliefs;
        private List<String> actionBeliefs;

        public BeliefDisplayData(List<String> symbolicBeliefs, List<String> actionBeliefs) {
            this.symbolicBeliefs = symbolicBeliefs;
            this.actionBeliefs = actionBeliefs;
        }
    }
}
