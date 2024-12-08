import java.util.HashMap;
import java.util.Map;

public class Util {
    private static Map<String, String> objectMapping = new HashMap<>();

    public static Map<String, String> reformatInputData(Map<String, Integer> inputData) {
        Map<String, String> result = new HashMap<>();
        objectMapping.clear();
        int objectCounter = 1;

        for (String originalKey : inputData.keySet()) {
            String[] components = originalKey.split(" ");
            String action = components[0]; // The first word is the action

            String obj1 = null;
            String obj2 = null;

            if (components.length == 2) {
                obj1 = components[1];
            } else if (components.length >= 3) {
                obj1 = components[1];
                obj2 = components[2];
            } else {
                throw new IllegalArgumentException("Unexpected key format: " + originalKey);
            }
            if (!objectMapping.containsKey(obj1)) {
                objectMapping.put(obj1, "obj" + objectCounter);
                objectCounter++;
            }
            if (obj2 != null && !objectMapping.containsKey(obj2)) {
                objectMapping.put(obj2, "obj" + objectCounter);
                objectCounter++;
            }
            String funcString;
            if (obj2 != null) {
                funcString = String.format("%s(%s, %s)", action, objectMapping.get(obj1), objectMapping.get(obj2));
            } else {
                funcString = String.format("%s(%s)", action, objectMapping.get(obj1));
            }
            result.put(originalKey, funcString);
        }
        return result;
    }

    public static Map<String, String> getObjectMapping() {
        return new HashMap<>(objectMapping);
    }

    public static String[] getActionStates() {
        return new String[] {
            "grasped akita_black_bowl_1",
            "grasped akita_black_bowl_2",
            "grasped cookies_1",
            "grasped glazed_rim_porcelain_ramekin_1",
            "grasped plate_1",
            "should-move-towards akita_black_bowl_1",
            "should-move-towards akita_black_bowl_2",
            "should-move-towards cookies_1",
            "should-move-towards flat_stove_1",
            "should-move-towards glazed_rim_porcelain_ramekin_1",
            "should-move-towards plate_1",
            "should-move-towards wooden_cabinet_1"
        };
    }
    public static String[] getSymbolicStates() {
        return new String[] {
            "behind akita_black_bowl_1 akita_black_bowl_2",
            "behind akita_black_bowl_1 cookies_1",
            "behind akita_black_bowl_1 flat_stove_1",
            "behind akita_black_bowl_1 glazed_rim_porcelain_ramekin_1",
            "behind akita_black_bowl_1 plate_1",
            "behind akita_black_bowl_1 wooden_cabinet_1",
            "behind akita_black_bowl_2 akita_black_bowl_1",
            "behind akita_black_bowl_2 cookies_1",
            "behind akita_black_bowl_2 flat_stove_1",
            "behind akita_black_bowl_2 glazed_rim_porcelain_ramekin_1",
            "behind akita_black_bowl_2 plate_1",
            "behind akita_black_bowl_2 wooden_cabinet_1",
            "behind cookies_1 akita_black_bowl_1",
            "behind cookies_1 akita_black_bowl_2",
            "behind cookies_1 flat_stove_1",
            "behind cookies_1 glazed_rim_porcelain_ramekin_1",
            "behind cookies_1 plate_1",
            "behind cookies_1 wooden_cabinet_1",
            "behind flat_stove_1 akita_black_bowl_1",
            "behind flat_stove_1 akita_black_bowl_2",
            "behind flat_stove_1 cookies_1",
            "behind flat_stove_1 glazed_rim_porcelain_ramekin_1",
            "behind flat_stove_1 plate_1",
            "behind flat_stove_1 wooden_cabinet_1",
            "behind glazed_rim_porcelain_ramekin_1 akita_black_bowl_1",
            "behind glazed_rim_porcelain_ramekin_1 akita_black_bowl_2",
            "behind glazed_rim_porcelain_ramekin_1 cookies_1",
            "behind glazed_rim_porcelain_ramekin_1 flat_stove_1",
            "behind glazed_rim_porcelain_ramekin_1 plate_1",
            "behind glazed_rim_porcelain_ramekin_1 wooden_cabinet_1",
            "behind plate_1 akita_black_bowl_1",
            "behind plate_1 akita_black_bowl_2",
            "behind plate_1 cookies_1",
            "behind plate_1 flat_stove_1",
            "behind plate_1 glazed_rim_porcelain_ramekin_1",
            "behind plate_1 wooden_cabinet_1",
            "behind wooden_cabinet_1 akita_black_bowl_1",
            "behind wooden_cabinet_1 akita_black_bowl_2",
            "behind wooden_cabinet_1 cookies_1",
            "behind wooden_cabinet_1 flat_stove_1",
            "behind wooden_cabinet_1 glazed_rim_porcelain_ramekin_1",
            "behind wooden_cabinet_1 plate_1",
            "in-front-of akita_black_bowl_1 akita_black_bowl_2",
            "in-front-of akita_black_bowl_1 cookies_1",
            "in-front-of akita_black_bowl_1 flat_stove_1",
            "in-front-of akita_black_bowl_1 glazed_rim_porcelain_ramekin_1",
            "in-front-of akita_black_bowl_1 plate_1",
            "in-front-of akita_black_bowl_1 wooden_cabinet_1",
            "in-front-of akita_black_bowl_2 akita_black_bowl_1",
            "in-front-of akita_black_bowl_2 cookies_1",
            "in-front-of akita_black_bowl_2 flat_stove_1",
            "in-front-of akita_black_bowl_2 glazed_rim_porcelain_ramekin_1",
            "in-front-of akita_black_bowl_2 plate_1",
            "in-front-of akita_black_bowl_2 wooden_cabinet_1",
            "in-front-of cookies_1 akita_black_bowl_1",
            "in-front-of cookies_1 akita_black_bowl_2",
            "in-front-of cookies_1 flat_stove_1",
            "in-front-of cookies_1 glazed_rim_porcelain_ramekin_1",
            "in-front-of cookies_1 plate_1",
            "in-front-of cookies_1 wooden_cabinet_1",
            "in-front-of flat_stove_1 akita_black_bowl_1",
            "in-front-of flat_stove_1 akita_black_bowl_2",
            "in-front-of flat_stove_1 cookies_1",
            "in-front-of flat_stove_1 glazed_rim_porcelain_ramekin_1",
            "in-front-of flat_stove_1 plate_1",
            "in-front-of flat_stove_1 wooden_cabinet_1",
            "in-front-of glazed_rim_porcelain_ramekin_1 akita_black_bowl_1",
            "in-front-of glazed_rim_porcelain_ramekin_1 akita_black_bowl_2",
            "in-front-of glazed_rim_porcelain_ramekin_1 cookies_1",
            "in-front-of glazed_rim_porcelain_ramekin_1 flat_stove_1",
            "in-front-of glazed_rim_porcelain_ramekin_1 plate_1",
            "in-front-of glazed_rim_porcelain_ramekin_1 wooden_cabinet_1",
            "in-front-of plate_1 akita_black_bowl_1",
            "in-front-of plate_1 akita_black_bowl_2",
            "in-front-of plate_1 cookies_1",
            "in-front-of plate_1 flat_stove_1",
            "in-front-of plate_1 glazed_rim_porcelain_ramekin_1",
            "in-front-of plate_1 wooden_cabinet_1",
            "in-front-of wooden_cabinet_1 akita_black_bowl_1",
            "in-front-of wooden_cabinet_1 akita_black_bowl_2",
            "in-front-of wooden_cabinet_1 cookies_1",
            "in-front-of wooden_cabinet_1 flat_stove_1",
            "in-front-of wooden_cabinet_1 glazed_rim_porcelain_ramekin_1",
            "in-front-of wooden_cabinet_1 plate_1",
            "inside akita_black_bowl_1 wooden_cabinet_1_bottom_region",
            "inside akita_black_bowl_1 wooden_cabinet_1_middle_region",
            "inside akita_black_bowl_1 wooden_cabinet_1_top_region",
            "inside akita_black_bowl_2 wooden_cabinet_1_bottom_region",
            "inside akita_black_bowl_2 wooden_cabinet_1_middle_region",
            "inside akita_black_bowl_2 wooden_cabinet_1_top_region",
            "inside cookies_1 wooden_cabinet_1_bottom_region",
            "inside cookies_1 wooden_cabinet_1_middle_region",
            "inside cookies_1 wooden_cabinet_1_top_region",
            "inside glazed_rim_porcelain_ramekin_1 wooden_cabinet_1_bottom_region",
            "inside glazed_rim_porcelain_ramekin_1 wooden_cabinet_1_middle_region",
            "inside glazed_rim_porcelain_ramekin_1 wooden_cabinet_1_top_region",
            "inside plate_1 wooden_cabinet_1_bottom_region",
            "inside plate_1 wooden_cabinet_1_middle_region",
            "inside plate_1 wooden_cabinet_1_top_region",
            "left-of akita_black_bowl_1 akita_black_bowl_2",
            "left-of akita_black_bowl_1 cookies_1",
            "left-of akita_black_bowl_1 flat_stove_1",
            "left-of akita_black_bowl_1 glazed_rim_porcelain_ramekin_1",
            "left-of akita_black_bowl_1 plate_1",
            "left-of akita_black_bowl_1 wooden_cabinet_1",
            "left-of akita_black_bowl_2 akita_black_bowl_1",
            "left-of akita_black_bowl_2 cookies_1",
            "left-of akita_black_bowl_2 flat_stove_1",
            "left-of akita_black_bowl_2 glazed_rim_porcelain_ramekin_1",
            "left-of akita_black_bowl_2 plate_1",
            "left-of akita_black_bowl_2 wooden_cabinet_1",
            "left-of cookies_1 akita_black_bowl_1",
            "left-of cookies_1 akita_black_bowl_2",
            "left-of cookies_1 flat_stove_1",
            "left-of cookies_1 glazed_rim_porcelain_ramekin_1",
            "left-of cookies_1 plate_1",
            "left-of cookies_1 wooden_cabinet_1",
            "left-of flat_stove_1 akita_black_bowl_1",
            "left-of flat_stove_1 akita_black_bowl_2",
            "left-of flat_stove_1 cookies_1",
            "left-of flat_stove_1 glazed_rim_porcelain_ramekin_1",
            "left-of flat_stove_1 plate_1",
            "left-of flat_stove_1 wooden_cabinet_1",
            "left-of glazed_rim_porcelain_ramekin_1 akita_black_bowl_1",
            "left-of glazed_rim_porcelain_ramekin_1 akita_black_bowl_2",
            "left-of glazed_rim_porcelain_ramekin_1 cookies_1",
            "left-of glazed_rim_porcelain_ramekin_1 flat_stove_1",
            "left-of glazed_rim_porcelain_ramekin_1 plate_1",
            "left-of glazed_rim_porcelain_ramekin_1 wooden_cabinet_1",
            "left-of plate_1 akita_black_bowl_1",
            "left-of plate_1 akita_black_bowl_2",
            "left-of plate_1 cookies_1",
            "left-of plate_1 flat_stove_1",
            "left-of plate_1 glazed_rim_porcelain_ramekin_1",
            "left-of plate_1 wooden_cabinet_1",
            "left-of wooden_cabinet_1 akita_black_bowl_1",
            "left-of wooden_cabinet_1 akita_black_bowl_2",
            "left-of wooden_cabinet_1 cookies_1",
            "left-of wooden_cabinet_1 flat_stove_1",
            "left-of wooden_cabinet_1 glazed_rim_porcelain_ramekin_1",
            "left-of wooden_cabinet_1 plate_1",
            "on akita_black_bowl_1 akita_black_bowl_2",
            "on akita_black_bowl_1 cookies_1",
            "on akita_black_bowl_1 flat_stove_1",
            "on akita_black_bowl_1 glazed_rim_porcelain_ramekin_1",
            "on akita_black_bowl_1 plate_1",
            "on akita_black_bowl_1 wooden_cabinet_1",
            "on akita_black_bowl_2 akita_black_bowl_1",
            "on akita_black_bowl_2 cookies_1",
            "on akita_black_bowl_2 flat_stove_1",
            "on akita_black_bowl_2 glazed_rim_porcelain_ramekin_1",
            "on akita_black_bowl_2 plate_1",
            "on akita_black_bowl_2 wooden_cabinet_1",
            "on cookies_1 akita_black_bowl_1",
            "on cookies_1 akita_black_bowl_2",
            "on cookies_1 flat_stove_1",
            "on cookies_1 glazed_rim_porcelain_ramekin_1",
            "on cookies_1 plate_1",
            "on cookies_1 wooden_cabinet_1",
            "on glazed_rim_porcelain_ramekin_1 akita_black_bowl_1",
            "on glazed_rim_porcelain_ramekin_1 akita_black_bowl_2",
            "on glazed_rim_porcelain_ramekin_1 cookies_1",
            "on glazed_rim_porcelain_ramekin_1 flat_stove_1",
            "on glazed_rim_porcelain_ramekin_1 plate_1",
            "on glazed_rim_porcelain_ramekin_1 wooden_cabinet_1",
            "on plate_1 akita_black_bowl_1",
            "on plate_1 akita_black_bowl_2",
            "on plate_1 cookies_1",
            "on plate_1 flat_stove_1",
            "on plate_1 glazed_rim_porcelain_ramekin_1",
            "on plate_1 wooden_cabinet_1",
            "on-table akita_black_bowl_1",
            "on-table akita_black_bowl_2",
            "on-table cookies_1",
            "on-table flat_stove_1",
            "on-table glazed_rim_porcelain_ramekin_1",
            "on-table plate_1",
            "on-table wooden_cabinet_1"
            "open wooden_cabinet_1_bottom_region",
            "open wooden_cabinet_1_middle_region",
            "open wooden_cabinet_1_top_region",
            "right-of akita_black_bowl_1 akita_black_bowl_2",
            "right-of akita_black_bowl_1 cookies_1",
            "right-of akita_black_bowl_1 flat_stove_1",
            "right-of akita_black_bowl_1 glazed_rim_porcelain_ramekin_1",
            "right-of akita_black_bowl_1 plate_1",
            "right-of akita_black_bowl_1 wooden_cabinet_1",
            "right-of akita_black_bowl_2 akita_black_bowl_1",
            "right-of akita_black_bowl_2 cookies_1",
            "right-of akita_black_bowl_2 flat_stove_1",
            "right-of akita_black_bowl_2 glazed_rim_porcelain_ramekin_1",
            "right-of akita_black_bowl_2 plate_1",
            "right-of akita_black_bowl_2 wooden_cabinet_1",
            "right-of cookies_1 akita_black_bowl_1",
            "right-of cookies_1 akita_black_bowl_2",
            "right-of cookies_1 flat_stove_1",
            "right-of cookies_1 glazed_rim_porcelain_ramekin_1",
            "right-of cookies_1 plate_1",
            "right-of cookies_1 wooden_cabinet_1",
            "right-of flat_stove_1 akita_black_bowl_1",
            "right-of flat_stove_1 akita_black_bowl_2",
            "right-of flat_stove_1 cookies_1",
            "right-of flat_stove_1 glazed_rim_porcelain_ramekin_1",
            "right-of flat_stove_1 plate_1",
            "right-of flat_stove_1 wooden_cabinet_1",
            "right-of glazed_rim_porcelain_ramekin_1 akita_black_bowl_1",
            "right-of glazed_rim_porcelain_ramekin_1 akita_black_bowl_2",
            "right-of glazed_rim_porcelain_ramekin_1 cookies_1",
            "right-of glazed_rim_porcelain_ramekin_1 flat_stove_1",
            "right-of glazed_rim_porcelain_ramekin_1 plate_1",
            "right-of glazed_rim_porcelain_ramekin_1 wooden_cabinet_1",
            "right-of plate_1 akita_black_bowl_1",
            "right-of plate_1 akita_black_bowl_2",
            "right-of plate_1 cookies_1",
            "right-of plate_1 flat_stove_1",
            "right-of plate_1 glazed_rim_porcelain_ramekin_1",
            "right-of plate_1 wooden_cabinet_1",
            "right-of wooden_cabinet_1 akita_black_bowl_1",
            "right-of wooden_cabinet_1 akita_black_bowl_2",
            "right-of wooden_cabinet_1 cookies_1",
            "right-of wooden_cabinet_1 flat_stove_1",
            "right-of wooden_cabinet_1 glazed_rim_porcelain_ramekin_1",
            "right-of wooden_cabinet_1 plate_1",
            "turned-on flat_stove_1"
        };
    }

}
