// Standalone demonstration leaves; replace the bindings with real robot actions.
import java.lang.Boolean;
import edu.tufts.hrilab.fol.Symbol;

(Boolean ?ready) = exampleBatteryOK() {
  ?ready = op:equals(true, true);
}

() = exampleMoveBase(Symbol ?destination) {
  op:log("info", "Moving to ?destination");
  op:sleep(200);
  op:log("info", "Arrived at ?destination");
}

() = exampleRecharge() {
  op:log("info", "Recharging");
  op:sleep(100);
}

() = exampleInspectCamera() {
  op:log("info", "Inspecting camera");
}
