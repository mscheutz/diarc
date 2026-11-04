import java.lang.Long;
import java.lang.String;

() = test() {
  Long !long1;
  Long !long2;
  String !string1;

  Long !long1_target = 1;
  Long !long2_target = 2;
  String !string1_target = "Yay";

  (!long1, !long2, !string1) = act:returnMultipleArgs();
  op:log("info", "Values: !long1 !long2 !string1");

  if (op:!=(!long1,!long1_target)) {
    exit(FAIL);
  }
  if (op:!=(!long2,!long2_target)) {
    exit(FAIL);
  }
  if (op:!=(!string1,!string1_target)) {
    exit(FAIL);
  }

}

(Long ?long1, Long ?long2, String ?string1) = returnMultipleArgs() {
    ?long1 = op:newObject("java.lang.Long", 1);
    ?long2 = op:newObject("java.lang.Long", 2);
    ?string1 = op:newObject("java.lang.String", "Yay");
}

