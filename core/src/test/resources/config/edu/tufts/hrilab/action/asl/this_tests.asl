import java.lang.String;

() = aslThisTest() {
  String !cmd = this:getCommand();
  op:log("info", "this:getCommand() returned: !cmd");
  if (~op:equals(!cmd, "aslThisTest")) {
    exit(FAIL);
  }

  // wrapping in "if true" to test that "this" always refers to the containing action script (and not the "if context")
  if (true) {
    if (this:isAction()) {
      op:log("info", "this:isAction() returned true.");
    } else {
      op:log("error", "this:isAction() is false. It should always return true!");
      exit(FAIL);
    }
  }
}