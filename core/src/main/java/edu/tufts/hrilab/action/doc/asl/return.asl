() = exitTest() {
  if (true) {
    op:log("info", "this should execute");
    return;
  }
  op:log("error", "this should NOT execute");
}