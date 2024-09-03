() = successTests() {
  act:actionWithDefaultArgs(1, 1);
  act:actionWithDefaultArgs(1l, 1l);
  act:actionWithDefaultArgs(1, 1, "yes");
}

() = actionWithDefaultArgs(long ?longInput1=1, long ?longInput2=-1, java.lang.String ?stringInput="") {
  op:log("info", "actionWithDefaultArgs called with args: ?longInput1 ?longInput2 ?stringInput");
}