() = javaMethodsTest() {
    edu.tufts.hrilab.vision.stm.MemoryObject !mo;
    long !typeId = 0;
    long !tokenId = 0;
    edu.tufts.hrilab.fol.Term !desc;
    edu.tufts.hrilab.fol.Variable !var;
    double !x = 1;
    double !y = 1;
    double !z = 1;
    boolean !exists;
    java.lang.Object !obj;

    // test newObject methods
    !mo = op:newObject("edu.tufts.hrilab.vision.stm.MemoryObject");
    !var = op:newObject("edu.tufts.hrilab.fol.Variable", "X");
    !desc = op:newObject("edu.tufts.hrilab.fol.Term", "test", !var);

    // test invokeMethod methods
    op:invokeMethod(!mo, "setVariable", !var);
    op:invokeMethod(!mo, "setDirection", !x, !y, !z);
    op:invokeMethod(!mo, "setTokenId", !tokenId);
    op:invokeMethod(!mo, "printSceneGraph");

    op:log("info", "Hello memory object: !mo");

    // test invokeMethod methods
    op:invokeMethod(!mo, "addDescriptor", !desc, 0.5f);
    !exists = op:invokeMethod(!mo, "containsDescriptor", !desc);
    op:log("info", "memory object contains descriptor: !desc !exists");

    !typeId = op:invokeMethod(!mo, "getTypeId");
    op:log("info", "memory object typeId: !typeId");

    // test invokeStaticMethod
    !x = op:invokeStaticMethod("edu.tufts.hrilab.vision.stm.MemoryObject", "roundToXDigits", 0.12345d, 3);
    op:log("info", "rounded double: !x");
    !desc = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createPredicate", "text(Y)");
    op:log("info", "createPredicate: !desc");

    // test invokeStaticMethod
    op:invokeStaticMethod("edu.tufts.hrilab.util.Util", "Sleep", 3);
}

() = controlTestWithJavaMethods() {
    edu.tufts.hrilab.fol.Symbol !testArg = symbolName;
    boolean !value;

    if (op:invokeStaticMethod("edu.tufts.hrilab.action.operators.MiscJavaMethods", "getTrueStaticNoArgs")) {
      op:log("info", "getTrueStaticNoArgs test passed");
    } else {
      op:log("info", "getTrueStaticNoArgs test failed");
      exit(FAIL);
    }

    if (op:invokeStaticMethod("edu.tufts.hrilab.action.operators.MiscJavaMethods", "getFalseStaticNoArgs")) {
      op:log("info", "getFalseStaticNoArgs test failed");
      exit(FAIL);
    } else {
      op:log("info", "getFalseStaticNoArgs test passed");
    }

    if (op:invokeStaticMethod("edu.tufts.hrilab.action.operators.MiscJavaMethods", "getTrueStaticWithArgs", !testArg)) {
      op:log("info", "getTrueStaticWithArgs test passed");
    } else {
      op:log("info", "getTrueStaticWithArgs test failed");
      exit(FAIL);
    }

    if (op:invokeStaticMethod("edu.tufts.hrilab.action.operators.MiscJavaMethods", "getFalseStaticWithArgs", !testArg)) {
      op:log("info", "getFalseStaticWithArgs test failed");
      exit(FAIL);
    } else {
      op:log("info", "getFalseStaticWithArgs test passed");
    }
}
