() = tryCatchTestFail() {
    try {
      exit(FAIL_NOTFOUND); // should not be caught
    } catch(FAIL_PRECONDITIONS) {
      // shouldn't get caught as it's the wrong catch status (should be FAIL_NOTFOUND)
    }
}

() = tryCatchTestSuccess() {
    try {
      exit(FAIL_NOTFOUND); // should be caught
    } catch() {
      // default catch should catch all failure types
    }

    try {
      exit(FAIL_NOTFOUND); // should be caught
    } catch(FAIL_NOTFOUND) {
      // specific catch should catch the expected status of FAIL_NOTFOUND
    }

    try {
    } catch() {
      // nothing should be caught
      exit(FAIL); // try block shouldn't have failed
    }
}

() = finallyTest1() {
    try {
      exit(FAIL_NOTFOUND); // should be caught
    } catch() {
      // default catch should catch all failure types
    } finally {
      exit(FAIL); // this should execute --> action fails
    }
}

() = finallyTest2() {
    try {
      exit(FAIL_NOTFOUND); // should be caught
    } catch(FAIL_NOTFOUND) {
      // specific catch should catch the expected status of FAIL_NOTFOUND
    } finally {
     exit(FAIL); // this should execute --> action fails
     }
}

() = finallyTest3() {
    try {
    } catch() {
      // nothing should be caught
      exit(FAIL); // try block shouldn't have failed
    } finally {
      exit(FAIL); // this should execute --> action fails
    }
}

() = finallyTest4() {
    try {
    } finally {
      exit(FAIL); // this should execute --> action fails
    }
}

