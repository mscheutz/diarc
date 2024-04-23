() = test1() {
    op:log("info", "This script should count to 3. If it doesn't, or prints WRONG, and fails.");
    if (true) {
      op:log("info", "1");
      if (true) {
        if (~true) {
          op:log("error", "WRONG #1");
          exit(FAIL);
        } elseif (op:gt(2, 1)) {
          op:log("info", "2");
          if (op:gt(1, 2)) {
            op:log("error", "WRONG #2");
            exit(FAIL);
          } elseif (op:gt(1, 3)) {
            op:log("error", "WRONG #3");
            exit(FAIL);
          } else {
            op:log("info", "3");
          }
        } else {
          op:log("error", "WRONG #4");
          exit(FAIL);
        }
      } elseif (op:gt(2, 1)) {
        op:log("error", "WRONG #5");
        exit(FAIL);
      } else {
        op:log("error", "WRONG #6");
        exit(FAIL);
      }
    } else {
      op:log("error", "WRONG #7");
      exit(FAIL);
    }
}

() = test2() {
    if (op:gt(1, 2)) {
      op:log("error", "WRONG #1");
      exit(FAIL);
    } elseif (op:gt(1, 3)) {
      op:log("error", "WRONG #2");
      exit(FAIL);
    } else {
      op:log("info", "OK");
    }
}

() = test3() {
  if (true) {
    op:log("info", "OK");
    if (true) {
        op:log("info", "OK");
    }
  } else {
    op:log("error", "FAIL");
    exit(FAIL);
  }
}

