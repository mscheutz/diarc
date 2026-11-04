() = test() {
    op:log(info, "This script should fail when trying to find an action for throw");
    if (true) {
      op:log(info, "1");
      if (true) {
        if (~true) {
          op:log(error, "WRONG #1");
          exit(FAIL);
        } elseif (op:gt(2, 1)) {
          op:log(info, "2");
          if (op:gt(1, 2)) {
            act:throw(ball);
            exit(FAIL);
          } elseif (op:gt(1, 3)) {
            op:log(error, "WRONG #3");
            exit(FAIL);
          } else {
            op:log(info, "3");
          }
        } else {
          op:log(error, "WRONG #4");
          exit(FAIL);
        }
      } elseif (op:gt(2, 1)) {
        op:log(error, "WRONG #5");
        exit(FAIL);
      } else {
        op:log(error, "WRONG #6");
        exit(FAIL);
      }
    } else {
      op:log(error, "WRONG #7");
      exit(FAIL);
    }
}

() = test2() {
    for (!i=0; !i lt 5; !i ++) {
      op:log(info, "During: !i");
      if (op:ge(!i, 5)) {
        exit(FAIL);
      } else {
        act:moveTo(locationA);
      }
    }
}

() = test3() {
  while (op:lt(?i, ?target)) {
    op:++(?i, ?i);
    op:log(info, "Counting: ?i");
    act:throw(ball);
    op:sleep(100);
  }
}

() = moveTo(java.lang.String ?destination) {
    java.lang.String !start;

    conditions : {
      or : {
        pre : at(?actor,!start);
        pre : not(at(?actor,!start));
      }
    }
    effects : {
      success : at(?actor,?destination);
      success : not(at(?actor,!start));
    }

    op:log(info, "?actor moved from !start to ?destination");
    act:throw(ball);
}
