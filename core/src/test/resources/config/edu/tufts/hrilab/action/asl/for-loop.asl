() = test(int ?argument = 10) {
    int !local = 100;
    for (!i=0; !i lt 5; !i ++) {
        op:log("info", "During: !i");
        if (op:ge(!i, 5)) {
            exit(FAIL);
        }
    }
    op:log("info", "Argument: ?argument");
    for (?argument; ?argument gt 0; ?argument /=2) {
        op:log("info", "During: ?argument");
    }
    op:log("info", "Argument after: ?argument");
    if (op:gt(?argument, 0)) {
        op:log("info", "test failed with argument: ?argument");
        exit(FAIL);
    }
}

