() = test(long ?long, java.lang.String ?string) {
    java.lang.String !local = "something";
    act:subscript(?long, ?string);
}

() = subscript(long ?long_sub, java.lang.String ?string_sub) {
    long !local;
    try {
        op:log("info", "local value: !local");
    }
    finally {
    }
}

() = return_test() {
    long !ret = 2;
    long !retValue = 3;
    op:gt(1, 2);
    op:log("info", "ret value: !ret (should be 2)");
    op:log("info", "retValue value: !retValue (should be 3)");
}

