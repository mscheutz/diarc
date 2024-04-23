() = stringTest() {
    int !variable = 123;

    // Check that actspec
    //   - accepts strings literals between quotes
    //   - strips the quotes away
    //   - replaces variables by their value
    act:checkString("Hello world !variable");

    // Check the above but with a leading and trailing space
    act:checkString(" Hello world ");

    // Check the above but with commas
    act:checkString("Hello, world");

    // Check that actspec accepts a single word with quotes
    act:checkString("oneword");

    // Check that actspec accepts a single word with quotes and leading and trailing space
    act:checkString(" oneword ");

    // Check that actspec accepts words without quotes
    act:checkString(something);

    // Check that string argument that starts with a variable (? or !) gets correctly replaced with variable value
    op:log("info", "?actor should be set to the actor variable value");

    // Check that opspec
    //   - accepts strings literals between quotes
    //   - strips the quotes away
    //   - replaces variables by their value
    op:log("info", "Hello world !variable");

    // Check the above but with a leading and trailing space
    op:log("info", " Hello world !variable ");

    // Check the above but with commas
    op:log("info", "Hello, world, !variable");

    // Check that opspec accepts a single word with quotes
    op:log("info", "oneword");

    // Check that opspec accepts a single word with quotes and leading and trailing space
    op:log("info", " oneword ");

    // Check that opspec accepts words without quotes
    op:log("info", lets(try(a,predicate)));
}

() = checkString(java.lang.String ?argument) {
    op:log("info", "Here comes the string: ?argument.");
}

() = concatStringTest() {
    java.lang.String !arg1 = "one";
    java.lang.String !arg2 = "two";
    java.lang.String !arg3;
    !arg3 = op:invokeMethod(!arg1, "concat", !arg2);
    op:log("info", "Concat results (should be onetwo): !arg3");
    if (~op:equals(!arg3, "onetwo")) {
      op:log("error", "string concat doesn't equal onetwo. It's value is: !arg3");
      exit(FAIL);
    }
}

