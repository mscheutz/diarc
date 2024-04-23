if (op:lt(?var, 10)) {
  op:log("info", "?var is lower than 10");
} elseif (op:leq(?var, 100)) {
  op:log("info", "?var is lower or equals to 100");
} else {
  op:log("info", "?var is greater than 100");
}