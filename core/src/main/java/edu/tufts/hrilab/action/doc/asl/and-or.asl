if (op:gt(?var, -10) && op:lt(?var, 10)) {
  op:log("info", "?var is between -10 and 10");
} elseif (op:lt(?var, -10) || op:gt(?var, 10)) {
  op:log("info", "?var is l.t. -10 or g.t. 10");
}