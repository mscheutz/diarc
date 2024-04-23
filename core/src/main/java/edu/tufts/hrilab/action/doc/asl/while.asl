while (op:lt(?i, 10)) {
  op:log("info", "i=?i");
  ?i = op:++(?i);
}