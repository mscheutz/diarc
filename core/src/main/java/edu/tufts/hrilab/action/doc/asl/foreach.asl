!list = op:newArrayList("java.lang.Integer");

for (!i=0; !i lt 10; !i ++) {
  op:add(!list, !i);
}

foreach(!elem : !list) {
  op:log("info", "Element: !elem");
}