for (!i=0; !i lt 10; !i ++) {
  op:log("info", "One-by-one: !i");
}

for (!i=0; !i lt 10; !i +=2) {
  op:log("info", "Two-by-two: !i");
}

for (!i=100; !i geq 0; !i /=2) {
  op:log("info", "Half: !i");
}