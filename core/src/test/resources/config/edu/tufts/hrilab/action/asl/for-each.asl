() = forEachTest() {
    java.util.ArrayList !list;
    !list = op:newArrayList("java.lang.Integer");
    for (!i=0; !i lt 10; !i ++) {
        op:add(!list, !i);
    }
    for (!i=10; !i gt 0; !i -=2) {
        op:add(!list, !i);
    }
    for (!i=1; !i lt 10; !i *=3) {
        op:add(!list, !i);
    }
    for (!i=10; !i gt 0; !i /=3) {
        op:add(!list, !i);
    }
    foreach(!elem : !list) {
        op:log("info", "Element: !elem");
    }
}

