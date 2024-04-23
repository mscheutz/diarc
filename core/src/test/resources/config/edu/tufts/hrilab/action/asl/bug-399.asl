() = someScript() {
    op:log("info", "Some script!");
}

() = loop() {
    int !i = 0;
    while (op:lt(!i, 5)) {
        op:log("info", "i: !i");
        act:someScript();
        !i = op:++(!i);
    }
}

