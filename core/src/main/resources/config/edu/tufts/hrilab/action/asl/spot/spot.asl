() = dock() {
   op:log("info", "Attempting to dock...");
   try {
     act:dock(520);
   } catch () {
     act:dock(521);
   }
}