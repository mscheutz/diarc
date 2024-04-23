() = asyncTest() {
  java.lang.Long !asyncId;
  edu.tufts.hrilab.action.ActionStatus !actionStatus;

  !asyncId = async {
    op:sleep(5000);
    op:log("info", "first step in async");
    op:log("info", "second step async");
  }
  op:log("info", "before join. async id: !asyncId");

  join(!asyncId, 1000);
  op:log("info", "after join with timeout. async id: !asyncId status: !actionStatus");

  !actionStatus = join(!asyncId, 1000);
  op:log("info", "after join with timeout. async id: !asyncId status: !actionStatus");

  !actionStatus = join(!asyncId);
  op:log("info", "after join. async id: !asyncId status: !actionStatus");
}

() = asyncTest2() {
  java.lang.Long !asyncId;
  edu.tufts.hrilab.action.ActionStatus !actionStatus;

  !asyncId = async {
    async {
      op:sleep(5000);
    }
    async {
      op:log("info", "first async");
    }
    async {
      op:log("info", "second async");
    }
  }
  op:log("info", "before join. async id: !asyncId");

  !actionStatus = join(!asyncId, 3000);
  op:log("info", "after join with timeout. async id: !asyncId status: !actionStatus");

  !actionStatus = join(!asyncId);
  op:log("info", "after join. async id: !asyncId status: !actionStatus");
}