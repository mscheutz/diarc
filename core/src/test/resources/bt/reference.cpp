// Differential-test driver for BehaviorTree.CPP 4.10.0.
// Build against the upstream library; do not substitute a Java reference evaluator.
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
std::mutex trace_mutex;
std::vector<std::string> events;
std::map<std::string, bool> checks{{"battery", true}, {"urgent", false}, {"bad", false}};
std::map<std::string, bool> finished;

void record(const std::string& event) {
  std::lock_guard<std::mutex> lock(trace_mutex);
  events.push_back(event);
}

class Work : public BT::StatefulActionNode {
 public:
  Work(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("label")}; }

  BT::NodeStatus onStart() override {
    label_ = getInput<std::string>("label").value();
    record("start:" + label_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    return finished[label_] ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void onHalted() override { record("halt:" + label_); }

 private:
  std::string label_;
};
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: reference scenario input.xml\n";
    return 2;
  }
  try {
    BT::BehaviorTreeFactory factory;
    factory.registerScriptingEnum("INSPECT", 2);
    factory.registerNodeType<Work>("Work");
    factory.registerSimpleAction("Record", [](BT::TreeNode& node) {
      record("record:" + node.getInput<std::string>("label").value());
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("label")});
    factory.registerSimpleCondition("Check", [](BT::TreeNode& node) {
      return checks[node.getInput<std::string>("label").value()]
          ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }, {BT::InputPort<std::string>("label")});
    int attempts = 0;
    factory.registerSimpleCondition("Attempt", [&attempts](BT::TreeNode&) {
      return ++attempts >= 3 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
    factory.registerSimpleAction("Produce", [](BT::TreeNode& node) {
      node.setOutput("value", std::string("kitchen"));
      return BT::NodeStatus::SUCCESS;
    }, {BT::OutputPort<std::string>("value")});

    const std::string scenario = argv[1];
    auto blackboard = BT::Blackboard::create();
    if (scenario == "dynamic-script") {
      blackboard->createEntry("source", BT::TypeInfo::Create<std::string>());
      blackboard->set("source", std::string("x := 4"));
    }
    auto tree = factory.createTreeFromFile(argv[2], blackboard);
    auto status = tree.tickExactlyOnce();
    if (scenario == "sequence") {
      checks["battery"] = false;
      finished["move"] = true;
    } else if (scenario == "reactive-sequence") {
      checks["battery"] = false;
    } else if (scenario == "reactive-fallback" || scenario == "while-do-else") {
      checks["urgent"] = true;
    } else if (scenario == "parallel") {
      finished["one"] = true;
    } else if (scenario == "try-catch-running") {
      finished["cleanup"] = true;
    }
    for (int tick = 0; status == BT::NodeStatus::RUNNING && tick < 2000; ++tick) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      status = tree.tickExactlyOnce();
    }
    tree.haltTree();
    {
      std::lock_guard<std::mutex> lock(trace_mutex);
      for (const auto& event : events) {
        std::cout << event << '\n';
      }
    }
    std::cout << "root:" << BT::toStr(status) << '\n';
    return status == BT::NodeStatus::RUNNING ? 1 : 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
