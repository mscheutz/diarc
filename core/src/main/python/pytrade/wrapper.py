import sys
import warnings
import threading
import os
import time

import jpype.imports
from jpype import JObject

from pytrade.java_util import to_java_class, convert_to_java_object

# Todo: Should we still allow user specified jars?
try:
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    jars_dir = os.path.join(current_file_dir, 'jars/*')
    jpype.startJVM(classpath=[jars_dir, sys.argv[1]])
    from ai.thinkingrobots.trade import TRADE, TRADEServiceConstraints, TRADEServiceInfo, TRADEException

except Exception as e:
    warnings.warn("Failure starting JVM. Did you publish DIARC to your local maven?")
    print(e)
else:
    print("JVM started. You can now start using DIARC through TRADE.")


class TRADEWrapper:

    # Threading methods
    # def __init__(self):
    #     self.running = threading.Event()
    #     self.running.set()  # Allow the thread to run
    #
    # def _spin(self):
    #     while self.running.is_set():
    #         time.sleep(1)  # Simulate some work
    #
    # def spin(self):
    #     """
    #     Launches a background process that spins until the parent process exits
    #     :return:
    #     """
    #     self.thread = threading.Thread(target=self._spin, daemon=True)
    #     self.thread.start()
    #
    # def stop(self):
    #     """
    #     Manually stops the background process
    #     :return:
    #     """
    #     self.running.clear()  # Stop the loop
    #     self.thread.join()  # Wait for the thread to finish

    def call_trade(self, name: str, *args) -> JObject:
        return self.call_trade(name, [], args)

    def call_trade(self, name: str, groups: list[str], *args) -> JObject:
        """
        Calls a generic TRADE service. Name is the name of the service, argv is vararg arguments.

        :param name: Name of the TRADE service
        :param groups: TRADE Component groups for the service
        :param args: The TRADE service's varargs
        :return: The return value of the TRADE service, almost certainly java object
        """
        try:
            constraints = TRADEServiceConstraints()  # Initialize the TRADE constraints
            constraints.name(name)  # Add the service's name to the constraints

            # Get Java classes that correspond to the arguments
            class_array = [to_java_class(arg) for arg in args]
            constraints.argTypes(class_array)  # Add the argument types to the constraints
            if groups:
                constraints.inGroups(groups)

        # Todo: Better error handling
            service = TRADE.getAvailableService(constraints)  # Get the TRADE service
            parameters = convert_to_java_object(list(args))  # Convert the arguments to their Java versions
            return service.call(JObject, *parameters)  # Call the service
        except Exception as ex:
            print(f"Caught exception: {str(ex)}")
            raise(ex)

    def create_action(self, name: str, args, preconds, effects, executor: str) -> JObject:
        """
        Creates a new action in DIARC via TRADE.
        :param name: Name of the action
        :param args: Arguments of the action
        :param preconds: Preconditions of the action, in predicate form
        :param effects: Effects of the action, in predicate form
        :param executor: The body of the action (WIP. Currently calls "callPolicy([executor])")
        :return: A unique ID associated with this action. Might remove in favor of just using the name.
        """
        return self.call_trade("createAction", name, args, preconds, effects, executor)

    def create_observer(self, name: str, observes, detector):
        """
        Creates a DIARC observer (equivalent of "detector" for RL)
        :param name: Name of the observer action
        :param observes: Predicate to observe
        :param detector: The name of the Python TRADE service to call for this observation/detection
        :return:
        """
        warnings.warn("Not yet implemented.")

    def check_state(self, obs):
        """
        WIP, do not use.
        :param obs:
        :return:
        """
        warnings.warn("Not yet implemented.")
        return self.call_trade("checkState", obs)

    def parse_goal(self, goal: str):
        return [goal]
