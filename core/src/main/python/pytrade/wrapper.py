import sys
import warnings

import jpype.imports
import os

from jpype import JObject

from pytrade.java_util import to_java_class, convert_to_java_object

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
    def call_trade(self, name: str, *args) -> JObject:
        """
        Calls a generic TRADE service. Name is the name of the service, argv is vararg arguments.

        :param name: Name of the TRADE service
        :param args: The TRADE service's varargs
        :return: The return value of the TRADE service, almost certainly java object
        """
        constraints = TRADEServiceConstraints()  # Initialize the TRADE constraints
        constraints.name(name)  # Add the service's name to the constraints

        # Get Java classes that correspond to the arguments
        class_array = [to_java_class(arg) for arg in args]
        constraints.argTypes(class_array)  # Add the argument types to the constraints

        # Todo: Better error handling
        try:
            service = TRADE.getAvailableService(constraints)  # Get the TRADE service
            parameters = convert_to_java_object(list(args))  # Convert the arguments to their Java versions
            return service.call(JObject, *parameters)  # Call the service
        except TRADEException as ex:
            print(f"Caught TRADE exception: {str(ex)}")

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
