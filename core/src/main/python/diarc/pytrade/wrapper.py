import logging
import sys
import warnings
import os
import argparse
from typing import List, Optional

import jpype.imports
from jpype import JObject

from diarc.pytrade.java_util import to_java_class, convert_to_java_object

logging.basicConfig(stream=sys.stdout, level=logging.INFO)
logging.info("Starting PyTRADE")

try:
    # Parse any args passed in from the main command. We only handle "classpath" here, for now.
    parser = argparse.ArgumentParser(description="Setup the JVM classpath")
    parser.add_argument('--classpath')
    args, unknown = parser.parse_known_args()

    sys.argv = [sys.argv[0]] + unknown # Remove pytrade args from argv

    jpype.startJVM(classpath=[args.classpath])

    # jpype_cp = jpype.java.lang.System.getProperty('java.class.path')
    # logging.info("jpype_cp: {}".format(jpype_cp))

    from ai.thinkingrobots.trade import TRADE, TRADEServiceConstraints, TRADEServiceInfo, TRADEException

except Exception as e:
    warnings.warn("Failure starting JVM.")
    print(e)
else:
    print("JVM started. You can now start using DIARC through TRADE.")


class TRADEWrapper:

    def call_trade(self, name: str, groups: Optional[List[str]] = None, *args) -> JObject:
        if groups is None:
            groups = []
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
            raise (ex)

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


def cleanup():
    """
    Called when the wrapper is closed. For use in cleaning up resources.
    :return:
    """
    logging.info("Exiting wrapper")


import atexit

atexit.register(cleanup)
