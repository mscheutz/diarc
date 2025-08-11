import warnings
import logging
import sys

import numpy as np
from jpype.types import *

# Todo: Can we handle both JBoolean and java.lang.Boolean?
# Todo: Why does TRADE break with bool?
PY_TO_JAVA_MAP = {
    str: JString,
    bool: "java.lang.Boolean",
    int: JInt,
    float: JFloat,
    list: "java.util.List",
    tuple: "java.util.List",
    dict: "java.util.Map",
    np.float64: JDouble
}

logging.basicConfig(stream=sys.stdout, level=logging.INFO)

def to_java_object(arg):
    """
    Turns a python object into its java equivalent.
    :param arg: A python object
    :return: A corresponding java object
    """
    # Todo: Handle interfaces
    constructor = to_java_class(arg)
    return constructor(arg)

def to_java_object_from_class(arg, clazz: str):
    """
    Turns a python object into its java equivalent, from a specific Java classpath.
    :param clazz:
    :param arg: A python object
    :return: A corresponding java object
    """
    return JClass(clazz)(arg)

def to_java_class(arg):
    """
    Gets the class of a given python object.
    :param arg: A python object that we want to get the class of
    :return: A corresponding java class
    """
    if isinstance(arg, JClass):
        return arg
    elif isinstance(type(arg), JClass):
        return type(arg)
    clazz = PY_TO_JAVA_MAP.get(type(arg))
    if clazz is None:
        warnings.warn(f"Unsupported java type {type(arg)}. Using generic object.")
        return JObject
    return JClass(clazz)


def convert_to_java_object(obj):
    """
    Deeply turns a python object into its java equivalent
    :param obj: A python object we want to turn into a java object
    :return:
    """
    if isinstance(obj, (tuple, np.ndarray)):  # Convert python lists to Arraylists
        return JClass("java.util.ArrayList")([convert_to_java_object(item) for item in list(obj)])
    elif isinstance(obj, list):  # Convert python lists to Arraylists
        return JClass("java.util.ArrayList")([convert_to_java_object(item) for item in obj])
    elif isinstance(obj, dict):  # Convert python lists to Arraylists
        java_map = JClass('java.util.HashMap')()
        [java_map.put(convert_to_java_object(key), convert_to_java_object(value)) for key, value in obj.items()]
        return java_map
    else:  # Convert python objects to Java objects
        return to_java_object(obj)
