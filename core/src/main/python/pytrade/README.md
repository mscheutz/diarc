# PyTRADE README

Disclaimer: This project is a **work in progress**, and everything is subject to change. Please feel free to ask questions or report bugs!

## What is PyTRADE?

PyTRADE is a Python wrapper for TRADE, a Java-based middleware used in [DIARC](https://github.com/mscheutz/diarc), a cognitive architecture.

With PyTRADE, developers can:

-   Use DIARC components in Python.
-   Integrate DIARC with Python-based tools, such as simulation environments or reinforcement learning libraries.

## File Structure

### `core/src/main/python`

This is where you’ll write your Python code. Start by creating a new [Python package](https://docs.python.org/3/tutorial/modules.html#packages). Refer to other projects in the repository for examples.

### `core/src/main/java/python`

This directory contains the Java code responsible for starting the Python process.

## Example: Using PyTRADE in Python

Follow these steps to implement and register a simple TRADE service in Python:

### 1. **Logging**

Start by setting up your logging so it's visible from a java console:

```python
import sys
import logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)
logging.info("This will show up in Java output")
```

### 2. **Imports**
Start by importing `pytrade`. The order of imports matter here, and importing `pytrade` first allows you to import java classes. Import anything else you may need for the project.

```python
import time
from pytrade.wrapper import TRADEWrapper
from ai.thinkingrobots.trade import TRADE
from jpype import JImplements, JOverride
from edu.tufts.hrilab.interfaces import DockingInterface
```

### 3. **Implement a Java Interface in Python**

Define a Python class that implements the `DockingInterface` Java interface. Use the `@JImplements` decorator to link the interface and `@JOverride` for its methods:

```python
@JImplements(DockingInterface)
class DockingComponent:
    @JOverride
    def dock(self, dockId):
        # Implementation for dock method
        print("Docking")

    @JOverride
    def undock(self):
        # Implementation for undock method
        print("Undocking")
```
### 4. **Initialize the TRADE Wrapper**

Create an instance of the `TRADEWrapper` to connect Python to TRADE:

```python
wrapper = TRADEWrapper()
```

### 5. **Register with TRADE**

Register your Python object (`DockingComponent`) with TRADE to make its methods available as TRADE services:

```python
docking_component = DockingComponent()
TRADE.registerAllServices(docking_component, "")
```


### 6. **Call a TRADE Method**

Use the `call_trade` method of the `TRADEWrapper` to call the service you just registered in TRADE:

```python
wrapper.call_trade("undock")
```

### 7. **Run the Script**

Put everything together and ensure the script runs without issues:

```python
import time
from pytrade.wrapper import TRADEWrapper
from ai.thinkingrobots.trade import TRADE
from jpype import JImplements, JOverride
from edu.tufts.hrilab.interfaces import DockingInterface

@JImplements(DockingInterface)
class DockingComponent:
    @JOverride
    def dock(self, dockId):
        # Implementation for dock method
        print("Docking")


    @JOverride
    def undock(self):
        # Implementation for undock method
        print("Undocking")
        
if __name__ == '__main__':
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    logging.info("This will show up in Java output")

    wrapper = TRADEWrapper()
    docking_component = DockingComponent()
    TRADE.registerAllServices(docking_component, "")
    time.sleep(1)  # Wait for services to register
    wrapper.call_trade("undock")
```

When you run the script, you should see the following:

1.  The log message: `INFO:root:This will show up in Java output`
2.  The output from the `undock` method: `Undocking`

## Developing with PyTRADE

### Connecting Your Python Code to TRADE

-   To use TRADE in Python, simply import `pytrade.wrapper`. The JVM will start automatically.
-   For full DIARC integration, instantiate the `TRADEWrapper` object:

    ```python
    wrapper = TRADEWrapper()  
    ```

-   Call TRADE methods through the wrapper, e.g., `wrapper.call_trade("your_method_name")`.

> **Note:** You can also import any DIARC or TRADE classes directly from their Java packages using JPype. Ensure your java imports come AFTER the `pytrade` import!

### Implementing an existing Java Interface in Python

You can implement existing Java interfaces in Python to advertise services through TRADE. For example, to control a robotic arm via Python:

1.  Create a new Python class based on an existing Java interface, e.g., `PythonArmComponent`.
2.  Decorate the class with `@JImplements([YourInterface])`:

    ```python
    @JImplements(ArmInterface)
    class PythonArmComponent:
    ```

    Ensure you import the relevant interface (e.g., `ArmInterface`).
3.  Implement all methods from the Java interface, using `@JOverride` for each:

    ```python
    @JOverride
    def moveTo(self, position):
        # Implementation here
    ```     
4.  Instantiate and register your Python class with TRADE:

    ```python
    your_object = PythonArmComponent()
    TRADE.registerAllServices(your_object, "")
    ```


Your methods should now be accessible through TRADE if they align with Java methods annotated as `@TRADEService`.

----------

### Creating a New Interface

If no existing Java interface fits your needs, you can define a new one in Java and use it with PyTRADE:

1.  Write your new Java interface and include the necessary methods.
2.  Annotate the methods with `@TRADEService`.
3.  Implement the new interface in Python, following the steps outlined in the previous section.

## Support

For additional help or questions, contact **[marlow.fawn@gmail.com](mailto:marlow.fawn@gmail.com)**.


