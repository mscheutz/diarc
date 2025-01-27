# PyTRADE README

Welcome! This project is a work in progress, and everything is subject to change. Your feedback is invaluable—feel free to report questions or bugs!

---

## What is PyTRADE?

PyTRADE is a Python wrapper for TRADE, a Java-based middleware used in [DIARC](https://github.com/mscheutz/diarc), a cognitive architecture.

With PyTRADE, developers can:
- Use DIARC components in Python.
- Integrate DIARC with Python-based tools, such as simulation environments or reinforcement learning libraries.

---

## File Structure

### `core/src/main/python`
This is where you’ll write your Python code. Start by creating a new [Python package](https://docs.python.org/3/tutorial/modules.html#packages). Refer to other projects in the repository for examples.

### `core/src/main/java/python`
This directory contains the Java code responsible for starting the Python process.

---

## Getting Started with PyTRADE

### Example Script
Here’s a basic example of using PyTRADE to implement and register a TRADE service in Python:

```python
import time
import sys
import logging
from pytrade.wrapper import TRADEWrapper

from ai.thinkingrobots.trade import TRADE
from jpype import JImplements, JOverride
from edu.tufts.hrilab.interfaces import DockingInterface

@JImplements(DockingInterface)
class DummyWrapper:
    @JOverride
    def dock(self, dockId):
        pass

    @JOverride
    def undock(self):
        print("Undocking")

if __name__ == '__main__':
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    logging.info("This will show up in Java output")

    wrapper = TRADEWrapper()
    dummy_object = DummyWrapper()
    TRADE.registerAllServices(dummy_object, "")
    time.sleep(1)
    wrapper.call_trade("undock")
```

---

## Using PyTRADE

### 1. Connecting Your Python Code to TRADE
- To use TRADE in Python, simply import `pytrade.wrapper`. The JVM will start automatically.
- For full DIARC integration, instantiate the `TRADEWrapper` object:
  ```python
  wrapper = TRADEWrapper()
  ```
- Call TRADE methods through the wrapper, e.g., `wrapper.call_trade("your_method_name")`.

> Note: You can also import any DIARC or TRADE classes directly from their Java packages using JPype. Ensure your imports follow the correct order!

---

### 2. Implementing a Java Interface in Python
You can implement existing Java interfaces in Python to advertise services through TRADE. For example, to control a robotic arm via Python:

1. Create a new Python class, e.g., `PythonArmComponent`.
2. Decorate the class with `@JImplements([YourInterface])`:
   ```python
   @JImplements(ArmInterface)
   class PythonArmComponent:
   ```
   Ensure you import the relevant interface (e.g., `ArmInterface`).
3. Implement all methods from the Java interface, using `@JOverride` for each:
   ```python
   @JOverride
   def moveTo(self, position):
       # Implementation here
   ```
4. Register your Python class with TRADE:
   ```
   TRADE.registerAllServices(your_object, "")
   ```

Your methods should now be accessible through TRADE if they align with Java methods annotated as `@TRADEService`.

---

### 3. Creating a New Interface
If no existing Java interface fits your needs, you can define a new one in Java and use it with PyTRADE:

1. Write your new Java interface and include the necessary methods.
2. Annotate the methods with `@TRADEService`.
3. Implement the new interface in Python, following the steps outlined in the previous section.

---

## Support
For additional help or questions, contact marlow.fawn@gmail.com.
