# PyTRADE README

Disclaimer: This project is a **work in progress**, and everything is subject to change. Please feel free to ask questions or report bugs!

## What is PyTRADE?

PyTRADE is a Python wrapper for TRADE, a Java-based middleware used in [DIARC](https://github.com/mscheutz/diarc), a cognitive architecture.

With PyTRADE, developers can:

-   Use DIARC components in Python.
-   Integrate DIARC with Python-based tools, such as simulation environments or reinforcement learning libraries.

## Developing with PyTRADE
This section briefly go over the basics needed to create a new Python script and integrate it with DIARC via PyTRADE. If you find it difficult to follow, please see the [example](./README.md#example--using-pytrade-in-python).

---
### File Structure

`core/src/main/python`: This is where you’ll write your Python code.

`core/src/main/python/pytrade`: This is where PyTRADE actually lives, as well as some python/java utilities.

`core/src/main/java/edu/tufts/hrilab/python`: This directory contains the Java code responsible for starting the Python process.

`config/src/main/java/edu/tufts/hrilab/config`: This is where DIARC launch files live.

---
### Initializing Your Python Project
1. Start by creating a new [Python package](https://docs.python.org/3/tutorial/modules.html#packages) in `core/src/main/python`. Make sure you have an `__init__.py`
> **Note:** If you're familiar with Python package management, feel free to make your python package anywhere.  
2. Create a new virtual environment, for example [venv](https://docs.python.org/3/library/venv.html). It is recommended that you also make a `requirements.txt`.
4. Create an entry point for your script, e.g. `main.py`.
5. Update your python path:
```bash
export PYTHONPATH="[path/to/diarc]/core/src/main/python:$PYTHONPATH"
```
6. Update your trade properties path:
```bash
export TRADE_PROPERTIES_PATH="[path/to/diarc]/core/src/main/python/pytrade/local_trade_hub.properties"
```
> **Note:** Consider adding these environment variables to your `~/.bashrc` so your paths stays updated.

### Using TRADE Services from Python
1. To use TRADE in Python, simply import `pytrade.wrapper`. The JVM will start automatically.
2. Instantiate the `TRADEWrapper` object:

```python
wrapper = TRADEWrapper()  
```

3. Call TRADE methods through the wrapper, e.g., `wrapper.call_trade("your_method_name")`.

> **Note:** You can also import any DIARC or TRADE classes directly from their Java packages using JPype. Ensure your java imports come AFTER the `pytrade` import!
---
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

---

### Creating a New Interface

If no existing Java interface fits your needs, you can define a new one in Java and use it with PyTRADE:

1.  Write your new Java interface and include the necessary methods.
2.  Annotate the methods with `@TRADEService`.
3.  Implement the new interface in Python, following the steps outlined in the previous section.
---
## [Example: Using PyTRADE in Python](example)

Follow these steps to implement and register a simple TRADE service in Python:
### 0. File Setup

In `core/src/main/python`, create a package with python file called `main.py` and a file called `__init__.py`. All following code will be in the former. 

### 1. **Logging**

Start by setting up your logging so it's visible from a java console:

```python
import time
import sys
import logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)
```

### 2. **Imports**
Start by importing `pytrade`. The order of imports matter here, and importing `pytrade` first allows you to import java classes. Import anything else you may need for the project.

```python
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
        logging.info(f"Docking: {dockId}")

    @JOverride
    def undock(self):
        logging.info("Undocking")
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
while True:
    wrapper.call_trade("undock")
    time.sleep(5)
```

### 7. **Run the Script**

Put everything together and ensure the script runs without issues:

```python
import time
import sys
import logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)

from pytrade.wrapper import TRADEWrapper
from ai.thinkingrobots.trade import TRADE
from jpype import JImplements, JOverride
from edu.tufts.hrilab.interfaces import DockingInterface

@JImplements(DockingInterface)
class DockingComponent:
    @JOverride
    def dock(self, dockId):
        logging.info(f"Docking: {dockId}")

    @JOverride
    def undock(self):
        logging.info("Undocking")

if __name__ == '__main__':


    wrapper = TRADEWrapper()
    docking_component = DockingComponent()
    TRADE.registerAllServices(docking_component, "")
    time.sleep(1)

    while True:
        wrapper.call_trade("undock")
        time.sleep(5)
```

When you run the script, you should see the following:

1.  The log message: `INFO:root:This will show up in Java output`
2.  The output from the `undock` method: `Undocking`

Take a breather! Your Python code is done. If you want to integrate your code into DIARC, see the next section. 

---
## Adding a Python Script to a DIARC Config
1. Update your `trade.properties`
2. Follow the regular instructions for [creating a DIARC config](https://github.com/mscheutz/diarc/wiki/Writing-a-New-DIARC-Config).
3. Add your Python file as follows:
```java
String file = "examples.minimal_example";
PythonWrapper wrapper = new PythonWrapper(file, true);
wrapper.start();
try {
    Thread.sleep(2000);
} catch (InterruptedException e) {
    throw new RuntimeException(e);
}
```
> **Note:** Your file should follow **Python package notation**. For most simple use cases, this will just be the directory that your `__init__.py`, followed by your python file. No `.py` extension!

4. If you [set up your TRADE config](./README.md#initializing-your-trade-config), make a TRADE call by adding the following:

```java
try {
    TRADE.getAvailableService(new TRADEServiceConstraints().name("dock")).call(void.class, Factory.createSymbol(""));
} catch (TRADEException e) {
    throw new RuntimeException(e);
}
```

5. Launch your file as outlined in the DIARC config tutorial.
```bash
./gradlew launch -Pmain=edu.tufts.hrilab.config.python.PythonTesting
```

## Support

For additional help or questions, contact **[marlow.fawn@gmail.com](mailto:marlow.fawn@gmail.com)**.
