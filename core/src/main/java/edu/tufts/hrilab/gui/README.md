# GUI

## Overview

Navigate to `com/config/peruserfiles/[yourname]/yourconfig`.

## Launching the GUI

To add a GUI to your configuration:

1. In your configuration file located at `peruserfiles//[yourname]/yourconfig`, add the following imports:

    ```java
    import edu.tufts.hrilab.gui.DemoApplication;
    import org.springframework.boot.SpringApplication;
    ```

2. In the `main` method of your configuration class, before `runConfiguration()`, add:

    ```java
    SpringApplication.run(DemoApplication.class, args);
    ```

3. Compile the project using:

    ```bash
    ant gui
    ```

4. Launch your configuration with:

    ```bash
    ant launch -Dmain=edu.tufts.hrilab.config.peruserfiles.[yourname].[yourconfig]
    ```

   For example:

    ```bash
    ant launch -Dmain=edu.tufts.hrilab.config.peruserfiles.mfawn.GuiConfig
    ```

## Accessing Services

Once the GUI is launched, you can access all TRADE services grouped by their components in the launch container via your browser at `http://localhost:8080/services`.

## Example Curl Commands to Call Services

To call the `getCurrFloor()` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=getCurrFloor" \
     -H "Content-Type: application/json"
```

To call the `getNearestElevatorDoors()` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=getNearestElevatorDoors" \
-H "Content-Type: application/json"
```

To call the `getElevatorEnterPosePath(edu.tufts.hrilab.fol.Symbol,double)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=getElevatorEnterPosePath" \
-H "Content-Type: application/json" \
-d '["location_0:location", 0.5]'
```

To call the `markDoor(edu.tufts.hrilab.fol.Symbol,boolean)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=markDoor" \
-H "Content-Type: application/json" \
-d '["location_0:location", true]'
```

To call the `setCurrentFloor(int)` service with a parameter:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=setCurrentFloor" \
-H "Content-Type: application/json" \
-d '[1]'
```

TODO: direct

     [java] 2024-04-10T16:22:12.074-04:00  INFO 283047 --- [nio-8080-exec-7] e.tufts.hrilab.gui.TradeServiceTracker   : Before service call... Args: setCurrentFloor
     [java] 2024-04-10T16:22:12.074-04:00 ERROR 283047 --- [nio-8080-exec-7] e.t.hrilab.movebase.map.MapComponent     : No floor plan for floor exists: 4
     [java] 2024-04-10T16:22:12.074-04:00  INFO 283047 --- [nio-8080-exec-7] e.tufts.hrilab.gui.TradeServiceTracker   : After service call... Args: setCurrentFloor

to frontend.

To call the `getDoorEdges(edu.tufts.hrilab.fol.Symbol)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=getDoorEdges" \
-H "Content-Type: application/json" \
-d '["location_0:location"]'
```

To call the `createReferences(java.util.List)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=createReferences" \
-H "Content-Type: application/json" \
-d '["X:location", "Y:location"]'
```

To call the `getLocalReference(edu.tufts.hrilab.fol.Symbol)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=getLocalReference" \
-H "Content-Type: application/json" \
-d '["X:location"]'
```

To call the `getPath(javax.vecmath.Matrix4d,edu.tufts.hrilab.fol.Symbol,double,boolean)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=getPath" \
-H "Content-Type: application/json" \
-d '[[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], "location_0:location", 0.01, true]'
```

Currently three possible results may occur:

    Reference is not known: Y:dest
    Target location must be in a room
    passing "string" to 'dest' results in the program running indefinitely

## TODO:

1. updateRobotPose -> getPoseGlobalQuat

2. checkAt -> term

3. convertToType(edu.tufts.hrilab.fol.Symbol,java.lang.Class,java.util.List)
   - currently only supports single arguments and arguments as lists. This functionality needs to be expanded.
   - The current list assumes a list of variables. This assumption needs to be adjusted.

## Known issue:

multiple goToLocation services with different arguments

# Map GUI

For component specific GUI,

first `ant gui` to build the Spring Boot application,

and then `ant` to build the components.

## Documentation for Map GUI

As part of our ongoing efforts to simplify our application's architecture, we have made significant changes to how the `MapComponent` is integrated and managed within our Spring application. Previously, a service layer (`MapDataService`) was responsible for managing map data loading and access. This layer has been removed to streamline operations and reduce complexity.

### Key Changes:
- **`MapComponent` Integration**: `MapComponent` is now fully managed as a Spring bean. This integration allows direct access to map data throughout the application, ensuring that all components that depend on map data can access the most current and synchronized data directly from `MapComponent`.