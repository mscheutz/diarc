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

To call the `externalUpdateReferenceNumber(int)` service with a parameter:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=externalUpdateReferenceNumber" \
     -H "Content-Type: application/json" \
     -d '[5]'
```

To call the `getDoorEdges(edu.tufts.hrilab.fol.Symbol)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=getDoorEdges" \
-H "Content-Type: application/json" \
-d '["location_0:location"]'
```

To call the `markDoor(edu.tufts.hrilab.fol.Symbol,boolean)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=markDoor" \
-H "Content-Type: application/json" \
-d '["location_0:location", true]'
```

To call the `createReferences(java.util.List)` service:

```bash
curl -X POST "http://localhost:8080/invoke-service?serviceName=createReferences" \
-H "Content-Type: application/json" \
-d '["X:location", "Y:location"]'
```