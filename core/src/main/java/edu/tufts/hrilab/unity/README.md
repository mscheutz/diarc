# Unity <-> DIARC Components

-----

## UnityComponent

This component establishes the socket connection between Unity and DIARC.
It exposes TRADE services but **they should be avoided** in place of using other services registered by UnityDIARCAgent components.

Example configuration:

```java
DiarcComponent.createInstance(com.unity.UnityComponent.class, "-unityip 192.168.0.212 -unityport 1755");
```

TRADE services:

* **(AVOID)** unitySendMessage - Sends message to Unity
* **(AVOID)** unityGetResponse - Sends message to Unity and awaits a response
* sayText (Symbol agent, String text)

#### sayText (Symbol agent, String text)

Send a message to be synthesized into speech by agent on Unity side.
`TRADE.callAll()` with the agent Symbol as first argument plus the text containing the message.

-----

## UnityAgent

This is the base class for all Unity agents.
Specific Implementations, such as UnitySpaceStation and UnityPR2 provide additional functionalities.

Example configuration:

```java
DiarcComponent.createInstance(com.unity.UnityAgent.class, "-agent rover");
```

TRADE services:

* **(AVOID)** unityReceiveMessage - Accepts incoming messages and routes to local methods, used by UnityComponent 

-----

## UnitySpaceStation

This extension of the UnityAgent class representing the 

Example configuration: 

```java
DiarcComponent.createInstance(com.unity.UnitySpaceStation.class, "-agent spacestation");
```

TRADE services:

* getSpaceStationHealth -> float health
* getSpaceStationTubesDamaged (Symbol area *optional*) -> List<Symbol> tubes
* getSpaceStationTubesBroken (Symbol area *optional*) -> List<Symbol> tubes
* getSpaceStationTubesOff (Symbol area *optional*) -> List<Symbol> tubes
* getSpaceStationTubeHealth (Symbol tube) -> float health
* getSpaceStationTubeDamaged (Symbol tube) -> boolean isDamaged
* getSpaceStationTubeBroken (Symbol tube) -> boolean isBroken
* getSpaceStationTubeOff (Symbol tube) -> boolean isOff

#### getSpaceStationHealth -> float health

Returns the total overall health of the space station in a float ranging `[0:100]`.

#### getSpaceStationTubesDamaged (Symbol area *optional*) -> List<Symbol> tubes

Returns a List of tubes that are currently damaged.
Optionally accepts a Symbol representing the area to limit the search to, "alpha", "beta" or "gamma".

#### getSpaceStationTubesBroken (Symbol area *optional*) -> List<Symbol> tubes

Returns a List of tubes that are currently broken.
Optionally accepts a Symbol representing the area to limit the search to, "alpha", "beta" or "gamma".

#### getSpaceStationTubesOff (Symbol area *optional*) -> List<Symbol> tubes

Returns a List of tubes that are currently off.
Optionally accepts a Symbol representing the area to limit the search to, "alpha", "beta" or "gamma".

#### getSpaceStationTubeHealth (Symbol tube) -> float health

Returns a float representing the health of an individual tube ranging `[0:100]`.
Identified via the Symbol representing the tube name.

#### getSpaceStationTubeDamaged (Symbol tube) -> boolean isDamaged

Returns a boolean that is `true` when the requested tube is damaged.
Identified via the Symbol representing the tube name.

#### getSpaceStationTubeBroken (Symbol tube) -> boolean isBroken

Returns a boolean that is `true` when the requested tube is broken.
Identified via the Symbol representing the tube name.

#### getSpaceStationTubeOff (Symbol tube) -> boolean isOff

Returns a boolean that is `true` when the requested tube is off.
Identified via the Symbol representing the tube name.

-----

## UnityPR2

Example configuration: 

```java
DiarcComponent.createInstance(com.unity.UnityPR2.class, "-agent robot1");
```

TRADE services:

* getPR2Area -> Symbol area
* startPR2Repair (String target)

#### getPR2Area -> Symbol area

Returns Symbol of area where PR2 believes it is in the simulation.
This should be replaced by a Belief-based action and not rely on the simulation to store this information.

#### startPR2Repair (String target)

Begins repairing a target object in the simulation.


