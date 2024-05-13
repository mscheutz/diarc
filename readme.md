# The Distributed Interactive Affect Reflection Cognition (DIARC) Architecture

**Alpha release (updates will come over the next few weeks)**

# Introduction

DIARC is a component-based cognitive robotic architecture with deeply
integrated introspection and natural language facilities (e.g., see
[Scheutz et
al. 2019](https://hrilab.tufts.edu/publications/scheutzetal2018diarcchapter/)).
Different from classical cognitive architectures which are monolithic,
DIARC is intrinsically polylithic, i.e., any of its componens which
operate in parallel can be distributed over a set of heterogeneous
computing platforms with different operating systems.  This allows for
flexible configurations of DIARC that can meet computational and
real-time constraints.

There are at least three different ways to use DIARC that will require
increasingly deeper familiarity with the architecture and its
operation: (1) DIARC as the control architecture for applications of
virtual and physical agents, (2) DIARC as a framework for
component-based algorithm development, and (3) DIARC as a platform for
developing cognitive architectures.

## DIARC for applications

Different from other cognitive architectures which require different
"models" to be loaded, DIARC works "out-of-the-box" for a varying set
of tasks depending on the robot or virtual agent it controls.  For
example, DIARC can be used for task-based interactions where humans
instruct a DIARC-operated robot to perform actions like going to
particular locations or manipulating objects in a task environment.
These basic capabilities can be used for demonstrations but also for
human-robot interaction experiments.  In some cases additional
knowledge will have to be supplied to different DIARC components
(e.g., the vision system, the parsers, etc.) for DIARC to be able to
perform desired tasks.

## DIARC for algorithm development

DIARC can also serve as an architecture framework for algorithm
developers who work on specific component algorithms (e.g., planning,
reasoning, etc.) and would like to evaluate their algorithms in the
context of a complete agent.  I.e., somebody working on task planning
will be able to integrate a novel planning very easily due to
standardized interfaces and experiment with the planner in
human-machine teams without having to worry about the natural language
interactions, or perceptual processing.  Similarly, somebody
interested in active perception will be able to work on novel
perceptual algorithms and utilize the goal and action management in
DARC to navigate through the environment based on the needs of the
perceptual algorithms.

## DIARC for cognitive architecture research

Finally, for those interested in architecture research DIARC provides
a flexible set of configurable components that can be connected in
different ways.  The TRADE middleware in which DIARC is implemented
allows for deep introspection that can form the foundation for
important research directions on meta-cognition and self-awareness,
resilience and long-term autonomy, shared mental model and team
cognition, open-world AI and many others.  DIARC can also serve as a
framework connecting existing cognitive architecture to agent
platforms (e.g., see [Scheutz, Harris and Schermerhorn
2013](https://hrilab.tufts.edu/publications/scheutzetal13acs/)).

# Building & Running: how to get and run DIARC

DIARC is mostly Java-based and thus runs on any platform for which a
Java Virtual Machine exists, including Linux and Android, MacOS, and
Windows.  In fact, DIARC can run in mixed-OS environments, form
handhelds, to the cloud.  It is easy to install by simply cloning this
repository and then "building" it.

## Gradle
   
DIARC uses the [Grade Build Tool](https://gradle.org/) for building
and running all architecture configurations.  It already contains a
"gradle wrapper", so there is no need to install gradle separately.
Simply use

  **./gradlew** (instead of **gradle**)

to run all the gradle commands.

For algorithm and architecture development in DIARC we recommend using
IntelliJ IDEA which supports a deep integration with Gradle (many
gradle tasks can be run through the [IntelliJ
GUI](https://www.jetbrains.com/help/idea/gradle.html)).

**NOTE: To override the default gradle properties set in the root DIARC_HOME directory, gradle.properties file,
create your own local gradle.properties file (usually in ~/.gradle/gradle.properties).**

## Directory Structure

The DIARC repository is divided into four gradle subprojects.

<details>
<summary>CORE subproject</summary>

Contains the core set of DIARC components, such as task management and planning, language processing
and generation, and various devices and non-ROS robots.

```
core
└── src
    ├── main
    │   ├── java
    │   │   └── edu
    │   │       └── tufts
    │   │           └── hrilab : core DIARC components and Java classes
    │   ├── resources
    │   │   └── config
    │   │       └── edu
    │   │           └── tufts
    │   │               └── hrilab : configuration files for DIARC components in core
    │   └── scala
    │       └── edu
    │           └── tufts
    │               └── hrilab : scala code for ReferenceResolutionComponent
    ├── mock
    │   └── java
    │       └── edu
    │           └── tufts
    │               └── hrilab : "mock" DIARC components (can be used in place of real robot components)
    └── test
        ├── java
        │   └── edu
        │       └── tufts
        │           └── hrilab : unit tests for core DIARC components and Java classes in core 
        └── resources
            └── config
                └── edu
                    └── tufts
                        └── hrilab : configuration files for unit tests
```

</details>

<details>
<summary>CONFIG subproject</summary>

Contains all DIARC configurations, which are collections of DIARC components. Also includes build logic for running DIARC components and configs.

```
config
└── src
    ├── main
    │   ├── java
    │   │   └── edu
    │   │       └── tufts
    │   │           └── hrilab : DIARC configurations
    │   └── resources
    │       └── default : default settings for logging, trade properties, etc
    └── test
        ├── java
        │   └── edu
        │       └── tufts
        │           └── hrilab : integration tests for DIARC configurations
        └── resources
            └── controls
                └── edu
                    └── tufts
                        └── hrilab : configuration files for integration tests
```
</details>


<details>
<summary>DIARCROS subproject</summary>

DIARC provides support for using ROS with DIARC as well as several DIARC components for interfacing with
common ROS packages and robots, but is disabled by default. This code lives in the diarcRos subpropject.
See the [DIARC ROS wiki page](https://github.com/mscheutz/diarc/wiki/DIARC-ROS) for more information on using
the diarcRos subproject.

```
diarcRos
└── src
    ├── main
    │   ├── java
    │   │   └── edu
    │   │       └── tufts
    │   │           └── hrilab : DIARC component interfaces that do not depend on ROS but are needed
    │   │               │        by "mock" and non-mock diarcRos oocomponents
    │   │               ├── fetch : DIARC FetchInterface
    │   │               ├── pr2 : DIARC PR2Interface
    │   │               ├── ...
    │   │               └──
    │   └── resources
    │       └── config
    │           └── edu
    │               └── tufts
    │                   └── hrilab : configuration files for ROS dependent DIARC components
    ├── mock
    │   └── java
    │       └── edu
    │           └── tufts
    │               └── hrilab : "mock" DIARC components that are available without ROS    
    └── ros
        └── java
            └── edu
                └── tufts
                    └── hrilab : DIARC components and Java classes that depend on ROS
                        ├── diarcros : rosjava proxy nodes
                        │   ├── fetch
                        │   ├── ...
                        │   └──
                        ├── fetch : DIARC FetchComponent
                        ├── pr2 : DIARC PR2Component
                        ├── ...
                        └──
                        └──
```
</details>

<details>
<summary>VISION subproject</summary>

The vision subproject contains the DIARC VisionComponent which can be used for common computer vision tasks like
object detection and tracking. The VisionComponent relies on several native dependencies that need to be manually installed,
so it is disabled by default. See the [vision wiki page](https://github.com/mscheutz/diarc/wiki/Vision-Component) for
more information on using the vision subproject.

```
vision
└── src
    ├── main
    │   ├── cpp
    │   │   ├── capture : "camera" capture classes
    │   │   ├── detector : object detector classes
    │   │   ├── display : image and point cloud display classes
    │   │   ├── imgproc : image processor and validator classes
    │   │   ├── stm : short term memory classes
    │   │   ├── tracker : object tracker classes
    │   │   ├── visionproc : core vision processor classes
    │   │   │  ...
    │   │   └── 
    │   ├── java
    │   │   └── edu
    │   │       └── tufts
    │   │           └── hrilab : VisionComponent and other Java-side classes
    │   └── resources
    │       └── config
    │           └── edu
    │               └── tufts
    │                   └── hrilab : configuration files for VisionComponent
    ├── mock
    │   └── java
    │       └── edu
    │           └── tufts
    │               └── hrilab : "mock" vision component and helper classes
    └── test
        └── java
            └── edu
                └── tufts
                    └── hrilab : vision specific unit tests
```
</details>

## Javadocs

DIARC javadocs can be found here: https://hrilab.tufts.edu/javadoc

TRADE (middleware used by DIARC) javadocs can be found here: https://thinkingrobots.ai/javadoc

## Compiling the Code

There is generally no need to compile DIARC code before running it, but it can sometimes be useful during
code development. The following command will compile all the "enabled" code (some subprojects like vision and diarcRos
are disabled be default. See below for more info on those subprojects).

`./gradlew assemble`

NOTE: using the common `./gradlew build` command will also run _all_ the DIARC unit and integration tests,
which will take 10+ minutes so you should prefer using the _assemble_ gradle task.

## Running a Single DIARC Component 

To run a single DIARC Component use the launch task, specifying the full class
path of the component with the `-Pmain=` argument.

`./gradlew launch -Pmain=edu.tufts.hrilab.action.GoalManagerImpl`

To pass arguments to the component, use the `--args=` argument.

`./gradlew launch -Pmain=edu.tufts.hrilab.action.GoalManagerImpl --args="-editor"`

To run the component in debug mode (where you attach your IDE to the running process)
use the `--debug-jvm` argument.

`./gradlew launch -Pmain=edu.tufts.hrilab.action.GoalManagerImpl --args="-editor" --debug-jvm`

## Running a DIARC Configuration 

To run a DiarcConfiguration (a collection of DIARC components) which is located in the
config subproject you can run.

`./gradlew launch -Pmain=edu.tufts.hrilab.config.nao.TwoNaoDemo`

where the value of -Pmain is the class name of the config you want to run. See
[this page](https://github.com/mscheutz/diarc/wiki/two-nao-demo) for a more in-depth look at this demo configuration.

## Metric-FF

**NOTE:** without setting up Metric-FF on your local machine, two of the DIARC unit tests will fail.

The planning system can be configured to use the Metric-FF PDDL planner. To enable use of Metric-FF
1. Download Metric-FF version 2.1 sources from [its website](https://fai.cs.uni-saarland.de/hoffmann/metric-ff.html)
2. Configure build requirements
   1. make, gcc (included in `build-essential` if on Ubuntu)
   2. flex
   3. bison
3. Extract the archive and compile with `make`
4. update `~/.gradle/gradle.properties` to include the property `diarc.planner.ff= `  with the path to the artifact produced in step 3


## Demonstration Videos

Several examples of DIARC being used for human-robot interactions can be seen [here](https://www.youtube.com/@HRILaboratory).
