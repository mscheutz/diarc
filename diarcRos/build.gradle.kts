fun addRosSource(packageName: String, sources: List<String>, depends: List<String> = emptyList()) {

  sourceSets {
    create(packageName) {
      java {
        compileClasspath += sourceSets.main.get().compileClasspath + sourceSets.main.get().output
        runtimeClasspath += sourceSets.main.get().runtimeClasspath + sourceSets.main.get().output

        for (depend in depends) {
          compileClasspath += sourceSets.getByName(depend).compileClasspath + sourceSets.getByName(depend).output
          runtimeClasspath += sourceSets.getByName(depend).runtimeClasspath + sourceSets.getByName(depend).output
        }

        srcDir("src/main/java")
        include(sources)
        // Todo: This is caused by map not being a properly separated package.
        // Fix: Move ChangeMapComponent.java to new dir, have movebase not depend on map.
        if (project.hasProperty("diarc.rosVersion") && project.property("diarc.rosVersion").toString() == "indigo") {
          exclude(listOf(
              "edu/tufts/hrilab/diarcros/map/MapServer.java",
              "edu/tufts/hrilab/movebase/ChangeMapComponent.java",
          ))
        }
      }
    }
  }

  configurations.create(packageName + "Jar") {
    isCanBeConsumed = true
    isCanBeResolved = false

    extendsFrom(configurations[packageName + "Implementation"], configurations[packageName + "RuntimeOnly"])
  }

  val jarTask = tasks.register<Jar>(packageName + "JarTask") {
    for (depend in depends) {
      from(sourceSets.getByName(depend).output)
    }

    from(sourceSets.getByName(packageName).output)
    archiveBaseName = "diarc-diarcRos-$packageName"
    archiveVersion = properties["diarc.version"].toString()
    archiveExtension = "jar"

    manifest {
      attributes(
          "Implementation-Title" to archiveBaseName,
          "Implementation-Version" to archiveVersion,
      )
    }
  }

  artifacts.add(packageName + "Jar", jarTask)
}

plugins {
  `java-library`
  `maven-publish`
}

java {
  toolchain {
    // set the java version
    languageVersion.set(JavaLanguageVersion.of(17))
  }
}

tasks.named<Test>("test") {
  onlyIf {
    project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean()
  }
}

tasks.compileJava {
  onlyIf {
    project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean()
  }
//  options.sourcepath = files(listOf("src/main/java"))
  options.compilerArgs = listOf("-parameters")
}

tasks.named<Javadoc>("javadoc") {
  onlyIf {
    project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean()
  }
}

sourceSets {
  create("mock") {
    compileClasspath += sourceSets.main.get().compileClasspath + sourceSets.main.get().output
    runtimeClasspath += sourceSets.main.get().runtimeClasspath + sourceSets.main.get().output
  }
  main {
    java {
      srcDir("src/main/java")
      include(listOf(
          "edu/tufts/hrilab/diarcros/common/*",
          "edu/tufts/hrilab/manipulator/generic/*",
          "edu/tufts/hrilab/tf/**", //todo: This requires common, but common is not core?
          "edu/tufts/hrilab/diarcros/cmd_vel/*",
          "edu/tufts/hrilab/diarcros/gazebo/*",
          "edu/tufts/hrilab/diarcros/laserscan/*",
          "edu/tufts/hrilab/diarcros/tf/*",
          "edu/tufts/hrilab/diarcros/turtlesim/*",
          "edu/tufts/hrilab/diarcros/util/*",
          "edu/tufts/hrilab/diarcros/uvc/*",
      ))
    }
  }
}

// customize diarc-diarcRos jar
tasks.named<Jar>("jar") {
  archiveBaseName = "diarc-diarcRos"
  archiveVersion = properties["diarc.version"].toString()

  manifest {
    attributes(
        "Implementation-Title" to archiveBaseName,
        "Implementation-Version" to archiveVersion,
    )
  }
}

tasks.named<JavaCompile>("compileMockJava") {
  options.sourcepath = files(listOf("src/main/java"))
  options.compilerArgs = listOf("-parameters")
}

tasks.named("processMockResources") {
  dependsOn("processResources")
}

val mockJarTask = tasks.register<Jar>("mockJar") {
  from(sourceSets.getByName("mock").output)
  archiveBaseName = "diarc-diarcRos-mock"
  archiveVersion = properties["diarc.version"].toString()
  archiveExtension = "jar"

  manifest {
    attributes(
        "Implementation-Title" to archiveBaseName,
        "Implementation-Version" to archiveVersion,
    )
  }
}

val mockJar by configurations.creating {
  isCanBeConsumed = true
  isCanBeResolved = false
  extendsFrom(configurations["mockImplementation"], configurations["mockRuntimeOnly"])
}

artifacts {
  add("mockJar", mockJarTask)
}

repositories {
  mavenCentral()
  mavenLocal()
  maven {
//      url 'https://maven-us.nuxeo.org/nexus/content/repositories/public'
    url = uri("https://plugins.gradle.org/m2/")
  }
  maven {
    url = uri("https://clojars.org/repo") //This is needed for diarcros - but technically should come from the closure plugin. todo: Apply closure plugin instead?
  }
  maven { // hrilab archive
    name = "HRILabArchiva"
    url = uri("http://hrilab.tufts.edu:11361/repository/internal/")
    isAllowInsecureProtocol = true
  }
}

dependencies {
  configurations.getByName("mockImplementation")(project(":core"))
  api(project(":core"))
  if (project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean()) {
    api("rosjava_core:rosjava:0.0.0")
    api("rosjava_core:rosjava_bootstrap:0.0.0")
    api("rosjava_core:rosjava_messages:0.0.0")
    api("rosjava_core:actionlib_java:0.0.0")
    api("edu.tufts.hrilab:diarcros_messages:0.0.0")
    api("org.jboss.netty:netty:3.2.10.Final")
  }
}

addRosSource("common", listOf(
    "edu/tufts/hrilab/diarcros/common/*",
))

addRosSource("moveit", listOf(
    "edu/tufts/hrilab/moveit/**",
    "edu/tufts/hrilab/diarcros/moveit/GenericMoveGroup.java",
    "edu/tufts/hrilab/diarcros/moveit/" + project.properties.getOrDefault("diarc.rosVersion", "none").toString() + "/MoveGroup.java",
))

addRosSource("map", listOf(
    "edu/tufts/hrilab/diarcros/map/**",
), listOf("common"))

addRosSource("movebase", listOf(
    "edu/tufts/hrilab/movebase/**",
), listOf("map"))

addRosSource("kortex", listOf(
    "edu/tufts/hrilab/manipulator/kortex/*",
    "edu/tufts/hrilab/diarcros/kortex/*",
), listOf("moveit"))

addRosSource("fetch", listOf(
    "edu/tufts/hrilab/fetch/*",
    "edu/tufts/hrilab/diarcros/fetch/*",
    "edu/tufts/hrilab/manipulator/fetch/*",
), listOf("moveit"))

addRosSource("lidar", listOf(
    "edu/tufts/hrilab/lidar/*",
))
addRosSource("imu", listOf(
    "edu/tufts/hrilab/imu/*",
))

addRosSource("pr2", listOf(
    "edu/tufts/hrilab/pr2/*",
    "edu/tufts/hrilab/diarcros/pr2/*",
    "edu/tufts/hrilab/manipulator/pr2/*",
), listOf("moveit"))

addRosSource("spot", listOf(
    "edu/tufts/hrilab/spot/*",
), listOf("movebase"))

addRosSource("tower", listOf(
    "edu/tufts/hrilab/tower/*",
    "edu/tufts/hrilab/diarcros/tower/*",
), listOf("movebase"))