import org.gradle.kotlin.dsl.support.uppercaseFirstChar

plugins {
  `java-library`
  application
}

tasks.compileJava {

  if (project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean()) {
    if (project.hasProperty("diarc.rosPackages")) {
      // always include core diarcRos sourceSet
      dependsOn(":diarcRos:compileCoreJava")

      val rosPackages = project.findProperty("diarc.rosPackages").toString().split(",")
      for (rosPackage in rosPackages) {
        dependsOn(":diarcRos:compile" + rosPackage.trim().uppercaseFirstChar() + "Java")
      }
    }
  }

  options.compilerArgs = listOf("-parameters")
}

sourceSets {

  //for configs which don't depend on any external sources
  //TODO:brad: do we want all of them in the same target?
  main {
    java {
      exclude("**")
    }
  }

  test {
    java {
      exclude(
              "edu/tufts/hrilab/config/llm/*",
              "edu/tufts/hrilab/demos/llm/*",
      )
    }
  }
}

dependencies {
  implementation("ch.qos.logback:logback-classic:1.4.14") //TODO:figure out how to inherit this from the rootProject
  implementation(project(":core"))
  implementation(project(":core","mockJar"))
  implementation(project(":vision","mockJar"))
  implementation(project(":diarcRos"))
  implementation(project(":diarcRos","mockJar"))

  if(project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean()) {
    implementation(project(":diarcRos"))
  }

//  if(project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()) {
//    implementation(project(":vision"))
//  }

  testImplementation("ai.thinkingrobots:mtracs-mock:1.12.1") {
    exclude(group = "edu.tufts.hrilab", module = "diarc-core") //avoid a transitive dependency with the diarc-core inclusion for the external project.
  }

  testImplementation("junit:junit:4.13.1") //TODO:figure out how to inherit this from the rootProject
  testImplementation("com.fasterxml.jackson.core:jackson-core:2.7.4")
  testImplementation("com.fasterxml.jackson.core:jackson-annotations:2.7.4")
  testImplementation("com.fasterxml.jackson.core:jackson-databind:2.7.4")
  testImplementation(project(":core"))
  testImplementation(project(":core","mockJar"))
  testImplementation(project(":diarcRos"))
  testImplementation(project(":diarcRos","mockJar"))
  testImplementation(project(":vision","mockJar"))

  implementation("org.springframework.boot:spring-boot-starter-websocket:3.2.4")
}

// customize diarc-config jar
tasks.named<Jar>("jar") {
  archiveBaseName = "diarc-config"
  archiveVersion = properties["diarc.version"].toString()

  manifest {
    attributes(
            "Implementation-Title" to archiveBaseName,
            "Implementation-Version" to archiveVersion,
    )
  }
}

tasks.named<Test>("test") {
  useJUnit()
  maxHeapSize = "4g"
  forkEvery = 1
  systemProperty("diarc.planner.ff", properties["diarc.planner.ff"].toString())
  systemProperty("logback.configurationFile", properties["diarc.loggingConfigFile"].toString())
  systemProperty("logging.config", properties["diarc.loggingConfigFile"].toString()) // for springboot

  // to run tests with logging printed to console
  if (project.hasProperty("diarc.test.enableLogging") && project.property("diarc.test.enableLogging").toString().toBoolean()) {
    testLogging.showStandardStreams = true
  }

  // to run integration test(s) in generative  mode
  if (project.hasProperty("diarc.test.generativeMode") && project.property("diarc.test.generativeMode").toString().toBoolean()) {
    systemProperty("generativeMode", "true")
  }

  systemProperty("trade.properties.path", properties.getOrDefault("diarc.test.tradePropertiesFile", "src/main/resources/default/trade.properties.default").toString())
  systemProperty("tradeLogging.config.path", properties.getOrDefault("diarc.test.tradeLoggingConfigFile", "src/main/resources/default/tradeLogging.config").toString())

  //TODO:brad: is this the best way to include these resources?
  classpath += files("src/test/resources") +
          files(project(":diarcRos").projectDir.path+"/src/main/resources") +
          files(project(":vision").projectDir.path+"/src/main/resources") +
          files(project(":core").projectDir.path+"/src/test/resources")
}

tasks.register<JavaCompile>("compileConfig") {
  // compile DIARC configuration file
  if (rootProject.name == "diarc") {
    // if executing task from main diarc repo, set source relative to :config subproject
    source("src/main/java/", "src/test/java")
  } else {
    // else executing task from a gradle project outside the main diarc repo, assume this full path
    source(rootProject.rootDir.path + "/src/main/java/", rootProject.rootDir.path + "/src/test/java/")
  }
  options.compilerArgs = listOf("-parameters")
  include(project.findProperty("main").toString().replace(".","/")+".java")
  classpath = sourceSets.main.get().compileClasspath
  destinationDirectory = file(project.layout.buildDirectory.get().toString()+"/classes/java/main/")

  // add :diarcRos dependencies
  if (project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean()) {
    dependencies {
      implementation(project(":diarcRos", "coreJar"))
    }

    if (project.hasProperty("diarc.rosPackages")) {
      val rosPackages = project.findProperty("diarc.rosPackages").toString().split(",")
      for (rosPackage in rosPackages) {
        dependencies {
          implementation(project(":diarcRos", rosPackage.trim() + "Jar"))
        }
      }
    }
  }

  // add :vision dependencies
  if (project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()) {
    // not using vision Jar here because c++ cannot currently read resource files from a Jar
    classpath += project(":vision").sourceSets.main.get().output
  }
}

tasks.register<JavaExec>("launch") {
  dependsOn("compileConfig")

  classpath = sourceSets.main.get().runtimeClasspath

  classpath += files(System.getProperty("user.home") + "/.diarc")

  // planner executable path
  systemProperty("diarc.planner.ff", properties["diarc.planner.ff"].toString())

  // conditionally add vision
  if (project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()) {
    // add vision classes and resources
    // not using vision Jar here because c++ cannot currently read resource files from a Jar
    classpath += project(":vision").sourceSets.main.get().output

    // to find native libraries
    val visionDir = project(":vision").projectDir.path
    systemProperty("java.library.path", visionDir + "/build/cpp/lib:" + visionDir + "/src/main/cpp/third_party/vlfeat/bin/glnxa64:.:" + environment["LD_LIBRARY_PATH"])
  }

  // if launching YAML DIARC config (e.g., -Pyaml=<path/to/file.yaml>)
  if (project.hasProperty("yaml")) {
    mainClass = "edu.tufts.hrilab.diarc.YamlDiarcConfiguration"
    systemProperty("component", "edu.tufts.hrilab.diarc.YamlDiarcConfiguration")
    systemProperty("yaml", project.findProperty("yaml").toString())
  }

  // jvm args
  if (project.hasProperty("main")) {
    mainClass = project.findProperty("main").toString()
    systemProperty("component", project.findProperty("main").toString())
  }
  systemProperty("logback.configurationFile", properties.getOrDefault("diarc.loggingConfigFile", "src/main/resources/default/logback.xml").toString())
  systemProperty("logging.config", properties.getOrDefault("diarc.loggingConfigFile", "src/main/resources/default/logback.xml").toString()) // for springboot
  systemProperty("trade.properties.path", properties.getOrDefault("diarc.tradePropertiesFile", "src/main/resources/default/trade.properties.default").toString())
  systemProperty("tradeLogging.config.path", properties.getOrDefault("diarc.tradeLoggingConfigFile", "src/main/resources/default/tradeLogging.config").toString())

  // Conditionally enable features
  if (project.hasProperty("enableTradeTracker")) {
    systemProperty("enableTradeTracker", project.property("enableTradeTracker").toString())
  } else {
    systemProperty("enableTradeTracker", "false")
  }
}

tasks.register<Javadoc>("allJavadoc") {
//  dependsOn("assemble")

  title = "DIARC"
  setDestinationDir(file("${rootDir}/docs"))

  // keep cmu sphinx4 files out of javadoc -- these files should be removed from source tree
  setExcludes(listOf("edu/tufts/hrilab/sphinx4/SphinxRecognizer.java", "edu/tufts/hrilab/sphinx4/ConfigurableMicrophone.java", "edu/tufts/hrilab/sphinx4/SegmentStorer.java"))

  // :core
  source += project(":core").sourceSets.main.get().allJava + project(":core").sourceSets.getByName("mock").allJava
  classpath += project(":core").sourceSets.main.get().compileClasspath + project(":core").sourceSets.getByName("mock").compileClasspath

  // :vision
  source += project(":vision").sourceSets.getByName("mock").allJava
  classpath += project(":vision").sourceSets.getByName("mock").compileClasspath
  if (project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()) {
    dependsOn(":vision:compileJava") // need to generate swig Java classes
    source += project(":vision").sourceSets.main.get().allJava
    classpath += project(":vision").sourceSets.main.get().compileClasspath
  }

  // :diarcRos
  source += project(":diarcRos").sourceSets.main.get().allJava + project(":diarcRos").sourceSets.getByName("mock").allJava
  classpath += project(":diarcRos").sourceSets.main.get().compileClasspath + project(":diarcRos").sourceSets.getByName("mock").compileClasspath
  if (project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean() && project.hasProperty("diarc.rosPackages")) {
    // always add core when enableRos is true
    source += project(":diarcRos").sourceSets.getByName("core").allJava
    classpath += project(":diarcRos").sourceSets.getByName("core").compileClasspath

    val rosPackages = project.findProperty("diarc.rosPackages").toString().split(",")
    for (rosPackage in rosPackages) {
      val srcSetName = rosPackage.trim().lowercase()
      source += project(":diarcRos").sourceSets.getByName(srcSetName).allJava
      classpath += project(":diarcRos").sourceSets.getByName(srcSetName).compileClasspath
    }
  }
}

repositories {
  mavenCentral() // main maven archive
  maven {
    url = uri("https://plugins.gradle.org/m2/")
  }
  mavenLocal() // local maven archive
  maven {
    name = "Thinking Robots mtracs mock"
    url = uri("https://gitlab.com/api/v4/projects/34394212/packages/maven/")
  }
  maven {
    name = "Thinking Robots TRADE mvn host"
    url = uri("https://gitlab.com/api/v4/projects/31017133/packages/maven")
  }
  maven { // hrilab archive
    name = "HRILabArchiva"
    url = uri("http://hrilab.tufts.edu:11361/repository/internal/")
    isAllowInsecureProtocol = true
  }
}
