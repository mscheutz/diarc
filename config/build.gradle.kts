import org.gradle.kotlin.dsl.support.uppercaseFirstChar

plugins {
  `java-library`
  application
}

tasks.compileJava {

  if (project.hasProperty("diarc.rosPackages")) {
    val rosPackages = project.findProperty("diarc.rosPackages").toString().split(",")
    for(rosPackage in rosPackages) {
      dependsOn(":diarcRos:compile" + rosPackage.uppercaseFirstChar() + "Java")
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
  implementation(project(":diarcRos","mockJar"))

  if(project.hasProperty("diarc.enableRos") && project.property("diarc.enableRos").toString().toBoolean()) {
    implementation(project(":diarcRos"))
  }

  testImplementation("junit:junit:4.13.1") //TODO:figure out how to inherit this from the rootProject
  testImplementation("com.fasterxml.jackson.core:jackson-core:2.7.4")
  testImplementation("com.fasterxml.jackson.core:jackson-annotations:2.7.4")
  testImplementation("com.fasterxml.jackson.core:jackson-databind:2.7.4")
  testImplementation(project(":core"))
  testImplementation(project(":core","mockJar"))
  testImplementation(project(":diarcRos","mockJar"))
  testImplementation(project(":vision","mockJar"))
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
  systemProperty("logback.configurationFile", "../" + properties["diarc.loggingConfigFile"])

  // to run tests with logging printed to console
  if (project.hasProperty("diarc.testLogging") && project.property("diarc.testLogging").toString().toBoolean()) {
    testLogging.showStandardStreams = true
  }

  // to run integration test(s) in generative  mode
  if (project.hasProperty("generativeMode") && project.property("generativeMode").toString().toBoolean()) {
    systemProperty("generativeMode", "true")
  }

  //TODO:brad: is this the best way to include these resources?
  classpath += files("src/test/resources") +
          files(rootProject.rootDir.path+"/diarcRos/src/main/resources") +
          files(rootProject.rootDir.path+"/vision/src/main/resources") +
          files(rootProject.rootDir.path+"/core/src/test/resources")
}

tasks.register<JavaCompile>("compileConfig"){
  if (project.hasProperty("diarc.rosPackages")) {
    val rosPackages = project.findProperty("diarc.rosPackages").toString().split(",")
    for(rosPackage in rosPackages) {
      dependencies {
        implementation(project(":diarcRos",rosPackage.trim()+"Jar"))
      }
    }
  }

  source("src/main/java/","src/test/java")
  options.compilerArgs = listOf("-parameters")
  include(project.findProperty("main").toString().replace(".","/")+".java")
  //TODO:brad:add ros classes to classpath?
  classpath = sourceSets.main.get().compileClasspath + files(rootProject.rootDir.path.toString()+"/vision/build/classes/java/main/")
  destinationDirectory = file(project.layout.buildDirectory.get().toString()+"/classes/java/main/")
}

tasks.register<JavaExec>("launch") {
  dependsOn("compileConfig")

  workingDir = rootDir

  //TODO:brad: add ros classes to classpath
  classpath =  sourceSets.main.get().runtimeClasspath +
          files(rootProject.rootDir.path.toString()+"/vision/build/classes/java/main/") +
          files(rootProject.rootDir.path.toString()+"/vision/build/classes/java/swig/") +
          files(rootProject.rootDir.path.toString()+"/vision/build/resources/main/")

  mainClass = project.findProperty("main").toString()
  systemProperty("diarc.planner.ff", properties["diarc.planner.ff"].toString())

  val visionDir = rootProject.rootDir.path.toString() + "/vision"
  jvmArgs = listOf(
          "-Dcomponent=" + project.findProperty("main").toString(),
          "-Dtrade.properties.path=" + properties["diarc.tradePropertiesFile"].toString(),
          "-DtradeLogging.config.path=" + properties["diarc.tradeLoggingConfigFile"].toString(),
          "-Dlogback.configurationFile=" + properties["diarc.loggingConfigFile"].toString(),
          "-Djava.library.path=" + visionDir + "/build/cpp/lib:" + visionDir + "/src/main/cpp/third_party/vlfeat/bin/glnxa64:.:" + environment["LD_LIBRARY_PATH"]
  )
}

repositories {
  mavenCentral() // main maven archive
  maven {
    url = uri("https://plugins.gradle.org/m2/")
  }
  mavenLocal() // local maven archive
  maven { // hrilab archive
    name = "HRILabArchiva"
    url = uri("http://hrilab.tufts.edu:11361/repository/internal/")
    isAllowInsecureProtocol = true
  }
}
