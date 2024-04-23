plugins {
  `java-library`
}

repositories {
//  mavenLocal()
  mavenCentral() // main maven archive
    maven { // hrilab archive
    name = "HRILabArchiva"
    url = uri("http://hrilab.tufts.edu:11361/repository/internal/")
    isAllowInsecureProtocol = true
  }
}

tasks.named<Test>("test") {
  useJUnit()
  maxHeapSize = "4g"
  forkEvery = 1
  classpath += files("src/test/resources")

  val visionDir = layout.projectDirectory;
  jvmArgs = listOf("-Dlogback.configurationFile=" + properties["diarc.loggingConfigFile"],
          "-Djava.library.path=" + visionDir + "/build/cpp/lib:" + visionDir + "/src/main/cpp/third_party/vlfeat/bin/glnxa64:.:" + environment["LD_LIBRARY_PATH"]
  )
}

sourceSets{
  create("mock") {
  }
  create("swig") {
    compileClasspath += project(":core").sourceSets.main.get().compileClasspath + project(":core").sourceSets.main.get().output
    runtimeClasspath += project(":core").sourceSets.main.get().runtimeClasspath + project(":core").sourceSets.main.get().output
  }
  main{
    compileClasspath += sourceSets.getByName("swig").compileClasspath + sourceSets.getByName("swig").output
    runtimeClasspath += sourceSets.getByName("swig").runtimeClasspath + sourceSets.getByName("swig").output
  }
  test{
    java{
      exclude(
      //don't pass
              "edu/tufts/hrilab/vision/stm/MemoryObjectTest.java",
              "edu/tufts/hrilab/vision/stm/AvailableSearchManagersTest.java",
              "edu/tufts/hrilab/vision/VisionComponentTest.java",
      )
    }
  }
}

tasks.compileJava {
  onlyIf{
    project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()
  }
  options.compilerArgs = listOf("-parameters")
  dependsOn("compileSwigJava")
}

tasks.named("compileSwigJava") {
  onlyIf{
    project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()
  }

  dependsOn("compileVisionNative")
}

tasks.register<Exec>("compileVisionNative") {
  onlyIf{
    project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()
  }

  workingDir("build/cpp")

  //on linux
  commandLine("make", "-j", project.property("diarc.make.j").toString())
  dependsOn("cmakeVisionNative")
}

tasks.register<Exec>("cmakeVisionNative") {
  onlyIf{
    project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()
  }

  //on linux
  commandLine("cmake", "-Bbuild/cpp", "-Hsrc/main/cpp")
}

tasks.named("clean") {
  doLast {
    val path = layout.projectDirectory.toString() + "/src/swig"
    delete(path)
  }
}

// customize diarc-vision jar
tasks.named<Jar>("jar") {
  archiveBaseName = "diarc-vision"
  archiveVersion = properties["diarc.version"].toString()

  manifest {
    attributes(
            "Implementation-Title" to archiveBaseName,
            "Implementation-Version" to archiveVersion,
    )
  }
}

tasks.named<JavaCompile>("compileMockJava") {
  options.compilerArgs = listOf("-parameters")
}

tasks.named("processMockResources") {
  dependsOn("processResources")
}

val mockJarTask = tasks.register<Jar>("mockJar") {
  from(sourceSets.getByName("mock").output)
  archiveBaseName = "diarc-vision-mock"
  archiveVersion = properties["diarc.version"].toString()
  archiveExtension = "jar"

  manifest {
    attributes(
            "Implementation-Title" to archiveBaseName,
            "Implementation-Version" to archiveVersion,
    )
  }
}

val mockJar by configurations.creating{
  isCanBeConsumed = true
  isCanBeResolved = false

  extendsFrom(configurations["mockImplementation"],configurations["mockRuntimeOnly"])
}

artifacts{
  add("mockJar",mockJarTask)
}

dependencies {
  configurations.getByName("mockImplementation")(project(":core"))

    //TODO:figure out how to inherit this from the rootProject
  implementation("ch.qos.logback:logback-classic:1.4.14")
  implementation(project(":core"))

  implementation("com.google.code.gson:gson:2.6.2")
  implementation("javax.vecmath:vecmath:1.5.2")
  implementation("commons-cli:commons-cli:1.4")

  testImplementation("junit:junit:4.13.1")

}

tasks.register<JavaExec>("launchVision") {
  workingDir = rootDir
  classpath =  sourceSets.main.get().runtimeClasspath + sourceSets.getByName("swig").runtimeClasspath
  mainClass = "edu.tufts.hrilab.vision.VisionComponentImpl"

  val visionDir = layout.projectDirectory;
  jvmArgs = listOf(
          "-Dcomponent=edu.tufts.hrilab.vision.VisionComponentImpl",
          "-Dtrade.properties.path=" + properties["diarc.tradePropertiesFile"].toString(),
          "-DtradeLogging.config.path=" + properties["diarc.tradeLoggingConfigFile"].toString(),
          "-Dlogback.configurationFile=" + properties["diarc.loggingConfigFile"].toString(),
          "-Djava.library.path=" + visionDir + "/build/cpp/lib:" + visionDir + "/src/main/cpp/third_party/vlfeat/bin/glnxa64:.:" + environment["LD_LIBRARY_PATH"]
  )
}
