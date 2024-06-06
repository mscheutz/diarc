import groovyjarjarantlr.build.ANTLR

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
  systemProperty("logback.configurationFile", properties["diarc.loggingConfigFile"].toString())
  systemProperty("java.library.path", visionDir.toString() + "/build/cpp/lib:" + visionDir + "/src/main/cpp/third_party/vlfeat/bin/glnxa64:.:" + environment["LD_LIBRARY_PATH"])
}

sourceSets{
  create("mock") {
  }
  main {
    java {
      setSrcDirs(listOf("src/main/java", "src/swig/java"))
    }
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
  dependsOn("compileVisionNative")
}

tasks.register<Exec>("compileVisionNative") {
  onlyIf{
    project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()
  }

  workingDir("build/cpp")

  //on linux
  commandLine("make", "-j", properties.getOrDefault("diarc.make.j", "4").toString())
  dependsOn("cmakeVisionNative")
}

tasks.register<Exec>("cmakeVisionNative") {
  onlyIf{
    project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()
  }

  //on linux
  commandLine("cmake", "-Bbuild/cpp", "-Hsrc/main/cpp")
}

tasks.named<Javadoc>("javadoc") {
  source = sourceSets.getByName("mock").allJava

  if (project.hasProperty("diarc.enableVision") && project.property("diarc.enableVision").toString().toBoolean()) {
    source += sourceSets.main.get().allJava
  }
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
  from(sourceSets.getByName("mock").output + sourceSets.getByName("main").resources)
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
