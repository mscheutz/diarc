plugins {
  `java-library`
  scala
  `maven-publish`
}

java {
  toolchain {
    // set the java version
    languageVersion.set(JavaLanguageVersion.of(17))
  }
}

tasks.register<Copy>("copyGitHooks") {
  from(file(rootDir.path + "/git-hooks"))
  include("pre-commit")
  into(rootDir.path + "/.git/hooks")
}
tasks.named("compileJava").get().dependsOn("copyGitHooks")


// the repositories used to search for dependencies
repositories {
  mavenCentral()
//  mavenLocal()
  maven {
    name = "HRILabArchiva"
    url = uri("http://hrilab.tufts.edu:11361/repository/internal/")
    isAllowInsecureProtocol = true
  }
}

sourceSets {
  main {
    java {
      exclude(listOf(
              //scala
              "edu/tufts/hrilab/slug/refResolution/**",
              "edu/tufts/hrilab/nao/NaoExtended*",
      ))
    }
    scala {
      setSrcDirs(listOf("src/main/scala", "src/main/java/edu/tufts/hrilab/slug/refResolution"))
    }
  }

  test {
//    compileClasspath += sourceSets.main.get().output
//    runtimeClasspath += sourceSets.main.get().output

    java {
      exclude(
              //currently not passing
              "edu/tufts/hrilab/slug/pragmatics/**", "edu/tufts/hrilab/slug/nlg/**",
      )
    }
  }

  create("mock") {
    compileClasspath += sourceSets.main.get().compileClasspath + sourceSets.main.get().output
    runtimeClasspath += sourceSets.main.get().runtimeClasspath + sourceSets.main.get().output
  }

  create("controlApp"){
    java{
      srcDir("src/main/java")
      include(listOf("edu/tufts/hrilab/fol/**" ,"edu/tufts/hrilab/slug/common/**","edu/tufts/hrilab/util/**","edu/tufts/hrilab/action/annotations/**","edu/tufts/hrilab/action/ActionStatus.java","edu/tufts/hrilab/action/EffectType.java","edu/tufts/hrilab/action/ConditionType.java","edu/tufts/hrilab/action/util/Utilities.java","edu/tufts/hrilab/action/ActionBinding.java","edu/tufts/hrilab/action/justification/Justification.java"))
      exclude(listOf("edu/tufts/hrilab/slug/common/UtteranceUtil.java"))
      compileClasspath += sourceSets.main.get().compileClasspath
      runtimeClasspath += sourceSets.main.get().runtimeClasspath
    }
  }
}

tasks.named<Javadoc>("javadoc") {
  source = sourceSets.main.get().allJava + sourceSets.getByName("mock").allJava
}

tasks.compileJava {
  options.sourcepath = files(listOf("src/main/java", "src/main/scala"))
  options.compilerArgs = listOf("-parameters")
}

tasks.compileTestJava {
  options.sourcepath = files(listOf("src/main/java", "src/main/scala", "src/test/java", "src/test/scala"))
  options.compilerArgs = listOf("-parameters")
}

tasks.compileScala {
  scalaCompileOptions.additionalParameters.apply {
    scalaCompileOptions.additionalParameters = scalaCompileOptions.additionalParameters.orEmpty() + "-target:jvm-1.8"
  }
  options.compilerArgs = listOf("-parameters")
  options.sourcepath = files(listOf("src/main/java", "src/main/scala/"))

  dependsOn(listOf("processResources", "processTestResources"))
}

tasks.compileTestScala {
  options.sourcepath = files(listOf("src/main/java", "src/main/scala", "src/test/java", "src/test/scala"))
  options.compilerArgs = listOf("-parameters")
}

tasks.named<Test>("test") {
  useJUnit()
  maxHeapSize = "4g"
  forkEvery = 1
  classpath += files("src/test/resources") // this is so the ActionASLTest can read/write into this directory
  systemProperty("logback.configurationFile", properties["diarc.loggingConfigFile"].toString())

  // to run tests with logging printed to console
  if (project.hasProperty("diarc.testLogging") && project.property("diarc.testLogging").toString().toBoolean()) {
    testLogging.showStandardStreams = true
  }
}

tasks.named<JavaCompile>("compileMockJava") {
  options.sourcepath = files(listOf("src/main/java", "src/main/scala"))
  options.compilerArgs = listOf("-parameters")
}

// customize diarc-core jar
tasks.named<Jar>("jar") {
  archiveBaseName = "diarc-core"
  archiveVersion = properties["diarc.version"].toString()

  manifest {
    attributes(
            "Implementation-Title" to archiveBaseName,
            "Implementation-Version" to archiveVersion,
    )
  }
}

//brad:as far as I can tell we need to explicitly define this jar, so it can be used in other subprojects, we can just use the classes
val mockJarTask = tasks.register<Jar>("mockJar") {
  from(sourceSets.getByName("mock").output)
  archiveBaseName = "diarc-core-mock"
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
  isCanBeConsumed =true
  isCanBeResolved = false

  extendsFrom(configurations["mockImplementation"],configurations["mockRuntimeOnly"])
}

artifacts{
  add("mockJar",mockJarTask)
}

dependencies {

  //TODO:brad: these are api to make things smoother for internal use. Would need to consider how that affects external artifact consumers.
  //core diarc and interfaces
  api("commons-cli:commons-cli:1.4")
  api("commons-io:commons-io:2.11.0")
  api("org.apache.commons:commons-collections4:4.4")
  api("org.apache.commons:commons-lang3:3.2")
  api("org.apache.commons:commons-math3:3.0")
  api("org.apache.ws.commons.util:ws-commons-util:1.0.2")
  api("ch.qos.logback:logback-classic:1.4.14")
  api("javax.vecmath:vecmath:1.5.2")
  //gson
  api("com.google.code.gson:gson:2.6.2")
  api("com.google.guava:guava:33.0.0-jre")
  //trade
  api("ai.thinkingrobots:trade:1.0.0")

  //junit
  testImplementation("junit:junit:4.13.1")
  //DB Test, Performance measures test
  testImplementation("org.skyscreamer:jsonassert:1.5.1")

  //action
  implementation("org.antlr:antlr4-runtime:4.13.1")
  implementation("com.github.pellierd:pddl4j:3.8.3")

  //scala for rr
  implementation("org.scala-lang:scala-compiler:2.12.4")
  implementation("org.scala-lang:scala-library:2.12.4")
  implementation("org.scala-lang:scala-reflect:2.12.4")
  implementation("org.scalatest:scalatest_2.12:3.0.5")
  implementation("org.scalacheck:scalacheck_2.12:1.14.0")
  implementation("org.scalamock:scalamock-scalatest-support_2.12:3.6.0")
  implementation("org.scalamock:scalamock-core_2.12:3.6.0")
  //belief
  implementation("it.unibo.alice.tuprolog:tuprolog:3.3.0")
  implementation("org.xerial:sqlite-jdbc:3.45.2.0")

  //sphinx
  implementation("de.sciss:sphinx4-core:1.0.0")
  implementation("de.sciss:sphinx4-data:1.0.0")

  //whisper
  implementation("io.github.givimad:whisper-jni:1.6.1")

  //TLDL Parser component GUI
  implementation("org.jsoup:jsoup:1.12.1")

  //dialogue gui
  implementation("com.intellij:forms_rt:7.0.3")

  //<!-- for REST APIs -->
  implementation("org.apache.httpcomponents.client5:httpclient5:5.1.3")
  implementation("org.apache.httpcomponents.core5:httpcore5:5.1.4")

  implementation("javax.servlet:servlet-api:2.5")
  implementation("org.springframework.boot:spring-boot-starter-websocket:3.2.4")
  implementation("org.springdoc:springdoc-openapi-starter-common:2.5.0")
  implementation("org.json:json:20240303")

  //DesktopFirebaseConnectionComponent
  implementation("com.google.firebase:firebase-admin:7.1.0")
  implementation("com.google.cloud:google-cloud-firestore:2.1.0")
  implementation("com.google.cloud:google-cloud-storage:1.113.0")
  implementation("com.google.api-client:google-api-client-gson:1.30.9")

  //Scale Component
  implementation("org.usb4java:usb4java:1.3.0")
  implementation("org.usb4java:libusb4java:1.3.0")

  //LLM Tokenizers
  implementation("com.knuddels:jtokkit:0.4.0")

  //Environment Configuration
  implementation("io.github.cdimascio:dotenv-java:3.0.0")

  //hosted on HRI LAB
  //simplenlg
  implementation("simplenlg:SimpleNLG:4.4.8")
  //marytts
  implementation("de.dfki.mary:marytts-lang-en:5.0")
  implementation("de.dfki.mary:marytts-server-with-deps-no-slf4j:5.0")
  implementation("de.dfki.mary:voice-cmu-slt-hsmm:5.0")
  implementation("de.dfki.mary:voice-cmu-rms-hsmm:5.0")
  implementation("de.dfki.mary:emospeak:5.0")
  //nao
  implementation("com.aldebaran:qimessaging:2.1.0.19-linux64")

  //ros2
  api("io.github.pinorobotics:jros2services:3.0");
  api("io.github.lambdaprime:jros2client:6.0")
  api("io.github.pinorobotics:jros2actionlib:2.0");
  api("io.github.pinorobotics:jros2tf2:1.0")

  //gui
  implementation("org.reflections:reflections:0.10.2")
  implementation("org.springdoc:springdoc-openapi-starter-webmvc-ui:2.0.4")
  implementation("org.apache.commons:commons-imaging:1.0-alpha3")
}

//TODO:brad: this is potentially no longer needed for mtracs, tbd if it is necessary for the Temi. Those were the two main use cases,
// so this can be removed if it is no longer needed there. It is also possible that the diarc-core publish target below could be sufficient.
//Reference code/legacy support for building "tracs-jar" target
val diarcJarTask = tasks.register<Jar>("diarcJar") {
  from(sourceSets.main.get().output) {
    include(
            listOf(
                    "edu/tufts/hrilab/action/**",
                    "edu/tufts/hrilab/belief/**",
                    "edu/tufts/hrilab/consultant/**",
                    "edu/tufts/hrilab/diarc/**",
                    "edu/tufts/hrilab/fol/**",
                    "edu/tufts/hrilab/interfaces/**",
                    "edu/tufts/hrilab/pddl/**",
                    "edu/tufts/hrilab/slug/**",
                    "edu/tufts/hrilab/util/**",
                    //TODO:brad:should these be included?
//                    "edu/tufts/hrilab/simspeech/**",
//                    "edu/tufts/hrilab/firebase/**"
            )
    )
  }
  archiveBaseName = "diarc-core"
  archiveVersion = properties["diarc.version"].toString()
  archiveExtension = "jar"

  manifest {
    attributes(
            "Implementation-Title" to archiveBaseName,
            "Implementation-Version" to archiveVersion,
    )
  }
}

val controlAppJarTask = tasks.register<Jar>("controlAppJar") {
  from(sourceSets.getByName("controlApp").output)
  archiveBaseName = "diarc-controlApp"
  archiveVersion = properties["diarc.version"].toString()
  archiveExtension = "jar"

  manifest {
    attributes(
            "Implementation-Title" to archiveBaseName,
            "Implementation-Version" to archiveVersion,
    )
  }
}

val controlAppJar by configurations.creating{
  isCanBeConsumed =true
  isCanBeResolved = false

  extendsFrom(configurations["api"])
}

artifacts{
    add("controlAppJar",controlAppJarTask)
}

publishing {
  publications {
    create<MavenPublication>("diarc-core") {
      groupId = "edu.tufts.hrilab"
      artifactId = "diarc-core"
      version = properties["diarc.version"].toString()

      from(components["java"])

      //  TODO:update pom with license info and correct repo path
      pom {
        name = "diarc core"
        description = "core diarc functionality"
        url = "https://hrilab.tufts.edu:22280/ade/ade"
//                licenses {
//                    license {
//                        name = "DIARC License"
//                        url = "https://hrilab.tufts.edu:22280/ade/ade"
//                    }
//                }
      }
    }
    create<MavenPublication>("diarc-controlApp") {
      groupId = "edu.tufts.hrilab"
      artifactId = "diarc-controlApp"
      version = properties["diarc.version"].toString()

      artifact(controlAppJarTask)
//      from(components["controlApp"])

      //  TODO:update pom with license info and correct repo path
      pom {
        name = "diarc control app"
        description = "diarc libs used in control app"
        url = "https://hrilab.tufts.edu:22280/ade/ade"
//                licenses {
//                    license {
//                        name = "DIARC License"
//                        url = https://github.com/mscheutz/diarc
//                    }
//                }
      }
    }

  }
}
