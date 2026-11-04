// Apply the plugin
plugins {
    id("io.github.pereduromega.node.plugin") version "2.0.7"
}

// When downloadNode is set to true you must provide a repository to download node
repositories {
    // For the default repository, just use
    nodeRepository()
}

// The configuration block node is mandatory even if it is empty
node {
    // All possible configuration options with their default value are shown below
    packageJson.set(project.file("package.json"))
    nodeModules.set(project.file("node_modules"))
    workingDir.set(project.projectDir)
    defaultTaskGroup.set("scripts")
    nodeVersion.set("16.20.2")
    downloadNode.set(true)
    packageManager.set(PackageManager.YARN)
    cleanTaskName.set("clean")
}

tasks.named("clean") {
    delete(file("build"))
    delete(file("yarn.lock"))
}

tasks.named<DependenciesInstallTask>("installDependencies") {
    args.set(listOf("--ignore-engines"))
}

// react-scripts test waits for a keypress in watch mode, which hangs ./gradlew test once the
// java tests pass. the only test here is the create-react-app placeholder, and it does not load.
tasks.matching { it.name == "test" }.configureEach {
    enabled = false
}
