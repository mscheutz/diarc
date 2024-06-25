// Apply the plugin
plugins {
    id("io.github.pereduromega.node.plugin") version "2.0.7"
}

// When downloadNode is set to true you must provide a repository to download node
repositories {
    // For the default repository, just use
    nodeRepository()
}

// Example to further configure tasks extracted from scripts in package.json
//tasks.named<NodeScriptTask>("build") {
//    // Assign this task to a specific group (default is "scripts")
//    group = BasePlugin.BUILD_GROUP
//
//    // Configure the task inputs and outputs to allow for up-to-date checks
//    inputs.dir("src")
//    outputs.dir("dist")
//
//    // Other optional properties
//    ignoreExitValue.set(false) // default true
//    command.set("run")
//    args.set(listOf("scriptName"))
//    packageManager.set(PackageManager.NPM)
//}

// The configuration block node is mandatory even if it is empty
node {
    // All possible configuration options with their default value are shown below
    packageJson.set(project.file("package.json"))
    nodeModules.set(project.file("node_modules"))
    workingDir.set(project.projectDir)
    defaultTaskGroup.set("scripts")
//    autoCreateTasksFromPackageJsonScripts.set(true)
//    tasksDependingOnNodeInstallByDefault.set(true)
//    scriptsDependingOnNodeDevInstall.set(listOf())
//    scriptsDependingOnNodeInstall.set(listOf())
//    installCommand.set('install')
//    nodeVersion.set("18.19.0")
//    nodePath.set("")
    downloadNode.set(true)
    packageManager.set(PackageManager.NPM)
    cleanTaskName.set("nodeClean")
}