/*
Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#include "PythonHelper.hpp"

#include <pybind11/embed.h> // everything needed for embedding
#include <dlfcn.h>
namespace py = pybind11;

PythonHelper* PythonHelper::instance;
std::mutex PythonHelper::mutex;

PythonHelper* PythonHelper::getInstance() {
    std::lock_guard lock(mutex);
    if (instance == nullptr) {
        instance = new PythonHelper();
    }
    return instance;
}

PythonHelper::~PythonHelper() {
    LOG4CXX_INFO(logger, "[destructor] methode entered");
    py::finalize_interpreter();
    LOG4CXX_INFO(logger, "[destructor] done");
}

PythonHelper::PythonHelper()
    : logger(log4cxx::Logger::getLogger("diarc.visionproc.VisionProcess")) {
}

void PythonHelper::initializePython() {
    std::lock_guard lock(mutex);

    if (!initialized) {
        // TODO: how to handle this in general?
        // When you load a NumPy-dependent plugin, do the following just before that.
        dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);

        py::initialize_interpreter();

        initialized = true;
    }
}

void PythonHelper::importModule(const std::string& moduleName) {
    std::lock_guard lock(mutex);
    if (!initialized) {
        initializePython();
    }

    if (importedModules.find(moduleName) != importedModules.end()) {
        //return importedModules[moduleName];
    } else {
        py::module_ module = py::module_::import(moduleName.c_str());
        importedModules[moduleName] = module;
        //return module;
    }
}
