/*
Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#include "PythonHelper.hpp"

#include <pybind11/embed.h> // everything needed for embedding
#include <dlfcn.h>
namespace py = pybind11;

PythonHelper* PythonHelper::instance;
std::mutex PythonHelper::instanceMuutex;

PythonHelper* PythonHelper::getInstance() {
    std::lock_guard<std::mutex> lock(instanceMuutex);
    if (instance == nullptr) {
        instance = new PythonHelper();
    }
    return instance;
}

PythonHelper::~PythonHelper() {
    std::lock_guard<std::recursive_mutex> lock(mutex);

    LOG4CXX_INFO(logger, "[destructor] methode entered");
    py::finalize_interpreter();
    LOG4CXX_INFO(logger, "[destructor] done");
}

PythonHelper::PythonHelper()
    : logger(log4cxx::Logger::getLogger("diarc.pyvis.PythonHelper")) {
}

void PythonHelper::acquireGIL() {
    //mutex.lock();
    gstate = PyGILState_Ensure()    ;
    }

void PythonHelper::releaseGIL() {
    PyGILState_Release(gstate);
    //mutex.unlock();
}

void PythonHelper::initializePython() {
    std::lock_guard<std::recursive_mutex> lock(mutex);

    if (!initialized) {
        // TODO: how to handle this in general?
        // When you load a NumPy-dependent plugin, do the following just before that.
        dlopen("libpython3.8.so", RTLD_LAZY | RTLD_GLOBAL);
        //dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);
        py::initialize_interpreter();
        initialized = true;
    }
}

void PythonHelper::finalizePython() {
    std::lock_guard<std::recursive_mutex> lock(mutex);

    LOG4CXX_INFO(logger, "[finalizePython] methode entered");

    if (initialized) {
    LOG4CXX_INFO(logger, "[finalizePython] 0");
        // acquireGIL();
    LOG4CXX_INFO(logger, "[finalizePython] 1");
        py::finalize_interpreter();
    LOG4CXX_INFO(logger, "[finalizePython] 2");
        // releaseGIL();
    LOG4CXX_INFO(logger, "[finalizePython] 3");
        initialized = false;
    }
    LOG4CXX_INFO(logger, "[finalizePython] done");
}

pybind11::module_ PythonHelper::importModule(const std::string& moduleName) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    if (!initialized) {
        initializePython();
    }

    if (importedModules.find(moduleName) != importedModules.end()) {
        return importedModules[moduleName];
    } else {
        py::module_ module = py::module_::import(moduleName.c_str());
        importedModules[moduleName] = module;
        return module;
    }
}
