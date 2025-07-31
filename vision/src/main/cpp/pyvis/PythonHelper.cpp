/*
Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#include "PythonHelper.hpp"

#include <boost/format.hpp>
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
  _state = PyGILState_Ensure(); // c++ thread
}

void PythonHelper::releaseGIL() {
    PyGILState_Release(_state); // c++ thread
    //mutex.unlock();
}

void PythonHelper::initializePython() {
    std::lock_guard<std::recursive_mutex> lock(mutex);

    if (!initialized) {
        LOG4CXX_DEBUG(logger, "Initializing python...");
        // TODO: how to handle this in general?
        // When you load a NumPy-dependent plugin, do the following just before that.
        void *handle = dlopen("libpython3.8.so", RTLD_LAZY | RTLD_GLOBAL);
        if (!handle) {
          handle = dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);
        }
        if (!handle) {
          handle = dlopen("libpython3.12.so", RTLD_LAZY | RTLD_GLOBAL);
        }
        if (!handle) {
            LOG4CXX_ERROR(logger, "Failed to load python library. 3.8.x, 3.10.x, 3.12.x versions supported.");
        }
        py::initialize_interpreter();

        LOG4CXX_DEBUG(logger, "release GIL...");
        _save = PyEval_SaveThread();
        LOG4CXX_DEBUG(logger, boost::format("GIL held: %d") % PyGILState_Check());

        initialized = true;
    } else {
        LOG4CXX_DEBUG(logger, "Python already initialized.");
        LOG4CXX_DEBUG(logger, boost::format("GIL held: %d") % PyGILState_Check());
    }
}

void PythonHelper::finalizePython() {
    std::lock_guard<std::recursive_mutex> lock(mutex);

    LOG4CXX_DEBUG(logger, "[finalizePython] methode entered");

    if (initialized) {
        LOG4CXX_DEBUG(logger, "[finalizePython] 0");
        PyEval_RestoreThread(_save); // acquire GIL
        LOG4CXX_DEBUG(logger, "[finalizePython] 1");
        py::finalize_interpreter();
        LOG4CXX_DEBUG(logger, "[finalizePython] 2");
        initialized = false;
    }
    LOG4CXX_DEBUG(logger, "[finalizePython] done");
}

pybind11::module_ PythonHelper::importModule(const std::string& moduleName) {
    std::lock_guard<std::recursive_mutex> lock(mutex);

    if (!initialized) {
        LOG4CXX_ERROR(logger, "[importModule] Python not initialized.");
        return {};
    }

    if (importedModules.find(moduleName) != importedModules.end()) {
        return importedModules[moduleName];
    } else {
        py::module_ module = py::module_::import(moduleName.c_str());
        importedModules[moduleName] = module;
        return module;
    }
}
