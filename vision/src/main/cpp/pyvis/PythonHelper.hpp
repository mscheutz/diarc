/*
Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#ifndef PYTHONHELPER_HPP
#define PYTHONHELPER_HPP

#include <map>
#include <mutex>
#include <pybind11/pybind11.h>
#include <log4cxx/logger.h>

class PYBIND11_EXPORT PythonHelper {
public:
    static PythonHelper* getInstance();
    ~PythonHelper();

    // Delete copy constructor and assignment operator to prevent copying
    PythonHelper(const PythonHelper&) = delete;
    PythonHelper& operator=(const PythonHelper&) = delete;

    void initializePython();
    void finalizePython();

    void acquireGIL();
    void releaseGIL();

    pybind11::module_ importModule(const std::string& moduleName);
    pybind11::object importFromModule(const std::string& moduleName, const std::string& objectName);

private:
    PythonHelper();
    static PythonHelper* instance;
    static std::mutex instanceMuutex;

    std::recursive_mutex mutex;

    PyThreadState *_save;

    log4cxx::LoggerPtr logger;
    bool initialized = false;

    std::map<std::string, pybind11::module_> importedModules;
    std::map<std::string, pybind11::object> importedObjects;

};


#endif //PYTHONHELPER_HPP
