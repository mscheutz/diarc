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

    void acquireGIL();
    void releaseGIL();

    void initializePython();
    void finalizePython();
    pybind11::module_ importModule(const std::string& moduleName);

private:
    PythonHelper();
    static PythonHelper* instance;
    static std::mutex instanceMuutex;

    std::recursive_mutex mutex;

    PyGILState_STATE gstate;

    log4cxx::LoggerPtr logger;
    bool initialized = false;

    std::map<std::string, pybind11::module> importedModules;

};


#endif //PYTHONHELPER_HPP
