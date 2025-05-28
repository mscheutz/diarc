/*
Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#ifndef PYTHONHELPER_HPP
#define PYTHONHELPER_HPP

#include <map>
#include <mutex>
#include <pybind11/pybind11.h>
#include <log4cxx/logger.h>

class PythonHelper {
public:
    static PythonHelper* getInstance();
    ~PythonHelper();

    // Delete copy constructor and assignment operator to prevent copying
    PythonHelper(const PythonHelper&) = delete;
    PythonHelper& operator=(const PythonHelper&) = delete;

    void initializePython();
    void importModule(const std::string& moduleName);

private:
    PythonHelper();
    static PythonHelper* instance;
    static std::mutex mutex;

    log4cxx::LoggerPtr logger;
    bool initialized = false;

    std::map<std::string, pybind11::module> importedModules;

};


#endif //PYTHONHELPER_HPP
