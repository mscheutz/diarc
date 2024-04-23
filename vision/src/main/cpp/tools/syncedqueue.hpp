/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * A synchronised queue.
 * Essentially a queue together with semaphore semantics.
 *
 * @author Michael Zillich
 * @date Nov 2012
 */

#include <queue>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <log4cxx/logger.h>

template <class T>
class syncedqueue {
private:
  boost::mutex mutex;
  boost::condition_variable condition;
  std::queue<T> queue;
  log4cxx::LoggerPtr logger;

public:
  syncedqueue() {
    logger = log4cxx::Logger::getLogger("ade.tools.syncedqueue");
  }

  void push(T elem_) {
    boost::mutex::scoped_lock lock(mutex);
    queue.push(elem_);
    condition.notify_one();
  }

  T waitpop() {
    boost::mutex::scoped_lock lock(mutex);
    //needs to be interrupt-able (i.e., can't use while)
    if (queue.empty())
      condition.wait(lock);

    T front;
    if (queue.empty()) {
      //return default constructed object
      front = T();
    } else {
      front = queue.front();
      queue.pop();
    }
    return front;
  }

  bool trypop(T &front_) {
    boost::mutex::scoped_lock lock(mutex);
    if (!queue.empty()) {
      front_ = queue.front();
      queue.pop();
      return true;
    } else {
      return false;
    }
  }

  void wait() {
    LOG4CXX_TRACE(logger, "[wait] method entered.");
    boost::mutex::scoped_lock lock(mutex);
    LOG4CXX_TRACE(logger, "[wait] locked.");
    //needs to be interrupt-able (i.e., can't use while)
    if (queue.empty())
      condition.wait(lock);
  }

  size_t size() {
    boost::mutex::scoped_lock lock(mutex);
    return queue.size();
  }
  
  void interrupt() {
    LOG4CXX_TRACE(logger, "[interrupt] method entered.");
    boost::mutex::scoped_lock lock(mutex);
    LOG4CXX_TRACE(logger, "[interrupt] locked.");
    condition.notify_one();
  }
};
