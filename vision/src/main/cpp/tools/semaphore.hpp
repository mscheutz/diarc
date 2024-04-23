/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Implements a semaphore using boost mechanisms.
 * (taken from a post by Maxim Yegorushkin from stackoverflow.com)
 *
 * @author Michael Zillich
 * @date Nov 2012
 */

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

class semaphore
{
private:
    boost::mutex mutex_;
    boost::condition_variable condition_;
    unsigned long count_;

public:
    semaphore()
        : count_()
    {}

    void notify()
    {
        boost::mutex::scoped_lock lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    void wait()
    {
        boost::mutex::scoped_lock lock(mutex_);
        while(!count_)
            condition_.wait(lock);
        --count_;
    }

    bool trywait()
    {
        boost::mutex::scoped_lock lock(mutex_);
        if(count_)
        {
            --count_;
            return true;
        }
        else
        {
            return false;
        }
    }
};
