/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#ifndef CONCURRENT_QUEUE_HPP_
#define CONCURRENT_QUEUE_HPP_

#include <queue>
#include <mutex>
#include <condition_variable>

/**
 * @brief Built for SINGLE consumer and SINGLE publisher!!!
 */
template <typename T>
class concurrent_queue
{
  public:

    concurrent_queue();

    // c++ compliant (almost) queue interface.
    // TODO fill in the missing functions.

    bool empty() const;

    size_t size() const;

    T& front();

    const T& front() const;

    void push( const T& val );

    void pop();

    // extra functions

    /**
     * @brief Block while the queue is empty.
     */
    void waitData() const;

    /**
     * @brief Block while the queue is empty. As soon as the queue
     * is filled, return the front element and pop it.
     * This is equivalent to calling wait() front() and pop()
     * sequentially, but much faster since the mutex lock is performed
     * only once.
     */
    bool waitAndPop(T& val);

    /**
     * @brief Block while the queue is loaded.
     */
    void waitEmpty() const;

    /**
     * Raises a flag to stop the program and signals all condition
     * variables to wake up. If a consumer is waiting on a pop, the
     * returned value is garbage.
     */
    void stop();

  private:

    std::queue<T> queue_;

    bool stop_;

    mutable std::mutex mutex_;

    mutable std::condition_variable is_empty_;

    mutable std::condition_variable not_empty_;
};

template <typename T>
concurrent_queue<T>::concurrent_queue()
  : stop_( false )
{}

template <typename T>
bool concurrent_queue<T>::empty() const
{
  std::unique_lock<std::mutex> lock( mutex_ );
  return queue_.empty();
}

template <typename T>
size_t concurrent_queue<T>::size() const
{
  std::unique_lock<std::mutex> lock( mutex_ );
  return queue_.size();
}

template <typename T>
T& concurrent_queue<T>::front()
{
  std::unique_lock<std::mutex> lock( mutex_ );
  return queue_.front();
}

template <typename T>
const T& concurrent_queue<T>::front() const
{
  std::unique_lock<std::mutex> lock( mutex_ );
  return queue_.front();
}

template <typename T>
void concurrent_queue<T>::push( const T& val )
{
  bool was_empty;

  {
    std::unique_lock<std::mutex> lock( mutex_ );
    was_empty = queue_.empty();
    queue_.push( val );
  }

  // TODO que pasa si justo acá se elimina el elemento (pop)? Es cierto
  // que la cola dejó de estar vacía, pero para cuando se ejecute el
  // thread que estaba esperando un elemento, ya podría haber un elemento
  // nuevamente.
  // Para que esto suceda, tiene que haber un thread que haga pop(),
  // mientras otro esta en waitData(), mientras otro está ejecutando
  // este push, es decir, al menos 2 consumidores. Si tenemos un único
  // consumidor, esto anda.

  if ( was_empty )
    not_empty_.notify_one();
}

template <typename T>
void concurrent_queue<T>::pop()
{
  bool const is_empty;

  {
    std::unique_lock<std::mutex> lock( mutex_ );
    queue_.pop();
    is_empty = queue_.empty();
  }

  // TODO que pasa si justo acá se inserta un elemento? Es cierto
  // que la cola se vació, pero para cuando se ejecute el thread
  // que estaba esperando que se vacíe, ya podría haber un elemento
  // nuevamente.
  // Para que esto suceda, tiene que haber un thread que haga push(),
  // mientras otro esta en waitEmpty(), mientras otro está ejecutando
  // este pop, es decir, al menos 3 threads. Si tenemos únicamente 2,
  // esto anda.

  if ( is_empty )
    is_empty_.notify_one();
}

template <typename T>
void concurrent_queue<T>::waitData() const
{
  std::unique_lock<std::mutex> lock( mutex_ );
  
  while ( not stop_ and queue_.empty() )
    not_empty_.wait( lock );
}

template <typename T>
bool concurrent_queue<T>::waitAndPop(T& val)
{
  bool is_empty = false;

  {
    std::unique_lock<std::mutex> lock( mutex_ );
    
    while ( not stop_ and queue_.empty() )
      not_empty_.wait( lock );

    if ( stop_ )
      return queue_.empty();

    val = queue_.front();

    queue_.pop();
    is_empty = queue_.empty();
  }

  if ( is_empty )
    is_empty_.notify_one();

  return not is_empty;
}

template <typename T>
void concurrent_queue<T>::waitEmpty() const
{
  std::unique_lock<std::mutex> lock( mutex_ );
  
  while ( not stop_ and not queue_.empty() )
    is_empty_.wait( lock );
}

template <typename T>
void concurrent_queue<T>::stop()
{
  stop_ = true;
  is_empty_.notify_all();
  not_empty_.notify_all();
}

#endif
