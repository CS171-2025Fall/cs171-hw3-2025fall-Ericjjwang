/**
 * @file parallel_utils.h
 * @author ShanghaiTech CS171 TAs
 * @brief Parallel utilities for the renderer. Although we can muddle through
 * with OpenMP, it is still useful and instructive to implement our own parallel
 * utils like OneTBB. Checkout the functions and Classes below for more details.
 * @version 0.1
 * @date 2023-08-05
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __PARALLEL_UTILS_H__
#define __PARALLEL_UTILS_H__

#include <omp.h>

#include "rdr/rdr.h"

RDR_NAMESPACE_BEGIN

template <typename ExecutorType>
class ExecutorInterface {
public:
  /**
   * @brief Dispatch the tasks to the executor. The executor should be able to
   * schedule the tasks at any granularity and pass the corresponding TLS to
   * ensure thread-safety. When this function returns, tasks are not guaranteed
   * to be finished.
   */
  template <typename IndexType, typename TLSType,
      typename FuncType = std::function<void(IndexType &, TLSType &)>>
  void execute(IndexType begin, IndexType end, const FuncType &func) {
    static_cast<ExecutorType &>(*this)
        .template execute<IndexType, TLSType, FuncType>(begin, end, func);
  }

  /**
   * @brief Block and wait for all the tasks to finish.
   */
  void wait_for_all() {
    static_cast<const ExecutorType &>(*this).wait_for_all();
  }

protected:
  ExecutorInterface()          = default;
  virtual ~ExecutorInterface() = default;
};

// class TBBExecutor : public ExecutorInterface {};
class OMPExecutor : public ExecutorInterface<OMPExecutor> {
public:
  /// @see ExecutorInterface::execute
  template <typename IndexType, typename TLSType,
      typename FuncType = std::function<void(IndexType &, TLSType &)>>
  void execute(IndexType begin, IndexType end, const FuncType &func) {
    // TODO: is it too complex?
  }

  /// @see ExecutorInterface::wait_for_all
  void wait_for_all() {}
};

// We do not generalize this function for now because we don't want to increase
// the complexity again.
class ParallelHandler {
public:
  void do_sync() {}  // NOLINT
};

struct ParallelExecutionContext {
  int task_id;
  int thread_id;
};

template <typename T>
static void parallelFor(
    ParallelHandler &handler, int num_threads, const T &func) {
  (void)handler;
#pragma omp parallel for schedule(dynamic)
  for (int task_id = 0; task_id < num_threads; ++task_id) {
    ParallelExecutionContext context{task_id, omp_get_thread_num()};
    func(context);
  }
}

RDR_NAMESPACE_END

#endif
