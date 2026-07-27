/******************************************************************************
 * @brief Defines and implements the CommandQueue class: a single-consumer,
 *      many-producer control channel. Foreign threads Post() work that must run
 *      on one owning thread (for example, all mutation of a camera SDK handle);
 *      the owning thread calls DrainAll() once per loop iteration to execute
 *      that work on itself. This is the "commands flow in" half of the active
 *      object pattern and is what lets every SDK call happen on a single thread
 *      without scattering shared_mutexes through the accessors.
 *
 *      Every command that returns a result hands the caller a std::future. The
 *      queue guarantees that future is always satisfied - by execution during
 *      DrainAll(), or by a cancellation exception during Shutdown() / after
 *      shutdown - so a caller waiting on get() is never stranded.
 *
 *      The class is dependency-free (standard library only) so it can be unit
 *      tested in isolation and reused anywhere.
 *
 * @file CommandQueue.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#ifndef COMMAND_QUEUE_HPP
#define COMMAND_QUEUE_HPP

/// \cond
#include <chrono>
#include <exception>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <utility>

/// \endcond

/******************************************************************************
 * @brief A thread-safe queue of deferred commands that all execute on a single
 *      owning thread. Producers Post()/PostWithResult() from any thread; the
 *      owner DrainAll()s them on itself.
 *
 * @note Copying/moving is disabled; hold it as a member of the owning class.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2026-07-24
 ******************************************************************************/
class CommandQueue
{
    private:
        /******************************************************************************
         * @brief One queued unit of work. fnExecute runs the command (and, for
         *      result-bearing commands, fulfills its promise). fnCancel fulfills that
         *      same promise with a cancellation exception if the command is discarded
         *      instead of run. fnCancel is empty for fire-and-forget commands.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        struct QueuedCommand
        {
            public:
                std::function<void()> fnExecute;    // Runs the command.
                std::function<void()> fnCancel;     // Fulfills the result promise with a cancellation error. May be empty.
        };

    public:
        /******************************************************************************
         * @brief Construct a new Command Queue object.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        CommandQueue() = default;

        /******************************************************************************
         * @brief Destroy the Command Queue object. Cancels any command still queued so
         *      that no caller waiting on a result future is left stranded.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        ~CommandQueue() { this->Shutdown(); }

        // A CommandQueue owns synchronization state and must not be copied or moved.
        CommandQueue(const CommandQueue&)            = delete;
        CommandQueue& operator=(const CommandQueue&) = delete;
        CommandQueue(CommandQueue&&)                 = delete;
        CommandQueue& operator=(CommandQueue&&)      = delete;

        /******************************************************************************
         * @brief Set an optional handler invoked when a fire-and-forget command throws
         *      during DrainAll(). Lets the owner log the exception without this
         *      primitive depending on any logging library. The exception is always
         *      swallowed so one bad command cannot kill the owner's loop.
         *
         * @param fnHandler - The handler to invoke with the thrown exception pointer.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        void SetExceptionHandler(std::function<void(std::exception_ptr)> fnHandler)
        {
            // Store the handler under lock so DrainAll() sees a consistent value.
            std::lock_guard<std::mutex> lkQueue(m_muQueue);
            m_fnExceptionHandler = std::move(fnHandler);
        }

        /******************************************************************************
         * @brief Set an optional predicate that reports whether the owning thread is
         *      still alive and able to call DrainAll(). Without it, a command posted
         *      after the owning thread has stopped would sit in the queue forever and
         *      any caller waiting on its result future would block forever. With it,
         *      such a command is cancelled immediately instead.
         *
         * @param fnIsDrainerLive - Predicate returning true while the owning thread is
         *                  running (or starting) and will drain the queue. If unset,
         *                  the drainer is assumed to be live.
         *
         * @note The predicate is invoked without the queue lock held, so it may safely
         *      read the owner's thread state.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        void SetDrainerLivenessCheck(std::function<bool()> fnIsDrainerLive)
        {
            // Store the predicate under lock so posting threads see a consistent value.
            std::lock_guard<std::mutex> lkQueue(m_muQueue);
            m_fnIsDrainerLive = std::move(fnIsDrainerLive);
        }

        /******************************************************************************
         * @brief Check whether commands posted right now could ever run: the queue must
         *      not be shut down and the owning thread must still be able to drain it.
         *
         * @return true - A drainer is expected to run queued commands.
         * @return false - Nothing will drain the queue; posting would strand a caller.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        bool IsDrainerLive() const
        {
            // Copy the predicate out under lock so we can invoke it unlocked.
            std::function<bool()> fnIsDrainerLive;
            {
                // Lock to read the shutdown flag and the predicate together.
                std::lock_guard<std::mutex> lkQueue(m_muQueue);
                // A shut down queue is never live.
                if (m_bShutdown)
                {
                    // Nothing will ever drain again.
                    return false;
                }
                // Take a copy of the predicate to call outside the lock.
                fnIsDrainerLive = m_fnIsDrainerLive;
            }

            // With no predicate configured, assume a drainer is present.
            return fnIsDrainerLive ? fnIsDrainerLive() : true;
        }

        /******************************************************************************
         * @brief Post a fire-and-forget command to run on the owning thread.
         *
         * @param fnCommand - The command to execute during the next DrainAll().
         *
         * @note If the queue is shut down, or the owning thread is no longer able to
         *      drain it, the command is dropped.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        void Post(std::function<void()> fnCommand)
        {
            // Check liveness before taking the lock; the predicate reads owner state.
            if (!this->IsDrainerLive())
            {
                // Nothing will ever run this command; drop it rather than queue it forever.
                return;
            }

            // Lock the queue to check shutdown state and enqueue atomically.
            std::lock_guard<std::mutex> lkQueue(m_muQueue);
            // Drop the command if we are shutting down; there is no owner left to run it.
            if (m_bShutdown)
            {
                // Nothing to fulfill for a fire-and-forget command; simply drop it.
                return;
            }
            // Enqueue the command with no cancel action (nothing waits on it).
            m_qCommands.push(QueuedCommand{std::move(fnCommand), std::function<void()>{}});
        }

        /******************************************************************************
         * @brief Post a command that returns a result, and get a future for that
         *      result. The command runs on the owning thread during DrainAll(); the
         *      returned future is fulfilled with the result, with any exception the
         *      command throws, or with a cancellation error if the queue shuts down
         *      before the command runs.
         *
         * @tparam R - The return type of the command.
         * @param fnCommand - The command to execute.
         * @return std::future<R> - A future for the command's result.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        template<typename R>
        std::future<R> PostWithResult(std::function<R()> fnCommand)
        {
            // Create a shared promise so both the execute and cancel paths can fulfill it.
            std::shared_ptr<std::promise<R>> pmResult = std::make_shared<std::promise<R>>();
            std::future<R> fuResult                   = pmResult->get_future();

            // Build the execute action: run the command and fulfill the promise.
            std::function<void()> fnExecute = [pmResult, fnCommand = std::move(fnCommand)]()
            {
                try
                {
                    // Invoke the command and store its result (handling the void case).
                    if constexpr (std::is_void_v<R>)
                    {
                        // Void command: run it, then signal completion.
                        fnCommand();
                        pmResult->set_value();
                    }
                    else
                    {
                        // Value command: store the returned value.
                        pmResult->set_value(fnCommand());
                    }
                }
                catch (...)
                {
                    // Forward any exception to the waiting caller.
                    try
                    {
                        // set_exception can itself throw if the promise is already satisfied; guard it.
                        pmResult->set_exception(std::current_exception());
                    }
                    catch (...)
                    {
                    }
                }
            };

            // Build the cancel action: fulfill the promise with a cancellation error.
            std::function<void()> fnCancel = [pmResult]()
            {
                try
                {
                    // Signal that the command was discarded before it could run.
                    pmResult->set_exception(std::make_exception_ptr(std::runtime_error("CommandQueue shut down before command executed.")));
                }
                catch (...)
                {
                }
            };

            // Check liveness before taking the lock; the predicate reads owner state.
            const bool bDrainerLive = this->IsDrainerLive();

            // Enqueue under lock, honoring shutdown state.
            {
                // Lock the queue to check shutdown state and enqueue atomically.
                std::lock_guard<std::mutex> lkQueue(m_muQueue);
                // If not shutting down and a drainer is alive, enqueue for the owner to run.
                if (!m_bShutdown && bDrainerLive)
                {
                    // Enqueue with both the execute and cancel actions.
                    m_qCommands.push(QueuedCommand{std::move(fnExecute), std::move(fnCancel)});
                    // Return the future to the caller.
                    return fuResult;
                }
            }

            // Shut down or no drainer: cancel immediately so the caller is never stranded.
            fnCancel();
            return fuResult;
        }

        /******************************************************************************
         * @brief Post a command and block until it has run on the owning thread. Unlike
         *      waiting on PostWithResult()'s future directly, this re-checks drainer
         *      liveness while it waits, so an owning thread that stops AFTER the command
         *      was queued releases the caller with a cancellation error instead of
         *      leaving it blocked forever.
         *
         * @tparam R - The return type of the command.
         * @param fnCommand - The command to execute.
         * @param tmPollInterval - How often to re-check drainer liveness while waiting.
         * @return std::future<R> - A ready future holding the result, the command's
         *                  exception, or a cancellation error. Never returns unsatisfied.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        template<typename R>
        std::future<R> PostAndWait(std::function<R()> fnCommand, const std::chrono::milliseconds tmPollInterval = std::chrono::milliseconds(25))
        {
            // Post the command. This already cancels immediately if the drainer is known dead.
            std::future<R> fuResult = this->PostWithResult<R>(std::move(fnCommand));

            // Wait for the owning thread to run it, re-checking liveness on every tick.
            while (fuResult.wait_for(tmPollInterval) == std::future_status::timeout)
            {
                // If the owning thread can no longer drain, nothing will ever run this
                // command, so cancel everything still queued (including ours) and stop waiting.
                if (!this->IsDrainerLive())
                {
                    // Release this caller and any other stranded ones.
                    this->CancelPending();
                }
            }

            // Return the now-satisfied future.
            return fuResult;
        }

        /******************************************************************************
         * @brief Cancel every command currently queued, fulfilling their result futures
         *      with a cancellation error, without marking the queue shut down. Used when
         *      the owning thread has stopped but the object may still be restarted.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        void CancelPending()
        {
            // Take the queued commands under lock so we can cancel them unlocked.
            std::queue<QueuedCommand> qBatch;
            {
                // Lock only long enough to take ownership of the current commands.
                std::lock_guard<std::mutex> lkQueue(m_muQueue);
                std::swap(qBatch, m_qCommands);
            }

            // Cancel each command so its waiting caller (if any) is released.
            while (!qBatch.empty())
            {
                // Take the next command.
                QueuedCommand stCommand = std::move(qBatch.front());
                qBatch.pop();
                // Fulfill the result promise with a cancellation error if there is one.
                if (stCommand.fnCancel)
                {
                    // Release any caller waiting on this command's future.
                    stCommand.fnCancel();
                }
            }
        }

        /******************************************************************************
         * @brief Execute every currently-queued command on the calling thread. Call
         *      this once at the top of the owning thread's loop. Commands run outside
         *      the queue lock (so a command may safely Post() more work), and an
         *      exception from any one command is caught so it cannot kill the loop.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        void DrainAll()
        {
            // Swap the queued commands into a local batch so we can run them unlocked.
            std::queue<QueuedCommand> qBatch;
            std::function<void(std::exception_ptr)> fnExceptionHandler;
            {
                // Lock only long enough to take ownership of the current commands.
                std::lock_guard<std::mutex> lkQueue(m_muQueue);
                std::swap(qBatch, m_qCommands);
                // Snapshot the exception handler under the same lock.
                fnExceptionHandler = m_fnExceptionHandler;
            }

            // Run each command outside the lock.
            while (!qBatch.empty())
            {
                // Take the next command.
                QueuedCommand stCommand = std::move(qBatch.front());
                qBatch.pop();
                try
                {
                    // Execute the command. Result-bearing commands route their own errors to
                    // their promise; only fire-and-forget commands can throw out of here.
                    stCommand.fnExecute();
                }
                catch (...)
                {
                    // Never let one command kill the loop. Hand the error to the owner if it
                    // registered a handler, otherwise swallow it.
                    if (fnExceptionHandler)
                    {
                        // Report the exception without rethrowing.
                        fnExceptionHandler(std::current_exception());
                    }
                }
            }
        }

        /******************************************************************************
         * @brief Mark the queue shut down and cancel every command still queued,
         *      fulfilling their result futures with a cancellation error. After this,
         *      further Post()/PostWithResult() calls are dropped/cancelled immediately.
         *      Idempotent and safe to call from any thread.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        void Shutdown()
        {
            // Flip the shutdown flag so all further posts are rejected immediately.
            {
                // Lock only long enough to set the flag.
                std::lock_guard<std::mutex> lkQueue(m_muQueue);
                m_bShutdown = true;
            }

            // Cancel everything still queued so no waiting caller is left stranded.
            this->CancelPending();
        }

        /******************************************************************************
         * @brief Accessor for the number of commands currently queued.
         *
         * @return size_t - The number of pending commands.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        size_t GetPendingCount() const
        {
            // Lock to read the queue size consistently.
            std::lock_guard<std::mutex> lkQueue(m_muQueue);
            return m_qCommands.size();
        }

        /******************************************************************************
         * @brief Accessor for whether the queue has been shut down.
         *
         * @return true - The queue is shut down and rejects new commands.
         * @return false - The queue is accepting commands.
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2026-07-24
         ******************************************************************************/
        bool IsShutdown() const
        {
            // Lock to read the shutdown flag consistently.
            std::lock_guard<std::mutex> lkQueue(m_muQueue);
            return m_bShutdown;
        }

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        mutable std::mutex m_muQueue;                                    // Guards the command queue, shutdown flag, and callbacks.
        std::queue<QueuedCommand> m_qCommands;                           // Pending commands awaiting DrainAll().
        bool m_bShutdown = false;                                        // Once true, new commands are dropped/cancelled.
        std::function<void(std::exception_ptr)> m_fnExceptionHandler;    // Optional handler for fire-and-forget command exceptions.
        std::function<bool()> m_fnIsDrainerLive;                         // Optional predicate: is the owning thread still draining?
};

#endif    // COMMAND_QUEUE_HPP
