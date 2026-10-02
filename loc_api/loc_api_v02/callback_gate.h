#pragma once

#include <condition_variable>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace gnss_adr {

// A callback cookie must outlive the modem client's close operation.  This
// gate lets a callback acquire the current owner without dereferencing a
// LocApiV02 pointer after destruction has started.  The owner is cleared
// before waiting for callbacks, so a late callback is harmless.
template <typename Owner>
class CallbackGate {
    struct State {
        std::mutex mutex;
        std::condition_variable drained;
        Owner* owner = nullptr;
        std::size_t in_flight = 0;
        std::unordered_map<std::thread::id, std::size_t> threads;
        bool closing = false;
    };

public:
    class Lease {
    public:
        Lease() = default;
        Lease(const Lease&) = delete;
        Lease& operator=(const Lease&) = delete;
        Lease(Lease&& other) noexcept : mState(other.mState), mOwner(other.mOwner),
                mThread(other.mThread) {
            other.mState = nullptr;
            other.mOwner = nullptr;
        }
        Lease& operator=(Lease&& other) noexcept {
            if (this != &other) {
                release();
                mState = other.mState;
                mOwner = other.mOwner;
                mThread = other.mThread;
                other.mState = nullptr;
                other.mOwner = nullptr;
            }
            return *this;
        }
        ~Lease() { release(); }
        Owner* get() const { return mOwner; }
        explicit operator bool() const { return mOwner != nullptr; }

    private:
        friend class CallbackGate;
        Lease(State* state, Owner* owner) : mState(state), mOwner(owner),
                mThread(std::this_thread::get_id()) {}
        void release() {
            if (!mState) return;
            std::unique_lock<std::mutex> lock(mState->mutex);
            auto thread = mState->threads.find(mThread);
            if (--thread->second == 0) mState->threads.erase(thread);
            if (--mState->in_flight == 0) mState->drained.notify_all();
            mState = nullptr;
            mOwner = nullptr;
        }
        State* mState = nullptr;
        Owner* mOwner = nullptr;
        std::thread::id mThread;
    };

    explicit CallbackGate(Owner* owner) : mState(new State) {
        mState->owner = owner;
    }
    CallbackGate(const CallbackGate&) = delete;
    CallbackGate& operator=(const CallbackGate&) = delete;
    ~CallbackGate() {
        closeAndWait();
        delete mState;
    }

    Lease enter() {
        std::lock_guard<std::mutex> lock(mState->mutex);
        if (mState->closing || !mState->owner) return {};
        ++mState->in_flight;
        ++mState->threads[std::this_thread::get_id()];
        return Lease(mState, mState->owner);
    }

    void requestMeasurementReset() { mReset.store(true, std::memory_order_release); }
    bool consumeMeasurementReset() {
        return mReset.exchange(false, std::memory_order_acq_rel);
    }
    bool isClosed() const {
        std::lock_guard<std::mutex> lock(mState->mutex);
        return mState->closing;
    }

    // Must run before freeing the owner. It is idempotent and leaves late
    // callbacks with an empty lease. False means the caller holds a callback
    // lease and must queue the operation to its message task. Leases are not
    // transferable callback execution contexts: destroy them on entry thread.
    bool closeAndWait() {
        std::unique_lock<std::mutex> lock(mState->mutex);
        // Closing synchronously from our own callback would wait for itself.
        // The caller must queue close onto its owning message task instead.
        if (mState->threads.count(std::this_thread::get_id())) return false;
        mState->closing = true;
        mState->owner = nullptr;
        mState->drained.wait(lock, [this] { return mState->in_flight == 0; });
        return true;
    }

private:
    State* mState;
    std::atomic<bool> mReset{false};
};

}  // namespace gnss_adr
