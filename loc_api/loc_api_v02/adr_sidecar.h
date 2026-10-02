#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gnss_adr {

struct Sample {
    uint32_t system = 0;
    uint64_t signal = 0;
    uint32_t sv_id = 0;
    uint8_t n_hz = 0;
    uint32_t counter = 0;
    uint64_t valid_mask = 0;
    uint8_t cycle_slip_count = 0;
};

template <typename Record>
struct HistoryState {
    std::mutex mutex;
    bool retired = false;
    std::vector<Record> records;
};

using State = HistoryState<Sample>;

// Registry lifetime is independent of the frozen LocApiV02 object layout.
// A lease keeps the state alive while a modem callback is being processed.
template <typename Record>
class HistoryRegistry {
public:
    using Key = const void*;
    using StateType = HistoryState<Record>;
    using StatePtr = std::shared_ptr<StateType>;

    StatePtr attach(Key key) {
        if (!key) return {};
        auto state = std::make_shared<StateType>();
        std::lock_guard<std::mutex> lock(mMutex);
        if (!mEntries.emplace(key, state).second) {
            return {};
        }
        return state;
    }

    StatePtr lookup(Key key) const {
        std::lock_guard<std::mutex> lock(mMutex);
        auto it = mEntries.find(key);
        return it == mEntries.end() ? StatePtr{} : it->second;
    }

    // Detach only this generation. A reused address cannot remove a newer one.
    void retire(Key key, const StatePtr& expected) {
        if (!expected) return;
        {
            std::lock_guard<std::mutex> state_lock(expected->mutex);
            expected->retired = true;
            expected->records.clear();
        }
        std::lock_guard<std::mutex> lock(mMutex);
        auto it = mEntries.find(key);
        if (it != mEntries.end() && it->second == expected) {
            mEntries.erase(it);
        }
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mMutex);
        return mEntries.size();
    }

private:
    mutable std::mutex mMutex;
    std::unordered_map<Key, StatePtr> mEntries;
};

using Registry = HistoryRegistry<Sample>;

struct UpdateResult {
    bool accepted = false;
    bool found = false;
    bool reset = true;
    bool cycle_slip = false;
};

inline bool same_stream(const Sample& a, uint32_t system, uint64_t signal,
                        uint32_t sv_id, uint8_t n_hz) {
    return a.system == system && a.signal == signal && a.sv_id == sv_id && a.n_hz == n_hz;
}

// Mirrors the legacy implementation's state transition for one measurement.
inline UpdateResult update(State& state, uint32_t system, uint64_t signal, uint32_t sv_id,
                   uint8_t n_hz, uint32_t counter, uint64_t valid_mask,
                   uint8_t cycle_slip_count, uint64_t carrier_phase_valid_bit,
                   uint64_t cycle_slip_valid_bit, uint32_t min_interval,
                   bool* cycle_slip = nullptr) {
    std::lock_guard<std::mutex> lock(state.mutex);
    if (cycle_slip) *cycle_slip = false;
    UpdateResult result;
    if (state.retired) return result;
    result.accepted = true;

    auto it = std::find_if(state.records.begin(), state.records.end(),
                           [&](const Sample& sample) {
                               return same_stream(sample, system, signal, sv_id, n_hz);
                           });
    bool found = it != state.records.end();
    result.found = found;
    result.reset = !(found && (it->valid_mask & carrier_phase_valid_bit) &&
                     min_interval <= 1000 && it->counter == counter - 1);
    if (found && it->valid_mask & carrier_phase_valid_bit &&
        min_interval <= 1000 && it->counter == counter - 1 &&
        (it->valid_mask & cycle_slip_valid_bit) &&
        (valid_mask & cycle_slip_valid_bit) &&
        it->cycle_slip_count != cycle_slip_count) {
        if (cycle_slip) *cycle_slip = true;
        result.cycle_slip = true;
    }

    Sample current{system, signal, sv_id, n_hz, counter, valid_mask, cycle_slip_count};
    if (found) {
        *it = current;
    } else {
        state.records.push_back(current);
    }
    return result;
}

inline void prune(State& state, uint32_t counter, uint8_t n_hz) {
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.retired) return;
    state.records.erase(std::remove_if(state.records.begin(), state.records.end(),
                                       [&](const Sample& sample) {
                                           return sample.counter != counter && sample.n_hz == n_hz;
                                       }),
                       state.records.end());
}

}  // namespace gnss_adr
