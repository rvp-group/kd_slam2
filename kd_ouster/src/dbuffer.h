#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

namespace louster {

// Owned by the single reader thread while active; owned by the writer
// thread from waitSwap() until it calls reset().
class MBuffer {
public:
    explicit MBuffer(size_t capacity);

    // nullptr if full -- caller drops the datagram and warns.
    uint8_t* reserve(size_t size);
    void reset();

    size_t size() const;
    const uint8_t* data() const;

private:
    std::unique_ptr<uint8_t[]> _data;
    size_t _capacity;
    size_t _offset;
};

// Reader thread: getBuffer(). Writer thread: waitSwap(), then reset()
// the buffer once done with it -- DBuffer never resets it for you.
class DBuffer {
public:
    DBuffer(size_t capacity_each, double swap_threshold);

    uint8_t* getBuffer(size_t size);

    // Blocks until a buffer is ready, or returns nullptr once finish()
    // has been called and nothing is left pending. Caller must reset()
    // the returned buffer when done with it.
    MBuffer* waitSwap();

    // Shutdown: call once the reader has stopped, to flush the active
    // buffer. Retry if it returns false (writer hasn't caught up yet).
    bool done();

    // Call once, after done() succeeds, to let waitSwap() return nullptr.
    void finish();

private:
    bool swap();

    std::unique_ptr<MBuffer> _buffers[2];
    int _active;
    size_t _threshold_bytes;
    MBuffer* _pending;
    bool _finished;
    std::mutex _mutex;
    std::condition_variable _cv;
};

}  // namespace louster
