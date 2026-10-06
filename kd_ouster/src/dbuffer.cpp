#include "dbuffer.h"

namespace louster {

MBuffer::MBuffer(size_t capacity)
    : _data(std::make_unique<uint8_t[]>(capacity)), _capacity(capacity), _offset(0) {}

uint8_t* MBuffer::reserve(size_t size) {
    if (_offset + size > _capacity) return nullptr;
    uint8_t* ptr = _data.get() + _offset;
    _offset += size;
    return ptr;
}

void MBuffer::reset() { _offset = 0; }

size_t MBuffer::size() const { return _offset; }
const uint8_t* MBuffer::data() const { return _data.get(); }

DBuffer::DBuffer(size_t capacity_each, double swap_threshold)
    : _buffers{std::make_unique<MBuffer>(capacity_each),
               std::make_unique<MBuffer>(capacity_each)},
      _active(0),
      _threshold_bytes(static_cast<size_t>(capacity_each * swap_threshold)),
      _pending(nullptr),
      _finished(false) {}

uint8_t* DBuffer::getBuffer(size_t size) {
    MBuffer* target = _buffers[_active].get();
    if (target->size() >= _threshold_bytes) {
        swap();  // no-op if the writer hasn't picked up the last one yet
        target = _buffers[_active].get();
    }
    return target->reserve(size);
}

MBuffer* DBuffer::waitSwap() {
    std::unique_lock<std::mutex> lock(_mutex);
    _cv.wait(lock, [this] { return _pending != nullptr || _finished; });
    if (_pending == nullptr) return nullptr;
    MBuffer* frozen = _pending;
    _pending = nullptr;
    return frozen;
}

bool DBuffer::done() { return swap(); }

void DBuffer::finish() {
    std::lock_guard<std::mutex> lock(_mutex);
    _finished = true;
    _cv.notify_one();
}

bool DBuffer::swap() {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_pending != nullptr) return false;
    MBuffer* frozen = _buffers[_active].get();
    _active = 1 - _active;
    _pending = frozen;
    _cv.notify_one();
    return true;
}

}  // namespace louster
