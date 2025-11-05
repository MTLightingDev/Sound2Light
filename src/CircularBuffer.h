// Simple header-only C++17 circular buffer compatible with the subset of
// Qt3DCore::QCircularBuffer API used by Sound2Light.
//
// This avoids any dependency on Qt3D/QtCore in this project while keeping the
// existing type name and namespace so no call sites need to change.
//
// Provided operations (as used by this codebase):
//  - constructors: default, explicit capacity
//  - size(), count(), capacity(), isEmpty(), clear()
//  - push_back(const T&), push_back(T&&)
//  - operator[](int), operator[](int) const, at(int) const
//  - last(), last() const
//
// Semantics:
//  - Fixed capacity ring buffer. New elements are appended at the logical end.
//  - When full, push_back overwrites the oldest element (drops it) and advances
//    the head, keeping size()==capacity().
//  - Index 0 refers to the oldest element; index size()-1 is the most recent.
//
// Note: This is a minimal implementation tailored to the project’s needs. Extend
// it if you require more of the original Qt3D API.

#ifndef SOUND2LIGHT_CIRCULARBUFFER_H
#define SOUND2LIGHT_CIRCULARBUFFER_H

#include <vector>
#include <stdexcept>
#include <cassert>
#include <type_traits>

namespace Qt3DCore {

template <typename T>
class QCircularBuffer {
public:
    // Types
    using value_type = T;

    // Constructors
    QCircularBuffer() noexcept : m_capacity(0), m_size(0), m_head(0) {}

    explicit QCircularBuffer(int capacity)
        : m_capacity(capacity > 0 ? capacity : 0),
          m_data(static_cast<std::size_t>(m_capacity)),
          m_size(0),
          m_head(0) {}

    // Capacity/size
    int capacity() const noexcept { return m_capacity; }
    int size() const noexcept { return m_size; }
    int count() const noexcept { return m_size; }
    bool isEmpty() const noexcept { return m_size == 0; }

    // Modifiers
    void clear() noexcept {
        // Keep allocated storage; just reset logical state
        m_size = 0;
        m_head = 0;
    }

    // Qt3D API used in this project aliases append() to push_back()
    void append(const T& value) { push_back(value); }
    void append(T&& value) { push_back(std::move(value)); }

    void push_back(const T& value) {
        ensureStorage();
        if (m_capacity == 0) return; // nothing to store
        const int idx = physicalIndexForAppend();
        m_data[static_cast<std::size_t>(idx)] = value;
        advanceAfterAppend();
    }

    void push_back(T&& value) {
        ensureStorage();
        if (m_capacity == 0) return;
        const int idx = physicalIndexForAppend();
        m_data[static_cast<std::size_t>(idx)] = std::move(value);
        advanceAfterAppend();
    }

    // Element access
    const T& at(int i) const {
        checkIndex(i);
        return m_data[static_cast<std::size_t>(physicalIndex(i))];
    }

    T& operator[](int i) {
        checkIndex(i);
        return m_data[static_cast<std::size_t>(physicalIndex(i))];
    }

    const T& operator[](int i) const {
        checkIndex(i);
        return m_data[static_cast<std::size_t>(physicalIndex(i))];
    }

    T& last() {
        checkNotEmpty();
        return m_data[static_cast<std::size_t>(physicalIndex(m_size - 1))];
    }

    const T& last() const {
        checkNotEmpty();
        return m_data[static_cast<std::size_t>(physicalIndex(m_size - 1))];
    }

private:
    // Storage state
    int m_capacity;
    std::vector<T> m_data;  // fixed-size storage once constructed
    int m_size;             // number of valid elements (<= capacity)
    int m_head;             // physical index of logical element 0 (oldest)

    // Helpers
    void ensureStorage() {
        if (m_capacity != static_cast<int>(m_data.size())) {
            // Keep invariants if someone default-constructed without capacity
            m_data.resize(static_cast<std::size_t>(m_capacity));
        }
    }

    int physicalIndex(int logicalIndex) const noexcept {
        // 0 maps to m_head (oldest), size()-1 maps to last pushed
        const int base = m_head + logicalIndex;
        return (m_capacity == 0) ? 0 : (base >= m_capacity ? base - m_capacity : base);
    }

    int physicalIndexForAppend() const noexcept {
        // Where the next element will be written
        return physicalIndex(m_size == 0 ? 0 : m_size);
    }

    void advanceAfterAppend() noexcept {
        if (m_size < m_capacity) {
            ++m_size;
        } else if (m_capacity > 0) {
            // Overwrite oldest: move head forward (drop oldest)
            m_head = (m_head + 1);
            if (m_head == m_capacity) m_head = 0;
        }
    }

    void checkIndex(int i) const {
        assert(i >= 0 && i < m_size);
#ifndef NDEBUG
        if (i < 0 || i >= m_size) {
            throw std::out_of_range("QCircularBuffer index out of range");
        }
#endif
    }

    void checkNotEmpty() const {
        assert(m_size > 0);
#ifndef NDEBUG
        if (m_size == 0) {
            throw std::out_of_range("QCircularBuffer::last() on empty buffer");
        }
#endif
    }
};

} // namespace Qt3DCore

#endif // SOUND2LIGHT_CIRCULARBUFFER_H
