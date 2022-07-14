/**
 * Circular buffer model header.
 * \author anonymity, trantuan-20048607
 * \date 2022.1.28
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <mutex>
#include "lang-feature-extension/disable-constructors.h"

/**
 * \brief Circular buffer with mutex.
 * \details Refer to https://en.wikipedia.org/wiki/Circular_buffer.
 * \tparam T Type of elements in this buffer.
 * \tparam size Max size of this buffer.
 * \attention Size must be 2^N.
 */
template<typename T, unsigned int size>
class CircularBuffer : NO_COPY, NO_MOVE {
private:
    T data_[size];                              ///< Data array.
    unsigned int head_;                         ///< Head pointer.
    unsigned int tail_;                         ///< Tail pointer.
    std::mutex lock_[size];                     ///< Mutex lock for data.
    std::mutex head_lock_;                      ///< Mutex lock for head pointer.
    const unsigned int and_to_mod_ = size - 1;  ///< Use bitwise operation to accelerate modulus.

public:
    CircularBuffer<T, size>() : head_(0), tail_(0) {
        static_assert(!(size & (size - 1)));
    }

    ~CircularBuffer() = default;

    /**
     * \return Size of this buffer, which is specified when it is constructed.
     */
    [[maybe_unused]] [[nodiscard]] inline unsigned int Size() const { return size; }

    /**
     * \brief Is this buffer empty?
     * \return Whether buffer is empty.
     */
    [[maybe_unused]] [[nodiscard]] inline bool Empty() const { return head_ == tail_; }

    /**
     * \brief Push an element.
     * \param [in] obj Input element.
     */
    inline void Push(const T &obj) {
        std::lock_guard<std::mutex> lock(lock_[tail_]);
        data_[tail_] = obj;
        ++tail_;
        tail_ &= and_to_mod_;

        if (head_ == tail_) {
            std::lock_guard<std::mutex> head_lock(head_lock_);
            ++head_;
            head_ &= and_to_mod_;
        }
    }

    /**
     * \brief Pop an element.
     * \param [out] obj Output element.
     * \return Whether buffer is not empty.
     */
    inline bool Pop(T &obj) {
        if (head_ == tail_)
            return false;
        std::lock_guard<std::mutex> lock(lock_[head_]);
        obj = data_[head_];
        std::lock_guard<std::mutex> head_lock(head_lock_);
        ++head_;
        head_ &= and_to_mod_;
        return true;
    }

    [[maybe_unused]] [[nodiscard]] const T &operator[](unsigned int id) {
        while (tail_ + id < 0) id += size;
        return data_[(tail_ + id) & and_to_mod_];
    }
};

#endif  // BUFFER_H_
