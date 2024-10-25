#pragma once
#include <array>
#include <vector>

template <std::size_t dim = 0, class S>
auto compute_offset(const S&) noexcept {
    return typename S::value_type(0);
}

template <std::size_t dim = 0, class S, class I, class... Args>
auto compute_offset(const S& strides, I i, Args... args) noexcept {
    return i*strides[dim] + compute_offset<dim + 1>(strides, args...);
}

template<typename T, std::size_t N>
class mdarray {

    using size_type = std::size_t;
    using value_type = T;
    using reference = T&;
    using const_reference = T const&;
    using pointer = T*;
    using shape_type = std::array<size_type, N>;
    using stride_type = std::array<size_type, N>;

    private:
        size_type _size{0};
        shape_type _shape;
        stride_type _strides;
        std::vector<T> _storage;

    public:
        template<typename ... I>
        explicit mdarray(I ... i) : _shape{static_cast<size_type>(i) ...} {
            static_assert(sizeof...(I) == N, "wrong number of dimensions");
            size_type data_size = 1;
            for (size_type size = _shape.size(); size != 0; size--) {
                _strides[size-1] = data_size;
                data_size = _strides[size-1] * _shape[size-1];
            }

            _size = data_size;
            _storage = std::vector<T>(_size); 
        }

        const size_type size() const {
            return _size;
        }

        const size_type rank() const {
            return N;
        }

        const size_type dimension(size_type i) const {
            return _shape[i];
        }

        pointer data() {
            return _storage.data();
        }

        auto begin() {
            return _storage.begin();
        }

        auto end() {
            return _storage.end();
        }

        auto cbegin() const {
            return _storage.cbegin();
        }

        auto cend() const {
            return _storage.cend();
        }

        template<typename ...I>
        reference operator()(I ... i) {
            static_assert(sizeof...(I) == N, "wrong number of indices");
            auto offset = compute_offset(_strides, static_cast<size_type>(i) ...);
            return _storage[offset];
        }

        template<typename ...I>
        const_reference operator()(I ... i) const {
            static_assert(sizeof...(I) == N, "wrong number of indices");
            auto offset = compute_offset(_strides, static_cast<size_type>(i) ...);
            return _storage[offset];
        }
};