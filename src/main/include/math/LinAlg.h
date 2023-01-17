#pragma once

#include <math.h>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "math/ConstMath.h"
#include "types.h"

// needed because using the > or >= operator in templates confused the compiler to think it is the end of a template
constexpr bool between(usize n, usize min, usize max) {
    return n >= min && n <= max;
}

// put this macro in front of a method that returns type type to define it only for a
// matrix with a certain number of rows and columns, between the specified min and max (inclusive) for each
// also put modifiers that would normally go in front of the method, for example static, inline etc, in the modifiers field
// if the rows and cols do not match, the method will not be defined, so calling it will be an error
// this is very odd looking because c++ templates are very odd
#define MAT_METHOD(rows_min, rows_max, cols_min, cols_max, modifiers, type) \
    template<usize rr__ = r, usize cc__ = c>                                \
    modifiers typename std::enable_if_t<between(r, rows_min, rows_max) && between(rr__, rows_min, rows_max) && between(c, cols_min, cols_max) && between(cc__, cols_min, cols_max), type>

// this enables a method for a vector of any size
#define VEC_METHOD(modifiers, type) \
    template<usize cc__ = c>        \
    modifiers typename std::enable_if_t<between(c, 1, 1) && between(cc__, 1, 1), type>

// these macros are just for certain sized matrixes and vectors, and do the same as the one above prety much
#define MAT2_METHOD(modifiers, type) MAT_METHOD(2, 2, 2, 2, modifiers, type)
#define MAT3_METHOD(modifiers, type) MAT_METHOD(3, 3, 3, 3, modifiers, type)
#define MAT4_METHOD(modifiers, type) MAT_METHOD(4, 4, 4, 4, modifiers, type)

#define VEC2_METHOD(modifiers, type) MAT_METHOD(2, 2, 1, 1, modifiers, type)
#define VEC3_METHOD(modifiers, type) MAT_METHOD(3, 3, 1, 1, modifiers, type)
#define VEC4_METHOD(modifiers, type) MAT_METHOD(4, 4, 1, 1, modifiers, type)

#define VEC_RANGE_METHOD(min, max, modifiers, type) MAT_METHOD(min, max, 1, 1, modifiers, type)

// define an eccesor mathod on a veector between low and high dimension,
// with name name and accesing data at index index
#define ACCESOR_METHODS(name, index, low, high)      \
    VEC_RANGE_METHOD(low, high, constexpr, T&)       \
    name()& {                                        \
        static_assert(index < size(), "error");      \
        return data[index];                          \
    }                                                \
    VEC_RANGE_METHOD(low, high, constexpr, const T&) \
    name() const& {                                  \
        static_assert(index < size(), "error");      \
        return data[index];                          \
    }                                                \
    VEC_RANGE_METHOD(low, high, constexpr, T)        \
    name()&& {                                       \
        static_assert(index < size(), "error");      \
        return data[index];                          \
    }

// NOTE: this matrix does not use simd so it is not super fast or anything
// TODO: maybe use x and y instead of rows ans columns everywhere because rows adn columns are unintuitive to think about sometimes
template<typename T, usize r, usize c>
class Matrix {
    public:
        using Mtype = Matrix<T, r, c>;

        Matrix() = default;

        constexpr Matrix(T n) noexcept:
        data { n } {}

        // this makes a constructor that takes as many elements as the matrix will hold and initilizes the matrix with these arguments
        // the elements start on the first row, move across all the columns, than go to the second row
        template<typename... Args>
        constexpr Matrix(Args... args) noexcept:
        data { std::forward<Args>(args)... } {
            static_assert(sizeof...(args) == size(), "Incorrect number of arguments for Matrix");
        }

        // creates a 2d counterclockwise rotation matrix given an angle in radians
        MAT2_METHOD(static constexpr, Mtype)
        rotation(num angle) noexcept {
            return Matrix(
                cos(angle), -sin(angle),
                sin(angle), cos(angle));
        }

        constexpr static usize rows() {
            return r;
        }

        constexpr static usize cols() {
            return c;
        }

        constexpr static usize size() {
            return r * c;
        }

        // indexing methods
        // both of these throw index_out_of_range if it is out of range
        // operator[] takes in 1 index and returns an element from the data array without accounting for rows or columns
        T& operator[](usize index) & {
            return data[index_inner(index)];
        }

        const T& operator[](usize index) const& {
            return data[index_inner(index)];
        }

        T operator[](usize index) && {
            return data[index_inner(index)];
        }

        // at returns the element at the given x and y coordinates (x is column y is row)
        T& at(usize x, usize y) & {
            return data[at_inner(x, y)];
        }

        const T& at(usize x, usize y) const& {
            return data[at_inner(x, y)];
        }

        T at(usize x, usize y) && {
            return data[at_inner(x, y)];
        }

        // at_unchecked does not check if the index is in range, so the user must do that
        T& at_unchecked(usize x, usize y) & {
            return data[get_index(x, y)];
        }

        const T& at_unchecked(usize x, usize y) const& {
            return data[get_index(x, y)];
        }

        T at_unchecked(usize x, usize y) && {
            return data[get_index(x, y)];
        }

        ACCESOR_METHODS(x, 0, 1, 4)
        ACCESOR_METHODS(y, 1, 2, 4)
        ACCESOR_METHODS(z, 2, 3, 4)
        ACCESOR_METHODS(w, 3, 4, 4)

        // operator overloads
        // adition does alement wise adding, or adding a constant to every element
        constexpr Mtype& operator+=(const Mtype& other) {
            for (usize i = 0; i < size(); i++) {
                data[i] += other.data[i];
            }
            return *this;
        }

        constexpr Mtype operator+(const Mtype& other) const {
            Mtype out(*this);
            out += other;
            return out;
        }

        constexpr Mtype& operator+=(const T& n) {
            for (usize i = 0; i < size(); i++) {
                data[i] += n;
            }
            return *this;
        }

        constexpr Mtype operator+(const T& n) const {
            Mtype out(*this);
            out += n;
            return out;
        }

        // subtraction does alement wise subtraction, or subtracting a constant from every element
        constexpr Mtype& operator-=(const Mtype& other) {
            for (usize i = 0; i < size(); i++) {
                data[i] -= other.data[i];
            }
            return *this;
        }

        constexpr Mtype operator-(const Mtype& other) const {
            Mtype out(*this);
            out -= other;
            return out;
        }

        constexpr Mtype& operator-=(const T& n) {
            for (usize i = 0; i < size(); i++) {
                data[i] -= n;
            }
            return *this;
        }

        constexpr Mtype operator-(const T& n) const {
            Mtype out(*this);
            out -= n;
            return out;
        }

        // multiplying or dividing by a number multiplies or divides every element in the matrix
        constexpr Mtype& operator*=(const T& n) {
            for (usize i = 0; i < size(); i++) {
                data[i] *= n;
            }
            return *this;
        }

        constexpr Mtype operator*(const T& n) const {
            Mtype out(*this);
            out *= n;
            return out;
        }

        constexpr Mtype& operator/=(const T& n) {
            for (usize i = 0; i < size(); i++) {
                data[i] /= n;
            }
            return *this;
        }

        constexpr Mtype operator/(const T& n) const {
            Mtype out(*this);
            out /= n;
            return out;
        }

        // matrix multiplication
        // this does not use simd or anything, so it is not very fast
        // we shouldn't need super fast matrix multiplication though
        template<usize other_cols>
        constexpr Matrix<T, r, other_cols>& operator*=(const Matrix<T, c, other_cols>& other) {
            // default value of the default constructor, so we can reset the tmp value in the loop
            // apparantly is doesn't reset between loop iterations
            T def;
            // temporary array
            T row_tmp[cols()];
            for (usize row = 0; row < rows(); row++) {
                for (usize col = 0; col < other_cols; col++) {
                    T tmp = def;
                    for (usize i = 0; i < cols(); i++) {
                        // TODO: maybe avoid use of at
                        tmp += at_unchecked(i, row) * other.at_unchecked(col, i);
                    }
                    row_tmp[col] = tmp;
                }

                // copy temporary row to matrix
                for (usize col = 0; col < cols(); col++) {
                    at_unchecked(col, row) = row_tmp[col];
                }
            }
            return *this;
        }

        template<usize other_cols>
        constexpr Matrix<T, r, other_cols> operator*(const Matrix<T, c, other_cols>& other) const {
            // default value of the default constructor, so we can reset the tmp value in the loop
            // apparantly is doesn't reset between loop iterations
            T def;
            Matrix<T, r, other_cols> out;
            for (usize row = 0; row < rows(); row++) {
                for (usize col = 0; col < other_cols; col++) {
                    T tmp = def;
                    // TODO: maybe don't use at if performance is needed
                    for (usize i = 0; i < cols(); i++) {
                        tmp += at_unchecked(i, row) * other.at_unchecked(col, i);
                    }
                    out.at_unchecked(col, row) = tmp;
                }
            }
            return out;
        }

        // normalizes this vector
        VEC_METHOD(, void)
        normalize() {
            (*this) /= magnitude();
        }

        // returns this vector normalized, but does not actually normalize this vector
        VEC_METHOD(, Mtype)
        as_normalized() const {
            return (*this) / magnitude();
        }

        VEC_METHOD(, T)
        magnitude() const {
            return sqrt(magnitude_squared());
        }

        // constexpr versions of the three methods from above
        // they use a different square root, so they might be slower at run time, which is why they are constexpr
        // TODO: figure out if they are actually that much slower
        // normalizes this vector
        VEC_METHOD(constexpr, void)
        const_normalize() {
            (*this) /= const_magnitude();
        }

        // returns this vector normalized, but does not actually normalize this vector
        VEC_METHOD(constexpr, Mtype)
        const_as_normalized() const {
            return (*this) / const_magnitude();
        }

        VEC_METHOD(constexpr, T)
        const_magnitude() const {
            return constSqrt(magnitude_squared());
        }

        VEC_METHOD(constexpr, T)
        magnitude_squared() const {
            T out(0);
            for (usize i = 0; i < size(); i++) {
                out += data[i] * data[i];
            }
            return out;
        }

        // returns the angle from the positive x axis in radians in the range of (-pi, pi)
        VEC2_METHOD(constexpr, num)
        angle() const {
            return atan2(y(), x());
        }

        // returns the vetor normal to this vector and facing towards its right hand side
        VEC2_METHOD(constexpr, Mtype)
        right_normal() const {
            return Mtype(y(), -x());
        }

        // returns the vetor normal to this vector and facing towards its left hand side
        VEC2_METHOD(constexpr, Mtype)
        left_normal() const {
            return Mtype(-y(), x());
        }

    private:
        constexpr static usize get_index(usize x, usize y) {
            return y * cols() + x;
        }

        usize index_inner(usize index) const {
            if (index >= size()) {
                throw std::out_of_range("Mat out of range");
            }
            return index;
        }

        usize at_inner(usize x, usize y) const {
            if (x >= cols() || y >= rows()) {
                throw std::out_of_range("Mat out of range");
            }
            return get_index(x, y);
        }

        T data[size()];
};

using Vec2 = Matrix<num, 2, 1>;
using Vec3 = Matrix<num, 3, 1>;
using Vec4 = Matrix<num, 4, 1>;

using Mat2 = Matrix<num, 2, 2>;
using Mat3 = Matrix<num, 3, 3>;
using Mat4 = Matrix<num, 4, 4>;