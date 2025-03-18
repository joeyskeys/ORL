#pragma once

#include <array>
#include <cmath>
#include <type_traits>
#include <numbers>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORL
{

template <typename T, uint32_t N>
class Vec : public Eigen::Matrix<T, N, 1> {
public:
    using ScalarType = T;
    using Base = Eigen::Matrix<T, N, 1>;
    static constexpr uint32_t size = N;

    Vec(T v=static_cast<T>(0)) {
        Base::setConstant(v);
    }

    template <typename ...Ts, typename = std::enable_if_t<sizeof...(Ts) == N && (... && std::is_arithmetic_v<Ts>)>>
    Vec(Ts... args) : Base(static_cast<T>(args)...) {}

    template <typename Derived>
    Vec(const Eigen::MatrixBase<Derived>& p) : Base(p) {}

    template <typename Derived>
    inline Vec& operator =(const Eigen::MatrixBase<Derived>& p) {
        this->Base::operator =(p);
        return *this;
    }

    auto& operator [](const uint32_t idx) {
        return this->coeffRef(idx, 0);
    }

    auto operator [](const uint32_t idx) const {
        return this->coeffRef(idx, 0);
    }

    inline Vec<T, N> operator *(const T s) const {
        auto ret = this->array() * s;
        return ret.matrix();
    }

    template <typename Derived>
    inline Vec<T, N> operator *(const Eigen::MatrixBase<Derived>& v) const {
        auto ret = this->array() * v.array();
        return ret.matrix();
    }

    inline Vec<T, N>& operator *=(const Vec<T, N>& v) {
        *this = (this->array() * v.array()).matrix();
        return *this;
    }

    template <typename S, typename = std::enable_if_t<std::is_arithmetic_v<S>>>
    inline Vec<T, N> operator /=(const S s) {
        return Base::operator /=(static_cast<Base::ScalarType>(s));
    }

    template <typename Derived>
    inline Vec<T, N> operator /=(const Eigen::MatrixBase<Derived>& v) {
        *this = (this->array() / v.array()).matrix();
        return *this;
    }
};

// Operators
template <typename T, typename S, uint32_t N, typename = std::enable_if_t<std::is_arithmetic_v<S>>>
Vec<T, N> operator *(const S s, const Vec<T, N>& v) {
    return v * static_cast<T>(s);
}

template <typename S, typename Derived, typename = std::enable_if_t<std::is_arithmetic_v<S>>>
Vec<typename Eigen::MatrixBase<Derived>::ScalarType, Eigen::MatrixBase<Derived>::RowsAtCompileTime> operator /(const S s, const Eigen::MatrixBase<Derived>& rhs) {
    return rhs.cwiseInverse() * static_cast<typename Eigen::MatrixBase<Derived>::ScalarType>(s);
}

// Construct funcs
template <typename Derived>
inline auto concat(const Eigen::MatrixBase<Derived>& v, const typename Eigen::MatrixBase<Derived>::ScalarType& s) {
    Eigen::Matrix<typename Eigen::MatrixBase<Derived>::ScalarType, Eigen::MatrixBase<Derived>::RowsAtCompileTime + 1, 1> ret;
    ret << v, s;
    return ret;
}

template <typename Derived1, typename Derived2>
inline auto concat(const Eigen::MatrixBase<Derived1>& v1, const Eigen::MatrixBase<Derived2>& v2) {
    Eigen::Matrix<typename Eigen::MatrixBase<Derived1>::ScalarType, Eigen::MatrixBase<Derived1>::RowsAtCompileTime +
        Eigen::MatrixBase<Derived2>::RowsAtCompileTime, 1> ret;
    ret << v1, v2;
    return ret;
}

template <uint32_t M, typename Derived>
inline auto head(const Eigen::MatrixBase<Derived>& v) {
    return v.head(M);
}

// Horizontal funcs
template <typename T, uint32_t N>
inline T max_component(const Vec<T, N>& v) {
    return v.maxCoeff();
}

template <typename T, uint32_t N>
inline T min_component(const Vec<T, N>& v) {
    return v.minCoeff();
}

template <typename T, uint32_t N>
inline T sum(const Vec<T, N>& v) {
    return v.sum();
}

template <typename Derived>
inline Eigen::MatrixBase<Derived>::ScalarType length(const Eigen::MatrixBase<Derived>& v) {
    return v.norm();
}

template <typename Derived>
inline Eigen::MatrixBase<Derived>::ScalarType length_squared(const Eigen::MatrixBase<Derived>& v) {
    return v.squaredNorm();
}

// Dot & cross
template <typename Derived1, typename Derived2>
inline auto dot(const Eigen::MatrixBase<Derived1>& v1, const Eigen::MatrixBase<Derived2>& v2) {
    return v1.dot(v2);
}

template <typename Derived>
inline auto cross(const Eigen::MatrixBase<Derived>& v1, const Eigen::MatrixBase<Derived>& v2) {
    return v1.cross(v2);
}

template <typename Derived>
inline auto project(const Eigen::MatrixBase<Derived>& v, const Eigen::MatrixBase<Derived>& n) {
    v - v.dot(n) * n;
}

// Normalize
template <typename Derived>
inline auto normalize(const Eigen::MatrixBase<Derived>& v) {
    return v.normalized();
}

// Misc
template <typename T, uint32_t N>
inline Vec<T, N> abs(const Vec<T, N>& v) {
    return v.cwiseAbs();
}

template <typename T, uint32_t N>
inline Vec<T, N> square(const Vec<T, N>& v) {
    return v * v;
}

template <typename T, uint32_t N>
inline Vec<T, N> sqrt(const Vec<T, N>& v) {
    return v.sqrt();
}

template <typename T, uint32_t N>
inline Vec<T, N> exp(const Vec<T, N>& v) {
    return v.exp();
}

template <typename T, uint32_t N>
inline Vec<T, N> pow(const Vec<T, N>& v, const T beta) {
    return v.pow(beta);
}

template <typename T, uint32_t N>
inline Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, const T t) {
    return a * (static_cast<T>(1) - t) + b * t;
}

template <typename T, uint32_t N>
inline Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, const Vec<T, N>& t) {
    return a * (Vec<T, N>{1} - t) + b * t;
}

// Comparison
template <typename Derived>
inline bool is_zero(const Eigen::MatrixBase<Derived>& v,
    const typename Eigen::MatrixBase<Derived>::ScalarType& epsilon=1e-5)
{
    return v.isZero(epsilon);
}

template <typename T, uint32_t N>
inline Vec<T, N> vec_min(const Vec<T, N>& a, const Vec<T, N>& b) {
    return a.cwiseMin(b);
}

template <typename T, uint32_t N>
inline Vec<T, N> vec_max(const Vec<T, N>& a, const Vec<T, N>& b) {
    return a.cwiseMax(b);
}

inline auto degree_to_radians = []<typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>(T d) {
    return d / 180.f * std::numbers::pi_v<float>;
};

inline auto radians_to_degree = []<typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>(T r) {
    return r / std::numbers::pi_v<float> *180.f;
};

template <typename T, uint32_t N>
class Mat : public Eigen::Matrix<T, N, N> {
public:
    using ScalarType = T;
    using Base = Eigen::Matrix<T, N, N>;
    static constexpr uint32_t Size = N;

    Mat() {
        *this = Mat<T, N>::Identity(N, N);
    }

    template <typename T1, typename ...Ts, typename = std::enable_if_t<
        (sizeof...(Ts) + 1 == N * N && (... && std::is_arithmetic_v<Ts>))>>
    Mat(T1 arg1, Ts... args) {
        Eigen::CommaInitializer comma_initializer(*this, arg1);
        ((comma_initializer , args), ...);
    }

    template <typename ...Ts, typename = std::enable_if_t<
        (sizeof...(Ts) == N * N && (... && std::is_arithmetic_v<Ts>))>>
    Mat(Ts... args) : Base(args...) {}

    template <typename Derived>
    Mat(const Eigen::MatrixBase<Derived>& v1,
        const Eigen::MatrixBase<Derived>& v2)
    {
        static_assert(N == 2);
        *this << v1, v2;
    }

    template <typename Derived>
    Mat(const Eigen::MatrixBase<Derived>& v1,
        const Eigen::MatrixBase<Derived>& v2,
        const Eigen::MatrixBase<Derived>& v3)
    {
        static_assert(N == 3);
        *this << v1, v2, v3;
    }

    template <typename Derived>
    Mat(const Eigen::MatrixBase<Derived>& v1,
        const Eigen::MatrixBase<Derived>& v2,
        const Eigen::MatrixBase<Derived>& v3,
        const Eigen::MatrixBase<Derived>& v4)
    {
        static_assert(N == 4);
        *this << v1, v2, v3, v4;
    }

    template <typename Derived>
    Mat(const Eigen::MatrixBase<Derived>& p) : Base(p) {}

    template <typename Derived>
    inline Mat &operator =(const Eigen::MatrixBase<Derived>& p) {
        this->Base::operator=(p);
        return *this;
    }

    auto operator [](const uint32_t idx) {
        return Eigen::Ref<Eigen::Matrix<T, N, 1>>(this->col(idx));
    }

    const Vec<T, N> operator [](const uint32_t idx) const {
        return this->col(idx);
    }
};

template <typename T, uint32_t N>
inline Mat<T, N> identity() {
    return Mat<T, N>::Identity(N, N);
}

template <typename T, uint32_t N>
inline Mat<T, N> transpose(const Mat<T, N>& m) {
    return m.transpose();
}

template <typename T, uint32_t N>
inline Mat<T, N> inverse(const Mat<T, N>& m) {
    return m.inverse();
}

template <typename T, uint32_t N>
inline Mat<T, N> translate(const Eigen::Matrix<T, N - 1, 1>& v) {
    Eigen::Transform<T, N - 1, Eigen::Affine> t;
    t.setIdentity();
    t.pretranslate(v);
    return t.matrix();
}

const auto translate3f = translate<float, 4>;

template <typename T, uint32_t N>
inline Mat<T, N> rotate(const Vec<T, N - 1>& axis, const T& angle) {
    Eigen::Transform<T, N - 1, Eigen::Affine> t;
    t.setIdentity();
    t.rotate(Eigen::AngleAxis(degree_to_radians(angle), axis));
    return t.matrix();
}

const auto rotate3f = rotate<float, 4>;

template <typename T, uint32_t N>
inline Mat<T, N> scale(const Vec<T, N - 1>& v) {
    Eigen::Transform<T, N - 1, Eigen::Affine> t;
    t.setIdentity();
    return t.scale(v).matrix();
}

const auto scale3f = scale<float, 4>;

template <typename T>
using Vec2 = Vec<T, 2>;

template <typename T>
using Vec3 = Vec<T, 3>;

template <typename T>
using Vec4 = Vec<T, 4>;

using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;
using Vec2i = Vec2<int>;
using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;
using Vec3i = Vec3<int>;
using Vec4f = Vec4<float>;
using Vec4d = Vec4<double>;
using Vec4i = Vec4<int>;

template <typename T>
using Mat2 = Mat<T, 2>;

template <typename T>
using Mat3 = Mat<T, 3>;

template <typename T>
using Mat4 = Mat<T, 4>;

using Mat2f = Mat2<float>;
using Mat2d = Mat2<double>;
using Mat3f = Mat3<float>;
using Mat3d = Mat3<double>;
using Mat4f = Mat4<float>;
using Mat4d = Mat4<double>;

}