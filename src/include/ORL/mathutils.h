#pragma once

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
        return Base::operator /=(static_cast<Base::Scalar>(s));
    }

    template <typename Derived>
    inline Vec<T, N> operator /=(const Eigen::MatrixBase<Derived>& v) {
        *this = (this->array() / v.array()).matrix();
        return *this;
    }
};

// Operators
template <typename T, typename S, int N, typename = std::enable_if_t<std::is_arithmetic_v<S>>>
Vec<T, N> operator *(const S s, const Vec<T, N>& v) {
    return v * static_cast<T>(s);
}

template <typename S, typename Derived, typename = std::enable_if_t<std::is_arithmetic_v<S>>>
Vec<typename Eigen::MatrixBase<Derived>::Scalar, Eigen::MatrixBase<Derived>::RowsAtCompileTime> operator /(const S s, const Eigen::MatrixBase<Derived>& rhs) {
    return rhs.cwiseInverse() * static_cast<typename Eigen::MatrixBase<Derived>::Scalar>(s);
}

// Construct funcs
template <typename Derived>
inline auto concat(const Eigen::MatrixBase<Derived>& v, const typename Eigen::MatrixBase<Derived>::Scalar& s) {
    Eigen::Matrix<typename Eigen::MatrixBase<Derived>::Scalar, Eigen::MatrixBase<Derived>::RowsAtCompileTime + 1, 1> ret;
    ret << v, s;
    return ret;
}

template <typename Derived1, typename Derived2>
inline auto concat(const Eigen::MatrixBase<Derived1>& v1, const Eigen::MatrixBase<Derived2>& v2) {
    Eigen::Matrix<typename Eigen::MatrixBase<Derived1>::Scalar, Eigen::MatrixBase<Derived1>::RowsAtCompileTime +
        Eigen::MatrixBase<Derived2>::RowsAtCompileTime, 1> ret;
    ret << v1, v2;
    return ret;
}

template <int M, typename Derived>
inline auto head(const Eigen::MatrixBase<Derived>& v) {
    return v.head(M);
}

// Horizontal funcs
template <typename T, int N>
inline T max_component(const Vec<T, N>& v) {
    return v.maxCoeff();
}

template <typename T, int N>
inline T min_component(const Vec<T, N>& v) {
    return v.minCoeff();
}

template <typename T, int N>
inline T sum(const Vec<T, N>& v) {
    return v.sum();
}

template <typename Derived>
inline Eigen::MatrixBase<Derived>::Scalar length(const Eigen::MatrixBase<Derived>& v) {
    return v.norm();
}

template <typename Derived>
inline Eigen::MatrixBase<Derived>::Scalar length_squared(const Eigen::MatrixBase<Derived>& v) {
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
template <typename T, int N>
inline Vec<T, N> abs(const Vec<T, N>& v) {
    return v.cwiseAbs();
}

template <typename T, int N>
inline Vec<T, N> square(const Vec<T, N>& v) {
    return v * v;
}

template <typename T, int N>
inline Vec<T, N> sqrt(const Vec<T, N>& v) {
    return v.sqrt();
}

template <typename T, int N>
inline Vec<T, N> exp(const Vec<T, N>& v) {
    return v.exp();
}

template <typename T, int N>
inline Vec<T, N> pow(const Vec<T, N>& v, const T beta) {
    return v.pow(beta);
}

template <typename T, int N>
inline Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, const T t) {
    return a * (static_cast<T>(1) - t) + b * t;
}

template <typename T, int N>
inline Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, const Vec<T, N>& t) {
    return a * (Vec<T, N>{1} - t) + b * t;
}

// Comparison
template <typename Derived>
inline bool is_zero(const Eigen::MatrixBase<Derived>& v,
    const typename Eigen::MatrixBase<Derived>::Scalar& epsilon=1e-5)
{
    return v.isZero(epsilon);
}

template <typename T, int N>
inline Vec<T, N> vec_min(const Vec<T, N>& a, const Vec<T, N>& b) {
    return a.cwiseMin(b);
}

template <typename T, int N>
inline Vec<T, N> vec_max(const Vec<T, N>& a, const Vec<T, N>& b) {
    return a.cwiseMax(b);
}

}