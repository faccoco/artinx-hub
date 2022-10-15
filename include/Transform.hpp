#pragma once
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <glm/glm.hpp>

#include "SuppressWarningEnd.hpp"

enum class FrameOfRef : uint32_t { Ground, Robot, Gun, Camera };

enum class UnitType : uint32_t {
    Distance,
    Angle,
    Time,
    LinearVelocity,
    AngularVelocity,
    LinearAcceleration,
    AngularAcceleration,
    Undefined
};

template <UnitType Lhs, UnitType Rhs>
constexpr UnitType multiply = UnitType::Undefined;

template <>
constexpr UnitType multiply<UnitType::LinearVelocity, UnitType::Time> = UnitType::Distance;

template <>
constexpr UnitType multiply<UnitType::LinearAcceleration, UnitType::Time> = UnitType::LinearVelocity;

template <>
constexpr UnitType multiply<UnitType::AngularVelocity, UnitType::Time> = UnitType::Angle;

template <>
constexpr UnitType multiply<UnitType::AngularAcceleration, UnitType::Time> = UnitType::AngularVelocity;

template <UnitType Lhs, UnitType Rhs>
constexpr UnitType division = UnitType::Undefined;

template <>
constexpr UnitType division<UnitType::Distance, UnitType::Time> = UnitType::LinearVelocity;

template <>
constexpr UnitType division<UnitType::Angle, UnitType::Time> = UnitType::AngularVelocity;

template <>
constexpr UnitType division<UnitType::LinearVelocity, UnitType::Time> = UnitType::LinearAcceleration;

template <>
constexpr UnitType division<UnitType::AngularVelocity, UnitType::Time> = UnitType::AngularAcceleration;

template <UnitType Unit>
struct Scalar final {
    double val;
};

template <UnitType Unit, FrameOfRef FoR>
class Vector final {
    glm::dvec3 mValue;

public:
    Vector() = default;
    explicit Vector(const glm::dvec3& val) : mValue{ val } {}

    [[nodiscard]] glm::dvec3 raw() const noexcept {
        return mValue;
    }

    void setZero() noexcept {
        mValue = { 0, 0, 0 };
    }

    void setValue(const glm::dvec3& val) noexcept {
        mValue = val;
    }

    Vector operator+(Vector rhs) const noexcept {
        return Vector{ mValue + rhs.mValue };
    }
    Vector& operator+=(Vector rhs) noexcept {
        mValue += rhs.mValue;
        return *this;
    }

    Vector operator-(Vector rhs) const noexcept {
        return Vector{ mValue - rhs.mValue };
    }
    Vector& operator-=(Vector rhs) noexcept {
        mValue -= rhs.mValue;
        return *this;
    }

    template <UnitType RhsUnit>
    auto operator*(Scalar<RhsUnit> rhs) const noexcept {
        return Vector<multiply<Unit, RhsUnit>, FoR>{ mValue * rhs.val };
    }
    template <UnitType RhsUnit>
    auto operator/(Scalar<RhsUnit> rhs) const noexcept {
        return Vector<division<Unit, RhsUnit>, FoR>{ mValue / rhs.val };
    }
    Vector operator-() const noexcept {
        return Vector{ -mValue };
    }
};

template <UnitType Lhs, UnitType Rhs, FrameOfRef FoR>
auto operator*(Scalar<Lhs> lhs, Vector<Rhs, FoR> rhs) noexcept {
    return rhs * lhs;
}

template <UnitType Unit, FrameOfRef FoR>
auto length(Vector<Unit, FoR> val) noexcept {
    return Scalar<Unit>{ glm::length(val.raw()) };
}

template <UnitType Unit, FrameOfRef FoR>
auto lerp(Vector<Unit, FoR> a, Vector<Unit, FoR> b, double u) noexcept {
    return Vector<Unit, FoR>{ glm::mix(a.raw(), b.raw(), u) };
}

template <UnitType Unit, FrameOfRef FoR>
class Point final {
    glm::dvec3 mValue;

public:
    Point() = default;
    explicit Point(const glm::dvec3& val) : mValue{ val } {}

    [[nodiscard]] glm::dvec3 raw() const noexcept {
        return mValue;
    }

    void setZero() noexcept {
        mValue = { 0, 0, 0 };
    }

    void setValue(const glm::dvec3& val) noexcept {
        mValue = val;
    }

    Point operator+(Vector<Unit, FoR> rhs) const noexcept {
        return Point{ mValue + rhs.raw() };
    }
    Point& operator+=(Vector<Unit, FoR> rhs) noexcept {
        mValue += rhs.raw();
        return *this;
    }
    Point operator-(Vector<Unit, FoR> rhs) const noexcept {
        return { mValue - rhs.raw() };
    }
    Point& operator-=(Vector<Unit, FoR> rhs) noexcept {
        mValue -= rhs.raw();
        return *this;
    }
    Vector<Unit, FoR> operator-(Point rhs) const noexcept {
        return Vector<Unit, FoR>{ mValue - rhs.mValue };
    }
};

template <UnitType Unit, FrameOfRef FoR>
auto lerp(Point<Unit, FoR> a, Point<Unit, FoR> b, double u) noexcept {
    return Point<Unit, FoR>{ glm::mix(a.raw(), b.raw(), u) };
}

template <UnitType Unit, FrameOfRef FoR>
auto distance(Point<Unit, FoR> a, Point<Unit, FoR> b) noexcept {
    return Scalar<Unit>{ glm::distance(a.raw(), b.raw()) };
}

struct Normalized final {};

template <FrameOfRef FoR>
class Normal final {
    glm::dvec3 mValue;

public:
    Normal(const glm::dvec3& val, Normalized) : mValue{ val } {}
    template <UnitType Unit>
    explicit Normal(const Vector<Unit, FoR> v) : mValue{ glm::normalize(v) } {}
    template <UnitType Unit>
    auto operator*(const Scalar<Unit> distance) const noexcept {
        return Vector<Unit, FoR>{ mValue * distance.val };
    }
    Normal operator-() const noexcept {
        return { -mValue, Normalized{} };
    }

    [[nodiscard]] glm::dvec3 raw() const noexcept {
        return mValue;
    }
};

template <FrameOfRef FoR>
auto cross(Normal<FoR> a, Normal<FoR> b) noexcept {
    return Normal<FoR>{ glm::cross(a, b), Normalized{} };
}

template <FrameOfRef FoR>
auto dot(Normal<FoR> a, Normal<FoR> b) noexcept {
    return glm::dot(a.raw(), b.raw());
}

template <UnitType Unit, FrameOfRef FoR>
auto dot(Vector<Unit, FoR> a, Normal<FoR> b) noexcept {
    return Scalar<Unit>{ glm::dot(a.raw(), b.raw()) };
}

template <UnitType Unit, FrameOfRef FoR>
auto normalize(Vector<Unit, FoR> v) {
    return Normal<FoR>{ v };
}

template <FrameOfRef A, FrameOfRef B, bool HasTranslate = false>
class Transform final {
    glm::dmat4 mTransform;         // A to B
    glm::dmat4 mInverseTransform;  // B to A

    template <FrameOfRef RhsA, FrameOfRef RhsB, bool RhsHasTranslate>
    friend class Transform;

public:
    Transform() = default;
    explicit Transform(const glm::dmat4& transform) : mTransform{ transform }, mInverseTransform{ glm::inverse(transform) } {}
    explicit Transform(const glm::dmat4& transform, const glm::dmat4& inverseTransform)
        : mTransform{ transform }, mInverseTransform{ inverseTransform } {}

    [[nodiscard]] const glm::dmat4& raw() const noexcept {
        return mTransform;
    }

    [[nodiscard]] const glm::dmat4& rawInverse() const noexcept {
        return mInverseTransform;
    }

    template <UnitType Unit>
    std::enable_if_t<HasTranslate, Point<Unit, B>> operator()(const Point<Unit, A> val) const noexcept {
        return Point<Unit, B>{ glm::dvec3{ mTransform * glm::dvec4{ val.raw(), 1.0 } } };
    }
    template <UnitType Unit>
    Vector<Unit, B> operator()(const Vector<Unit, A> val) const noexcept {
        return Vector<Unit, B>{ glm::dvec3{ mTransform * glm::dvec4{ val.raw(), 0.0 } } };
    }
    Normal<B> operator()(const Normal<A> val) const noexcept {
        return Normal<B>{ glm::dvec3{ glm::dvec4{ val.raw(), 0.0 } * mInverseTransform }, Normalized{} };
    }

    template <UnitType Unit>
    std::enable_if_t<HasTranslate, Point<Unit, A>> operator()(const Point<Unit, B> val) const noexcept {
        return Point<Unit, A>{ glm::dvec3{ mInverseTransform * glm::dvec4{ val.raw(), 1.0 } } };
    }
    template <UnitType Unit>
    Vector<Unit, A> operator()(const Vector<Unit, B> val) const noexcept {
        return Vector<Unit, A>{ glm::dvec3{ mInverseTransform * glm::dvec4{ val.raw(), 0.0 } } };
    }
    Normal<A> operator()(const Normal<B> val) const noexcept {
        return Normal<A>{ glm::dvec3{ glm::dvec4{ val.raw(), 0.0 } * mTransform }, Normalized{} };
    }

    template <FrameOfRef C, bool RhsHasTranslate>
    auto operator*(const Transform<B, C, RhsHasTranslate>& rhs) const noexcept {
        return Transform < A, C,
               HasTranslate && RhsHasTranslate > { rhs.mTransform * mTransform, mInverseTransform * rhs.mInverseTransform };
    }

    template <bool NeedTranslate, typename = std::enable_if_t<HasTranslate || !NeedTranslate>>
    operator Transform<B, A, NeedTranslate>() const noexcept {
        return Transform<B, A, NeedTranslate>{ mInverseTransform, mTransform };
    }

    operator Transform<A, B, false>() const noexcept {
        return Transform<A, B, false>{ mTransform, mInverseTransform };
    }
};
