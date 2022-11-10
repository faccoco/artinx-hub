#pragma once
#include <cstdint>

#include "SuppressWarningBegin.hpp"

#include <glm/glm.hpp>

#include "SuppressWarningEnd.hpp"

enum class FrameOfRef : uint32_t { Ground, Robot, Gun, Camera, Armor };

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

    constexpr Scalar() {}
    constexpr Scalar(double _val) : val(_val) {}
    constexpr Scalar(const Scalar<Unit>& rhs) : val(rhs.val) {}

    constexpr Scalar& operator=(double _val) {
        val = _val;
        return *this;
    }
    constexpr Scalar& operator=(const Scalar<Unit>& rhs) {
        val = rhs.val;
        return *this;
    }

    Scalar<Unit> operator+(Scalar rhs) const noexcept {
        return Scalar<Unit>{ val + rhs.val };
    }
    Scalar<Unit>& operator+=(Scalar rhs) noexcept {
        val += rhs.val;
        return (*this);
    }
    Scalar<Unit> operator-(Scalar rhs) const noexcept {
        return Scalar<Unit>{ val - rhs.val };
    }
    Scalar<Unit>& operator-=(Scalar rhs) noexcept {
        val -= rhs.val;
        return (*this);
    }
    template <UnitType RhsUnit>
    Scalar<multiply<Unit, RhsUnit>> operator*(Scalar<RhsUnit> rhs) const noexcept {
        return Scalar<multiply<Unit, RhsUnit>>{ val * rhs.val };
    }
    template <UnitType RhsUnit>
    Scalar<division<Unit, RhsUnit>> operator/(Scalar<RhsUnit> rhs) const noexcept {
        return Scalar<division<Unit, RhsUnit>>{ val / rhs.val };
    }
    bool operator>(Scalar rhs) const noexcept {
        return val > rhs.val;
    }
    bool operator>=(Scalar rhs) const noexcept {
        return val >= rhs.val;
    }
    bool operator<(Scalar rhs) const noexcept {
        return val < rhs.val;
    }
    bool operator<=(Scalar rhs) const noexcept {
        return val <= rhs.val;
    }
    bool operator==(Scalar rhs) const noexcept {
        return val == rhs.val;
    }
    bool operator!=(Scalar rhs) const noexcept {
        return val != rhs.val;
    }
};

template <UnitType Unit, FrameOfRef FoR>
class Vector final {
public:
    glm::dvec3 val;

    constexpr Vector() = default;
    constexpr Vector(const double x, const double y, const double z) : val(x, y, z) {}
    constexpr Vector(const Vector& rhs) = default;
    constexpr Vector(Vector&& rhs) = default;
    constexpr Vector(const glm::dvec3& val) : val(val) {}
    constexpr Vector(glm::dvec3&& val) : val(std::move(val)) {}

    constexpr Vector& operator=(const Vector& rhs) = default;
    constexpr Vector& operator=(Vector&& rhs) = default;
    constexpr Vector& operator=(const glm::dvec3& rhsVal) {
        val = rhsVal;
        return *this;
    }
    constexpr Vector& operator=(glm::dvec3&& rhsVal) {
        val = std::move(rhsVal);
        return *this;
    }

    [[nodiscard]] constexpr glm::dvec3& raw() noexcept {
        return val;
    }
    [[nodiscard]] constexpr const glm::dvec3& raw() const noexcept {
        return val;
    }
    [[nodiscard]] constexpr auto& operator[](int i) noexcept {
        return val[i];
    }
    [[nodiscard]] constexpr const auto& operator[](int i) const noexcept {
        return val[i];
    }

    void setZero() noexcept {
        val = glm::dvec3{ 0, 0, 0 };
    }

    void setValue(const glm::dvec3& rhsVal) noexcept {
        val = rhsVal;
    }
    void setValue(glm::dvec3&& rhsVal) noexcept {
        val = std::move(rhsVal);
    }

    constexpr bool operator==(const Vector& rhs) const noexcept {
        return val == rhs.val;
    }
    constexpr bool operator!=(const Vector& rhs) const noexcept {
        return val != rhs.val;
    }

    constexpr Vector operator+(const Vector& rhs) const noexcept {
        return val + rhs.val;
    }
    constexpr Vector& operator+=(const Vector& rhs) noexcept {
        val += rhs.val;
        return *this;
    }

    constexpr Vector operator-(const Vector& rhs) const noexcept {
        return val - rhs.val;
    }
    constexpr Vector& operator-=(const Vector& rhs) noexcept {
        val -= rhs.val;
        return *this;
    }

    template <UnitType RhsUnit>
    constexpr auto operator*(const Scalar<RhsUnit>& rhs) const noexcept {
        return Vector<multiply<Unit, RhsUnit>, FoR>(val * rhs.val);
    }
    template <UnitType RhsUnit>
    constexpr auto operator/(const Scalar<RhsUnit>& rhs) const noexcept {
        return Vector<division<Unit, RhsUnit>, FoR>(val / rhs.val);
    }
    constexpr Vector operator-() const noexcept {
        return -val;
    }
};

template <UnitType Lhs, UnitType Rhs, FrameOfRef FoR>
constexpr auto operator*(const Scalar<Lhs>& lhs, const Vector<Rhs, FoR>& rhs) noexcept {
    return rhs * lhs;
}

template <UnitType Unit, FrameOfRef FoR>
auto length(const Vector<Unit, FoR>& val) noexcept {
    return Scalar<Unit>{ glm::length(val.val) };
}

template <UnitType Unit, FrameOfRef FoR>
auto lerp(const Vector<Unit, FoR>& a, const Vector<Unit, FoR>& b, double u) noexcept {
    return Vector<Unit, FoR>{ glm::mix(a.val, b.val, u) };
}

template <UnitType Unit, FrameOfRef FoR>
class Point final {
public:
    glm::dvec3 val;

    constexpr Point() = default;
    constexpr Point(const double x, const double y, const double z) : val(x, y, z) {}
    constexpr Point(const Point& rhs) = default;
    constexpr Point(Point&& rhs) = default;
    constexpr Point(const glm::dvec3& val) : val(val) {}
    constexpr Point(glm::dvec3&& val) : val(std::move(val)) {}

    constexpr Point& operator=(const Point& rhs) = default;
    constexpr Point& operator=(Point&& rhs) = default;
    constexpr Point& operator=(const glm::dvec3& rhsVal) {
        val = rhsVal;
        return *this;
    }
    constexpr Point& operator=(glm::dvec3&& rhsVal) {
        val = std::move(rhsVal);
        return *this;
    }

    [[nodiscard]] constexpr glm::dvec3& raw() noexcept {
        return val;
    }
    [[nodiscard]] constexpr const glm::dvec3& raw() const noexcept {
        return val;
    }
    [[nodiscard]] constexpr auto& operator[](int i) noexcept {
        return val[i];
    }
    [[nodiscard]] constexpr const auto& operator[](int i) const noexcept {
        return val[i];
    }

    void setZero() noexcept {
        val = glm::dvec3{ 0, 0, 0 };
    }

    void setValue(const glm::dvec3& rhsVal) noexcept {
        val = rhsVal;
    }
    void setValue(glm::dvec3&& rhsVal) noexcept {
        val = std::move(rhsVal);
    }

    constexpr bool operator==(const Point& rhs) const noexcept {
        return val == rhs.val;
    }
    constexpr bool operator!=(const Point& rhs) const noexcept {
        return val != rhs.val;
    }

    constexpr Point operator+(const Vector<Unit, FoR>& rhs) const noexcept {
        return val + rhs.val;
    }
    constexpr Point& operator+=(const Vector<Unit, FoR>& rhs) noexcept {
        val += rhs.val;
        return *this;
    }

    constexpr Point operator-(const Vector<Unit, FoR>& rhs) const noexcept {
        return val - rhs.val;
    }
    constexpr Point& operator-=(const Vector<Unit, FoR>& rhs) noexcept {
        val -= rhs.val;
        return *this;
    }
    constexpr Vector<Unit, FoR> operator-(const Point& rhs) const noexcept {
        return val - rhs.val;
    }
};

template <UnitType Unit, FrameOfRef FoR>
auto lerp(const Point<Unit, FoR>& a, const Point<Unit, FoR>& b, double u) noexcept {
    return Point<Unit, FoR>{ glm::mix(a.val, b.val, u) };
}

template <UnitType Unit, FrameOfRef FoR>
auto distance(const Point<Unit, FoR>& a, const Point<Unit, FoR>& b) noexcept {
    return Scalar<Unit>{ glm::distance(a.val, b.val) };
}

struct Normalized final {};

template <UnitType Unit, FrameOfRef FoR>
class Normal final {
public:
    glm::dvec3 val;

    Normal() = delete;
    Normal(const double x, const double y, const double z) : val(glm::normalize(glm::dvec3{ x, y, z })) {}
    constexpr Normal(const double x, const double y, const double z, Normalized) : val(x, y, z) {}
    constexpr Normal(const Normal&) = default;
    constexpr Normal(Normal&&) = default;
    Normal(const glm::dvec3& val) : val(glm::normalize(val)) {}
    constexpr Normal(const glm::dvec3& val, Normalized) : val(val) {}
    Normal(glm::dvec3&& val) : val(std::move(glm::normalize(val))) {}
    constexpr Normal(glm::dvec3&& val, Normalized) : val(std::move(val)) {}
    Normal(const Vector<Unit, FoR>& v) : val(glm::normalize(v.val)) {}
    constexpr Normal(const Vector<Unit, FoR>& v, Normalized) : val(v.val) {}
    Normal(Vector<Unit, FoR>&& v) : val(std::move(glm::normalize(v.val))) {}
    constexpr Normal(Vector<Unit, FoR>&& v, Normalized) : val(v.val) {}

    constexpr Normal& operator=(const Normal& rhs) = default;
    constexpr Normal& operator=(Normal&& rhs) = default;
    constexpr Normal& operator=(const glm::dvec3& rhs) {
        val = glm::normalize(rhs);
        return *this;
    }
    constexpr Normal& operator=(const Vector<Unit, FoR>& rhs) {
        val = glm::normalize(rhs.val);
        return *this;
    }

    [[nodiscard]] constexpr glm::dvec3& raw() noexcept {
        return val;
    }
    [[nodiscard]] constexpr const glm::dvec3& raw() const noexcept {
        return val;
    }
    [[nodiscard]] constexpr auto& operator[](int i) noexcept {
        return val[i];
    }
    [[nodiscard]] constexpr const auto& operator[](int i) const noexcept {
        return val[i];
    }

    constexpr Normal operator+(const Vector<Unit, FoR>& rhs) const noexcept {
        return val + rhs.val;
    }
    constexpr Normal& operator+=(const Vector<Unit, FoR>& rhs) noexcept {
        val = glm::normalize(val + rhs.val);
        return *this;
    }

    constexpr Normal operator-(const Vector<Unit, FoR>& rhs) const noexcept {
        return val - rhs.val;
    }
    constexpr Normal& operator-=(const Vector<Unit, FoR>& rhs) noexcept {
        val = glm::normalize(val - rhs.val);
        return *this;
    }
    constexpr Vector<Unit, FoR> operator*(const Scalar<Unit> s) const noexcept {
        return val * s.val;
    }
    constexpr Normal operator-() const noexcept {
        return -val;
    }
};

template <UnitType Unit, FrameOfRef FoR>
constexpr auto cross(const Normal<Unit, FoR>& a, const Normal<Unit, FoR>& b) noexcept {
    return Normal<Unit, FoR>{ glm::cross(a, b), Normalized{} };
}

template <UnitType Unit, FrameOfRef FoR>
constexpr auto dot(const Normal<Unit, FoR>& a, const Normal<Unit, FoR>& b) noexcept {
    return glm::dot(a.val, b.val);
}

template <UnitType Unit, FrameOfRef FoR>
constexpr auto dot(const Vector<Unit, FoR>& a, const Normal<Unit, FoR>& b) noexcept {
    return glm::dot(a.val, b.val);
}

template <UnitType Unit, FrameOfRef FoR>
constexpr auto dot(const Normal<Unit, FoR>& a, const Vector<Unit, FoR>& b) noexcept {
    return glm::dot(a.val, b.val);
}

template <UnitType Unit, FrameOfRef FoR>
constexpr auto dot(const Vector<Unit, FoR>& a, const Vector<Unit, FoR>& b) noexcept {
    return glm::dot(a.val, b.val);
}

template <UnitType Unit, FrameOfRef FoR>
constexpr auto normalize(Vector<Unit, FoR> v) {
    return Normal<Unit, FoR>{ v };
}

template <FrameOfRef A, FrameOfRef B, bool HasTranslate = false>
class Transform final {
public:
    glm::dmat4 val;

    Transform() = default;
    Transform(const Transform& rhs) = default;
    Transform(Transform&& rhs) = default;
    template <bool RhsTranslate, typename = std::enable_if_t<RhsTranslate || !HasTranslate>>
    Transform(const Transform<B, A, RhsTranslate>& rhs) : val(rhs.inverse().val) {}
    Transform(const glm::dmat4& transform) : val(transform) {}
    Transform(glm::dmat4&& transform) : val(std::move(transform)) {}

    Transform& operator=(const Transform&) = default;
    Transform& operator=(Transform&&) = default;
    Transform& operator=(const glm::dmat4& rhs) {
        val = rhs;
        return *this;
    }
    Transform& operator=(glm::dmat4&& rhs) {
        val = std::move(rhs);
        return *this;
    }

    [[nodiscard]] constexpr glm::dmat4& raw() noexcept {
        return val;
    }
    [[nodiscard]] constexpr const glm::dmat4& raw() const noexcept {
        return val;
    }
    [[nodiscard]] constexpr auto& operator[](int i) noexcept {
        return val[i];
    }
    [[nodiscard]] constexpr const auto& operator[](int i) const noexcept {
        return val[i];
    }

    auto inverse() const noexcept {
        return Transform<B, A, HasTranslate>(glm::inverse(val));
    }

    template <UnitType Unit>
    std::enable_if_t<HasTranslate, Point<Unit, B>> operator()(const Point<Unit, A> rhs) const noexcept {
        return glm::dvec3{ val * glm::dvec4{ rhs.val, 1.0 } };
    }
    template <UnitType Unit>
    Vector<Unit, B> operator()(const Vector<Unit, A> rhs) const noexcept {
        return glm::dvec3{ val * glm::dvec4{ rhs.val, 0.0 } };
    }
    template <UnitType Unit>
    Normal<Unit, B> operator()(const Normal<Unit, A> rhs) const noexcept {
        return glm::dvec3{ val * glm::dvec4{ rhs.val, 0.0 } };
    }

    template <FrameOfRef C, bool RhsHasTranslate>
    auto operator*(const Transform<B, C, RhsHasTranslate>& rhs) const noexcept {
        return Transform < A, C, HasTranslate && RhsHasTranslate > (rhs.val * val);
    }
    template <FrameOfRef C, bool RhsHasTranslate>
    auto operator*(const Transform<C, A, RhsHasTranslate>& rhs) const noexcept {
        return Transform < C, B, HasTranslate && RhsHasTranslate > (val * rhs.val);
    }
    template <FrameOfRef C, bool RhsHasTranslate>
    auto operator()(const Transform<B, C, RhsHasTranslate>& rhs) const noexcept {
        return Transform < A, C, HasTranslate && RhsHasTranslate > (rhs.val * val);
    }
    template <FrameOfRef C, bool RhsHasTranslate>
    auto operator()(const Transform<C, A, RhsHasTranslate>& rhs) const noexcept {
        return Transform < C, B, HasTranslate && RhsHasTranslate > (val * rhs.val);
    }

    operator Transform<A, B, false>() const noexcept {
        return Transform<A, B, false>(val);
    }

    // displacement under B
    const Point<UnitType::Distance, B>& displacement() const {
        return reinterpret_cast<const Point<UnitType::Distance, B>&>(val[3]);
    }

    Point<UnitType::Distance, B>& displacement() {
        return reinterpret_cast<Point<UnitType::Distance, B>&>(val[3]);
    }
};

template <FrameOfRef A, FrameOfRef B, bool LhsHasTranslate, bool RhsHasTranslate>
auto combine(const Transform<A, B, LhsHasTranslate>& first, const Transform<A, B, RhsHasTranslate>& second) noexcept {
    return Transform < A, B, LhsHasTranslate && RhsHasTranslate > (second.val * first.val);
}

template <FrameOfRef A, FrameOfRef B, FrameOfRef C, bool LhsHasTranslate, bool RhsHasTranslate>
auto combine(const Transform<A, B, LhsHasTranslate>& first, const Transform<B, C, RhsHasTranslate>& second) noexcept {
    return Transform < A, C, LhsHasTranslate && RhsHasTranslate > (second.val * first.val);
}

template <FrameOfRef A, FrameOfRef B, FrameOfRef C, bool LhsHasTranslate, bool RhsHasTranslate>
auto combine(const Transform<B, C, LhsHasTranslate>& second, const Transform<A, B, RhsHasTranslate>& first) noexcept {
    return Transform < A, C, LhsHasTranslate && RhsHasTranslate > (second.val * first.val);
}

#define COMMA ,
static_assert((offsetof(Point<UnitType::Undefined COMMA FrameOfRef::Ground>, val.x) == offsetof(glm::dvec4, x)) &&
              (offsetof(Point<UnitType::Undefined COMMA FrameOfRef::Ground>, val.y) == offsetof(glm::dvec4, y)) &&
              (offsetof(Point<UnitType::Undefined COMMA FrameOfRef::Ground>, val.z) == offsetof(glm::dvec4, z)));
#undef COMMA