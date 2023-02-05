#include "DataDesc.hpp"
#include "Utility.hpp"

glm::dvec3 circleCenter(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) {
    double a1, b1, c1, d1;
    double a2, b2, c2, d2;
    double a3, b3, c3, d3;

    double la, lb, lc, l;

    glm::dvec3 res;

    la = a.x * a.x + a.y * a.y + a.z * a.z;
    lb = b.x * b.x + b.y * b.y + b.z * b.z;
    lc = c.x * c.x + c.y * c.y + c.z * c.z;

    a1 = (a.y * b.z - b.y * a.z - a.y * c.z + c.y * a.z + b.y * c.z - c.y * b.z);
    b1 = -(a.x * b.z - b.x * a.z - a.x * c.z + c.x * a.z + b.x * c.z - c.x * b.z);
    c1 = (a.x * b.y - b.x * a.y - a.x * c.y + c.x * a.y + b.x * c.y - c.x * b.y);
    d1 = a.x * b.z * c.y + a.y * b.x * c.z + a.z * b.y * c.x - a.x * b.y * c.z - a.y * b.z * c.x - a.z * b.x * c.y;

    a2 = 2 * (b.x - a.x);
    b2 = 2 * (b.y - a.y);
    c2 = 2 * (b.z - a.z);
    d2 = la - lb;

    a3 = 2 * (c.x - a.x);
    b3 = 2 * (c.y - a.y);
    c3 = 2 * (c.z - a.z);
    d3 = la - lc;

    l = a1 * b2 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a1 * b3 * c2 - a2 * b1 * c3 - a3 * b2 * c1;

    res.x = -(b1 * c2 * d3 + b2 * c3 * d1 + b3 * c1 * d2 - b1 * c3 * d2 - b2 * c1 * d3 - b3 * c2 * d1) / l;
    res.y = (a1 * c2 * d3 + a2 * c3 * d1 + a3 * c1 * d2 - a1 * c3 * d2 - a2 * c1 * d3 - a3 * c2 * d1) / l;
    res.z = -(a1 * b2 * d3 + a2 * b3 * d1 + a3 * b1 * d2 - a1 * b3 * d2 - a2 * b1 * d3 - a3 * b2 * d1) / l;

    return res;
}

std::pair<glm::dvec3, double> CircleFitByTaubin(const std::vector<glm::dvec3>& pts) {
    static constexpr int maxIterTimes = 99;

    int n = pts.size();
    double x, y, z, meanX, meanY, meanZ, Mxx, Mzz, Mxz, Mxl, Mzl, Mll;
    meanX = meanY = meanZ = Mxx = Mzz = Mxz = Mxl = Mzl = Mll = 0;

    for(auto& pt : pts) {
        meanX += pt.x;
        meanY += pt.y;
        meanZ += pt.z;
    }
    meanX /= n;
    meanY /= n;
    meanZ /= n;

    for(auto& pt : pts) {
        double xi = pt.x - meanX;
        double zi = pt.z - meanZ;
        double li = xi * xi + zi * zi;
        Mxx += xi * xi;
        Mzz += zi * zi;
        Mxz += xi * zi;
        Mxl += xi * li;
        Mzl += zi * li;
        Mll += li * li;
    }
    Mxx /= n;
    Mzz /= n;
    Mxz /= n;
    Mxl /= n;
    Mzl /= n;
    Mll /= n;

    double Ml = Mxx + Mzz;
    double Cov_xz = Mxx * Mzz - Mxz * Mxz;
    double Var_l = Mll - Ml * Ml;
    double A3 = 4 * Ml;
    double A2 = -3 * Ml * Ml - Mll;
    double A1 = Var_l * Ml + 4 * Cov_xz * Ml - Mxl * Mxl - Mzl * Mzl;
    double A0 = Mxl * (Mxl * Mzz - Mzl * Mxz) + Mzl * (Mzl * Mxx - Mxl * Mxz) - Var_l * Cov_xz;
    double A22 = 2 * A2;
    double A33 = 3 * A3;

    int i;
    for(x = 0, z = A0, i = 0; i < maxIterTimes; i++) {
        double xnew = x - z / (A1 + x * (A22 + A33 * x));
        if((xnew == x) || (!std::isfinite(xnew)))
            break;
        double znew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));
        if(std::abs(znew) >= std::abs(z))
            break;
        x = xnew;
        z = znew;
    }

    double det = x * x - x * Ml + Cov_xz;
    double Xcenter, Zcenter;
    if(det == 0) {
        Xcenter = 0;
        Zcenter = 0;
    } else {
        Xcenter = (Mxl * (Mzz - x) - Mzl * Mxz) / det / 2;
        Zcenter = (Mzl * (Mxx - x) - Mxl * Mxz) / det / 2;
    }
    x = Xcenter + meanX;
    y = meanY;
    z = Zcenter + meanZ;

    return std::make_pair(glm::dvec3{ x, y, z }, std::sqrt(Xcenter * Xcenter + Zcenter * Zcenter + Ml));
}

std::pair<double, double> FitLine(const std::vector<std::pair<double, double>>& pts) {
    int n = pts.size();
    double meanX = 0, meanY = 0, sigma1 = 0, sigma2 = 0, k, m;
    for(const auto& pt : pts) {
        meanX += pt.first;
        meanY += pt.second;
        sigma1 += pt.first * pt.second;
        sigma2 += pt.first * pt.first;
    }
    meanX /= n;
    meanY /= n;
    k = (sigma1 - n * meanX * meanY) / (sigma2 - n * meanX * meanX);
    m = meanY - k * meanX;
    return std::make_pair(k, m);
}

std::complex<double> sqrtN(const std::complex<double>& x, double n) {
    if(auto r = std::hypot(x.real(), x.imag()); r > 0.0) {
        auto a = std::atan2(x.imag(), x.real());
        n = 1.0 / n;
        r = std::pow(r, n);
        a *= n;
        return std::polar(r, a);
    }
    return {};
}

double ferrari(std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d,
               std::complex<double> e) {
    std::complex<double> x[4];
    a = 1.0 / a;
    b *= a;
    c *= a;
    d *= a;
    e *= a;
    const auto p = (c * c + 12.0 * e - 3.0 * b * d) / 9.0;
    const auto q = (27.0 * d * d + 2.0 * c * c * c + 27.0 * b * b * e - 72.0 * c * e - 9.0 * b * c * d) / 54.0;
    const auto D = sqrtN(q * q - p * p * p, 2.0);
    std::complex<double> u = q + D;
    std::complex<double> v = q - D;
    if(v.real() * v.real() + v.imag() * v.imag() > u.real() * u.real() + u.imag() * u.imag()) {
        u = sqrtN(v, 3.0);
    } else {
        u = sqrtN(u, 3.0);
    }
    std::complex<double> y;
    if(u.real() * u.real() + u.imag() * u.imag() > 0.0) {
        v = p / u;
        const std::complex<double> o1(-0.5, +0.86602540378443864676372317075294);
        const std::complex<double> o2(-0.5, -0.86602540378443864676372317075294);
        std::complex<double>& yMax = x[0];
        double m2Max = 0.0;
        // int iMax = -1;
        for(int i = 0; i < 3; ++i) {
            y = u + v + c / 3.0;
            u *= o1;
            v *= o2;
            a = b * b + 4.0 * (y - c);
            if(const auto m2 = a.real() * a.real() + a.imag() * a.imag(); 0 == i || m2Max < m2) {
                m2Max = m2;
                yMax = y;
                // iMax = i;
            }
        }
        y = yMax;
    } else {
        y = c / 3.0;
    }
    if(const auto m = sqrtN(b * b + 4.0 * (y - c), 2.0); m.real() * m.real() + m.imag() * m.imag() >= DBL_MIN) {
        const std::complex<double> n = (b * y - 2.0 * d) / m;

        a = sqrtN((b + m) * (b + m) - 8.0 * (y + n), 2.0);
        x[0] = (-(b + m) + a) / 4.0;
        x[1] = (-(b + m) - a) / 4.0;
        a = sqrtN((b - m) * (b - m) - 8.0 * (y - n), 2.0);
        x[2] = (-(b - m) + a) / 4.0;
        x[3] = (-(b - m) - a) / 4.0;
    } else {
        a = sqrtN(b * b - 8.0 * y, 2.0);
        x[0] = x[1] = (-b + a) / 4.0;
        x[2] = x[3] = (-b - a) / 4.0;
    }
    double ans = 1000;
    for(auto& i : x) {
        if(i.real() > 0 && std::fabs(i.imag()) < 1e7 && i.real() < ans)
            ans = i.real();
    }
    return ans;
}

std::tuple<double, double, double> solveWithoutAirDrag(glm::dvec3 targetPos, glm::dvec3 targetVel) {
    static const double g = GlobalSettings::get().gForce;
    const double bulletSpeed = GlobalSettings::get().bulletSpeed;
    double airDuration = ferrari(
        1, 0, -(4 * g * targetPos.z + 4 * square(bulletSpeed) - 4 * square(targetVel.x) - 4 * square(targetVel.y)) / square(g),
        (8 * targetPos.x * targetVel.x + 8 * targetPos.y * targetVel.y) / square(g),
        (4 * square(targetPos.x) + 4 * square(targetPos.y) + 4 * square(targetPos.z)) / square(g));
    double verticalSpeed = targetPos.z / airDuration - 0.5 * g * airDuration;
    double horizontalSpeedX = (targetPos.x + targetVel.x * airDuration) / airDuration;
    double horizontalSpeedY = (targetPos.y + targetVel.y * airDuration) / airDuration;

    double pitchAngle = std::asin(verticalSpeed / bulletSpeed);
    double yawAngle = std::atan2(horizontalSpeedY, horizontalSpeedX) - glm::half_pi<double>();

    return std::make_tuple(airDuration, yawAngle, pitchAngle);
}