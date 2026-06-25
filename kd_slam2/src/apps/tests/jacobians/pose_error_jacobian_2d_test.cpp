#include <iostream>
#include <iomanip>
#include <cstdlib>

#include "kd_slam/utils/geometry_2d_impl_.h"

using namespace kd_slam;
using Geometry             = geometry::GeometryTraits_<2, float>;
using Scalar               = Geometry::Scalar;
using VectorType           = Geometry::VectorType;
using IsometryType         = Geometry::IsometryType;
using PerturbationPoseType = Geometry::PerturbationPoseType;
using HessianType          = Geometry::HessianType;

static Scalar rnd(Scalar range = Scalar(1)) {
  return range * (Scalar(2) * std::rand() / RAND_MAX - Scalar(1));
}

static IsometryType randomTransform() {
  IsometryType T = IsometryType::Identity();
  T.linear()      = Geometry::rot2d(rnd(Scalar(M_PI * 0.8f)));
  T.translation() = VectorType(rnd(2), rnd(2));
  return T;
}

static bool checkMatrix(const HessianType& J_an, const HessianType& J_nu,
                         int idx, Scalar tol) {
  const Scalar err = (J_an - J_nu).cwiseAbs().maxCoeff();
  const bool   ok  = err < tol;
  std::cout << " " << std::setw(2) << idx
            << "  |  " << std::setw(12) << err
            << "   " << (ok ? "OK" : "FAIL") << "\n";
  return ok;
}

int main() {
  std::srand(42);
  const int    NUM_TESTS = 20;
  const Scalar H         = Scalar(1e-4);
  const Scalar TOL       = Scalar(1e-2);
  int n_pass = 0, n_total = 0;

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "\n--- dPoseToPoseError 2D ---\n";
  std::cout << "  #  |  |J_an - J_nu|_inf\n";
  std::cout << "-----+--------------------\n";

  for (int t = 0; t < NUM_TESTS; ++t) {
    const IsometryType ZXi = randomTransform().inverse() * randomTransform().inverse();
    const IsometryType Xm  = randomTransform();

    const HessianType J_an = Geometry::dPoseToPoseError(ZXi, Xm);

    HessianType J_nu;
    for (int i = 0; i < 3; ++i) {
      PerturbationPoseType v = PerturbationPoseType::Zero();
      v(i) =  H;  const IsometryType dXp = Geometry::expmap(v);
      v(i) = -H;  const IsometryType dXm = Geometry::expmap(v);
      J_nu.col(i) = (Geometry::poseToPoseError(ZXi, dXp * Xm)
                   - Geometry::poseToPoseError(ZXi, dXm * Xm)) / (Scalar(2) * H);
    }

    if (checkMatrix(J_an, J_nu, t, TOL)) ++n_pass;
    ++n_total;
  }

  std::cout << "-----+--------------------\n";
  std::cout << "PASSED " << n_pass << "/" << n_total << "\n";
  return (n_pass < n_total) ? 1 : 0;
}
