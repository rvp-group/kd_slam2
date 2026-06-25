#include <iostream>
#include <iomanip>
#include <cstdlib>

#include "kd_slam/d3/cticp_impl_.h"
using namespace kd_slam::d3;
using namespace kd_slam;

using Traits               = CTICP_3D_Traits;
using Scalar               = Traits::Scalar;
using VectorType           = Traits::VectorType;
using TransformType        = Traits::IsometryType;
using PerturbationPoseType = Traits::PerturbationPoseType;
using JacobianVelocityType = Traits::JacobianVelocityType;

// ---- random helpers ---------------------------------------------------

static Scalar rnd(Scalar range = Scalar(1)) {
  return range * (Scalar(2) * std::rand() / RAND_MAX - Scalar(1));
}
static VectorType randomUnit() {
  return VectorType(rnd(), rnd(), rnd()).normalized();
}
static TransformType randomTransform() {
  TransformType T = TransformType::Identity();
  T.linear()      = Eigen::AngleAxis<Scalar>(rnd(Scalar(M_PI)), randomUnit()).toRotationMatrix();
  T.translation() = VectorType(rnd(2), rnd(2), rnd(2));
  return T;
}

// ---- numerical Jacobian helpers ----------------------------------------

static Scalar evalErrorA(const TransformType& Ta,
                          const VectorType& pa_local, const VectorType& na_local,
                          const VectorType& pb_global, const VectorType& nb_global,
                          const PerturbationPoseType& va, Scalar dt, int i, Scalar h) {
  PerturbationPoseType v = va; v(i) += h;
  Scalar e; JacobianVelocityType J;
  Traits::errorAndJacobianVelocityA(e, J, Ta, pa_local, na_local, pb_global, nb_global, v, dt);
  return e;
}

static Scalar evalErrorB(const TransformType& Tb,
                          const VectorType& pa_global, const VectorType& na_global,
                          const VectorType& pb_local,  const VectorType& nb_local,
                          const PerturbationPoseType& vb, Scalar dt, int i, Scalar h) {
  PerturbationPoseType v = vb; v(i) += h;
  Scalar e; JacobianVelocityType J;
  Traits::errorAndJacobianVelocityB(e, J, Tb, pa_global, na_global, pb_local, nb_local, v, dt);
  return e;
}


// ---- reporting ---------------------------------------------------------

static bool checkJacobian(const JacobianVelocityType& J_an,
                           const JacobianVelocityType& J_nu,
                           int idx, const char* tag, Scalar tol) {
  const Scalar err = (J_an - J_nu).cwiseAbs().maxCoeff();
  const bool   ok  = err < tol;
  std::cout << " " << std::setw(2) << idx << " " << tag
            << "  |  " << std::setw(12) << err
            << "   " << (ok ? "OK" : "FAIL") << "\n";
  return ok;
}

// -----------------------------------------------------------------------
int main() {
  std::srand(42);

  const int    NUM_TESTS = 20;
  const Scalar H         = Scalar(1e-3);
  const Scalar TOL       = Scalar(5e-3);

  int n_pass = 0, n_total = 0;

  std::cout << std::fixed << std::setprecision(6);

  // ---- Section 1: reference A/B vs numerical -------------------------
  std::cout << "\n--- Reference functions ---\n";
  std::cout << "  #  fn  |J_analytic - J_numeric|_inf\n";
  std::cout << "--------+-----------------------------\n";

  for (int t = 0; t < NUM_TESTS; ++t) {
    const TransformType Ta = randomTransform(), Tb = randomTransform();
    const VectorType pa_l  = VectorType(rnd(3), rnd(3), rnd(3));
    const VectorType na_l  = randomUnit();
    const VectorType pb_l  = VectorType(rnd(3), rnd(3), rnd(3));
    const VectorType nb_l  = randomUnit();
    const VectorType pa_g  = Ta * pa_l,  na_g = Ta.linear() * na_l;
    const VectorType pb_g  = Tb * pb_l,  nb_g = Tb.linear() * nb_l;
    PerturbationPoseType va, vb;
    va << rnd(.5), rnd(.5), rnd(.5), rnd(.3), rnd(.3), rnd(.3);
    vb << rnd(.5), rnd(.5), rnd(.5), rnd(.3), rnd(.3), rnd(.3);
    const Scalar dt = Scalar(0.1);

    // A
    {
      Scalar e; JacobianVelocityType J_an;
      Traits::errorAndJacobianVelocityA(e, J_an, Ta, pa_l, na_l, pb_g, nb_g, va, dt);
      JacobianVelocityType J_nu;
      for (int i = 0; i < 6; ++i)
        J_nu(i) = (evalErrorA(Ta, pa_l, na_l, pb_g, nb_g, va, dt, i, H)
                 - evalErrorA(Ta, pa_l, na_l, pb_g, nb_g, va, dt, i,-H)) / (2*H);
      if (checkJacobian(J_an, J_nu, t, "A    ", TOL))
        ++n_pass;
      ++n_total;
    }
    // B
    {
      Scalar e; JacobianVelocityType J_an;
      Traits::errorAndJacobianVelocityB(e, J_an, Tb, pa_g, na_g, pb_l, nb_l, vb, dt);
      JacobianVelocityType J_nu;
      for (int i = 0; i < 6; ++i)
        J_nu(i) = (evalErrorB(Tb, pa_g, na_g, pb_l, nb_l, vb, dt, i, H)
                 - evalErrorB(Tb, pa_g, na_g, pb_l, nb_l, vb, dt, i,-H)) / (2*H);
      if (checkJacobian(J_an, J_nu, t, "B    ", TOL))
        ++n_pass;
      ++n_total;
    }
  }


  std::cout << "--------+--------------+--------------------\n";
  std::cout << "PASSED " << n_pass << "/" << n_total << "\n";
  return (n_pass < n_total) ? 1 : 0;
}
