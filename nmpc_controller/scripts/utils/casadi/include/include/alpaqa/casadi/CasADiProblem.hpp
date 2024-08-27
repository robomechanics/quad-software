#pragma once

#include <alpaqa/casadi-loader-export.hpp>
#include <alpaqa/config/config.hpp>
#include <alpaqa/problem/box-constr-problem.hpp>
#include <alpaqa/util/copyable_unique_ptr.hpp>
#include <filesystem>

namespace alpaqa {
namespace casadi_loader {
template <Config>
struct CasADiFunctionsWithParam;
}  // namespace casadi_loader

/// Problem definition for a CasADi problem, loaded from a DLL.
/// @ingroup grp_Problems
template <Config Conf = DefaultConfig>
class CasADiProblem : public BoxConstrProblem<Conf> {
 public:
  USING_ALPAQA_CONFIG(Conf);
  vec param;

  /// Load a problem generated by CasADi (with parameters).
  ///
  /// @param  filename
  ///         Filename of the shared library to load the functions from.
  ///
  /// The file should contain functions with the names `f`, `grad_f`, `g` and
  /// `grad_g`. These functions evaluate the objective function, its gradient,
  /// the constraints, and the constraint gradient times a vector respectively.
  /// For second order solvers, additional functions `hess_L`, `hess_ψ`,
  /// `hess_L_prod` and `hess_ψ_prod` can be provided to evaluate the
  /// Hessian of the (augmented) Lagrangian and Hessian-vector products.
  ///
  /// @throws std::invalid_argument
  ///         The dimensions of the loaded functions do not match.
  CasADiProblem(const std::string &filename);
  ~CasADiProblem();

  CasADiProblem(const CasADiProblem &);
  CasADiProblem &operator=(const CasADiProblem &);
  CasADiProblem(CasADiProblem &&) noexcept;
  CasADiProblem &operator=(CasADiProblem &&) noexcept;

  /// Load the numerical problem data (bounds and parameters) from a CSV file.
  /// The file should contain 7 rows, with the following contents:
  ///   1. @ref C lower bound [n]
  ///   2. @ref C upper bound [n]
  ///   3. @ref D lower bound [m]
  ///   4. @ref D upper bound [m]
  ///   5. @ref param [p]
  ///   6. @ref l1_reg [0, 1 or n]
  ///   7. @ref penalty_alm_split [1]
  ///
  /// Line endings are encoded using a single line feed (`\n`), and the column
  /// separator can be specified using the @p sep argument.
  void load_numerical_data(const std::filesystem::path &filepath,
                           char sep = ',');

  // clang-format off
    [[nodiscard]] real_t eval_f(crvec x) const;
    void eval_grad_f(crvec x, rvec grad_fx) const;
    real_t eval_f_grad_f(crvec x, rvec grad_fx) const; // NOLINT(*nodiscard)
    void eval_g(crvec x, rvec g) const;
    void eval_grad_g_prod(crvec x, crvec y, rvec grad_gxy) const;
    void eval_grad_ψ(crvec x, crvec y, crvec Σ, rvec grad_ψ, rvec work_n, rvec work_m) const;
    real_t eval_ψ_grad_ψ(crvec x, crvec y, crvec Σ, rvec grad_ψ, rvec work_n, rvec work_m) const; // NOLINT(*nodiscard)
    void eval_grad_L(crvec x, crvec y, rvec grad_L, rvec work_n) const;
    [[nodiscard]] real_t eval_ψ(crvec x, crvec y, crvec Σ, rvec ŷ) const;
    void eval_grad_gi(crvec x, index_t i, rvec grad_i) const;
    [[nodiscard]] length_t get_jac_g_num_nonzeros() const;
    void eval_jac_g(crvec x, rindexvec inner_idx, rindexvec outer_ptr, rvec J_values) const;
    void eval_hess_L_prod(crvec x, crvec y, real_t scale, crvec v, rvec Hv) const;
    [[nodiscard]] length_t get_hess_L_num_nonzeros() const;
    void eval_hess_L(crvec x, crvec y, real_t scale, rindexvec inner_idx, rindexvec outer_ptr, rvec H_values) const;
    void eval_hess_ψ_prod(crvec x, crvec y, crvec Σ, real_t scale, crvec v, rvec Hv) const;
    [[nodiscard]] length_t get_hess_ψ_num_nonzeros() const;
    void eval_hess_ψ(crvec x, crvec y, crvec Σ, real_t scale, rindexvec inner_idx, rindexvec outer_ptr, rvec H_values) const;
  // clang-format on

  /// @see @ref TypeErasedProblem::provides_eval_grad_gi
  [[nodiscard]] bool provides_eval_grad_gi() const;
  /// @see @ref TypeErasedProblem::provides_eval_jac_g
  [[nodiscard]] bool provides_eval_jac_g() const;
  /// @see @ref TypeErasedProblem::provides_eval_hess_L_prod
  [[nodiscard]] bool provides_eval_hess_L_prod() const;
  /// @see @ref TypeErasedProblem::provides_eval_hess_L
  [[nodiscard]] bool provides_eval_hess_L() const;
  /// @see @ref TypeErasedProblem::provides_eval_hess_ψ_prod
  [[nodiscard]] bool provides_eval_hess_ψ_prod() const;
  /// @see @ref TypeErasedProblem::provides_eval_hess_ψ
  [[nodiscard]] bool provides_eval_hess_ψ() const;

 private:
  using Functions = casadi_loader::CasADiFunctionsWithParam<Conf>;
  util::copyable_unique_ptr<Functions> impl;
};

CASADI_LOADER_EXPORT_EXTERN_TEMPLATE(class, CasADiProblem, EigenConfigd);
CASADI_LOADER_EXPORT_EXTERN_TEMPLATE(class, CasADiProblem, DefaultConfig);

}  // namespace alpaqa