#pragma once

#include <SofaCaribou/Ode/NewtonRaphsonSolver.h>
#include <SofaBaseLinearSolver/DefaultMultiMatrixAccessor.h>
#include <sofa/core/objectmodel/Data.h>

namespace SofaCaribou::ode {

class StaticODESolver : public NewtonRaphsonSolver {
public:
    SOFA_CLASS(StaticODESolver, NewtonRaphsonSolver);

    template <typename T>
    using Data = sofa::core::objectmodel::Data<T>;

private:

    /** @see NewtonRaphsonSolver::assemble_rhs_vector */
    void assemble_rhs_vector(const sofa::core::MechanicalParams & mechanical_parameters,
                             const sofa::core::behavior::MultiMatrixAccessor & matrix_accessor,
                             sofa::core::MultiVecDerivId & f_id,
                             sofa::defaulttype::BaseVector * f) final;

    /** @see NewtonRaphsonSolver::assemble_system_matrix */
    void assemble_system_matrix(const sofa::core::MechanicalParams & mechanical_parameters,
                                sofa::component::linearsolver::DefaultMultiMatrixAccessor & matrix_accessor,
                                sofa::defaulttype::BaseMatrix * A) final;

    /** @see NewtonRaphsonSolver::propagate_position_increment */
    void propagate_position_increment(const sofa::core::MechanicalParams & mechanical_parameters,
                                      const sofa::core::behavior::MultiMatrixAccessor & matrix_accessor,
                                      const sofa::defaulttype::BaseVector * dx,
                                      sofa::core::MultiVecCoordId & x_id,
                                      sofa::core::MultiVecDerivId & v_id,
                                      sofa::core::MultiVecDerivId & dx_id) final;
};

} // namespace SofaCaribou::ode