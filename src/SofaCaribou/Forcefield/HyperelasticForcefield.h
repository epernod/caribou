#pragma once

#include <functional>
#include <array>

#include <SofaCaribou/config.h>
#include <SofaCaribou/Material/HyperelasticMaterial.h>
#include <SofaCaribou/Forcefield/CaribouForcefield.h>

#include <Caribou/config.h>
#include <Caribou/constants.h>
#include <Caribou/Geometry/Element.h>
#include <Caribou/Topology/Mesh.h>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <SofaCaribou/Topology/CaribouTopology.h>

#if (defined(SOFA_VERSION) && SOFA_VERSION < 201200)
namespace sofa { using Index = unsigned int; }
#endif

namespace SofaCaribou::forcefield {

template <typename Element>
class HyperelasticForcefield : public CaribouForcefield<Element> {
public:
    SOFA_CLASS(SOFA_TEMPLATE(HyperelasticForcefield, Element), SOFA_TEMPLATE(CaribouForcefield, Element));

    // Type definitions
    using Inherit  = CaribouForcefield<Element>;
    using DataTypes = typename Inherit::DataTypes;
    using VecCoord  = typename DataTypes::VecCoord;
    using VecDeriv  = typename DataTypes::VecDeriv;
    using Coord     = typename DataTypes::Coord;
    using Deriv     = typename DataTypes::Deriv;
    using Real      = typename DataTypes::Real;

    static constexpr INTEGER_TYPE Dimension = caribou::geometry::traits<Element>::Dimension;
    static constexpr INTEGER_TYPE NumberOfNodesPerElement = caribou::geometry::traits<Element>::NumberOfNodesAtCompileTime;
    static constexpr INTEGER_TYPE NumberOfGaussNodesPerElement = caribou::geometry::traits<Element>::NumberOfGaussNodesAtCompileTime;

    template<int nRows, int nColumns>
    using Matrix = typename Inherit::template Matrix<nRows, nColumns>;

    template<int nRows>
    using Vector = typename Inherit::template Vector<nRows>;

    using Mat33   = Matrix<3, 3>;
    using Vec3   = Vector<3>;

    template <typename ObjectType>
    using Link = sofa::core::objectmodel::SingleLink<HyperelasticForcefield<Element>, ObjectType, sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK>;

    // Data structures
    struct GaussNode {
        Real weight;
        Real jacobian_determinant;
        Matrix<NumberOfNodesPerElement, Dimension> dN_dx;
        Mat33 F = Mat33::Identity(); // Deformation gradient
    };

    // The container of Gauss points (for each elements) is an array if the number of integration
    // points per element is known at compile time, or a dynamic vector otherwise.
    using GaussContainer = typename std::conditional<
            NumberOfGaussNodesPerElement != caribou::Dynamic,
            std::array<GaussNode, static_cast<std::size_t>(NumberOfGaussNodesPerElement)>,
            std::vector<GaussNode>
    >::type;

    // Public methods

    CARIBOU_API
    HyperelasticForcefield();

    CARIBOU_API
    void init() override;

    CARIBOU_API
    void addForce(
        const sofa::core::MechanicalParams* mparams,
        sofa::core::objectmodel::Data<VecDeriv>& d_f,
        const sofa::core::objectmodel::Data<VecCoord>& d_x,
        const sofa::core::objectmodel::Data<VecDeriv>& d_v) override;

    CARIBOU_API
    void addDForce(
        const sofa::core::MechanicalParams* /*mparams*/,
        sofa::core::objectmodel::Data<VecDeriv>& /*d_df*/,
        const sofa::core::objectmodel::Data<VecDeriv>& /*d_dx*/) override;

    CARIBOU_API
    SReal getPotentialEnergy(
        const sofa::core::MechanicalParams* /* mparams */,
        const sofa::core::objectmodel::Data<VecCoord>& /* d_x */) const override;

    CARIBOU_API
    void addKToMatrix(sofa::defaulttype::BaseMatrix * /*matrix*/, SReal /*kFact*/, unsigned int & /*offset*/) override;

    /** Get the set of Gauss integration nodes of an element */
    auto gauss_nodes_of(std::size_t element_id) const -> const auto & {
        return p_elements_quadrature_nodes[element_id];
    }

    /** Get the complete tangent stiffness matrix as a compressed sparse matrix */
    auto K() -> Eigen::SparseMatrix<Real> {
        using StorageIndex = typename Eigen::SparseMatrix<Real>::StorageIndex;

        // K is symmetric, so we only stored "one side" of the matrix.
        // But to accelerate the computation, coefficients were not
        // stored only in the upper or lower triangular part, but instead
        // in whatever triangular part (upper or lower) the first node
        // index of the element was. This means that a coefficient (i,j)
        // might be in the lower triangular part, while (k,l) is in the
        // upper triangular part. But no coefficient will be both in the
        // lower AND the upper part.

        std::vector<Eigen::Triplet<Real>> triplets;
        triplets.reserve(p_K.size()*2);
        for (StorageIndex k = 0; k < p_K.outerSize(); ++k) {
            for (typename Eigen::SparseMatrix<Real>::InnerIterator it(p_K, k); it; ++it) {
                const auto i = static_cast<StorageIndex>(it.row());
                const auto j = static_cast<StorageIndex>(it.col());
                const auto v = it.value();
                if (i != j) {
                    triplets.emplace_back(i, j, v);
                    triplets.emplace_back(j, i, v);
                } else {
                    triplets.emplace_back(i, i, v);
                }
            }
        }

        Eigen::SparseMatrix<Real> K;
        K.resize(p_K.rows(), p_K.cols());
        K.setFromTriplets(triplets.begin(), triplets.end());

        return K;
    }

    /** Get the eigen values of the tangent stiffness matrix */
    CARIBOU_API
    auto eigenvalues() -> const Vector<Eigen::Dynamic> &;

    /** Get the condition number of the tangent stiffness matrix */
    CARIBOU_API
    auto cond() -> Real;

private:

    // These private methods are implemented but can be overridden

    /** Compute and store the shape functions and their derivatives for every integration points */
    virtual void initialize_elements();

    /** Update the stiffness matrix for every elements */
    virtual void update_stiffness();

    /** Get the set of Gauss integration nodes of the given element */
    virtual auto get_gauss_nodes(const std::size_t & element_id, const Element & element) const -> GaussContainer;

    // Data members
    Link<material::HyperelasticMaterial<DataTypes>> d_material;
    sofa::core::objectmodel::Data<bool> d_enable_multithreading;

    // Private variables
    std::vector<GaussContainer> p_elements_quadrature_nodes;
    Eigen::SparseMatrix<Real> p_K;
    Eigen::Matrix<Real, Eigen::Dynamic, 1> p_eigenvalues;
    bool K_is_up_to_date = false;
    bool eigenvalues_are_up_to_date = false;
};

} // namespace SofaCaribou::forcefield
