#pragma once

#include "Eigen/Geometry"
#include "boost/container/small_vector.hpp"

#include <concepts>
#include <memory>
#include <vector>

namespace ELN
{

// TODO: define a ElementData concept, should it have an embedded boundary?
template <class ElementData, std::floating_point Real_t = float>
class TreeBVH
{
  public:
    struct Node;
    using Tree_t = std::vector<Node>;

    struct ElemIt
    {
        size_t pos;
        Tree_t &bvh;
    };

    struct CElemIt
    {
        size_t pos;
        const Tree_t &bvh;
    };

  private:
    constexpr static size_t m_STATIC_CHILD_NO {4};
    constexpr static size_t m_RESERVE_AMOUNT {128};

    Tree_t m_tree;

  public:
    TreeBVH();

    /**
     * @brief Inserts the element into the BVH
     * @param boundary The bounding box associated to data.
     * @param data The element to be moved into the BVH.
     * @return The iterator of the inserted element from the BVH.
     */
    ElemIt insertElement(Eigen::AlignedBox<Real_t, 3> &&boundary, ElementData &&data);

    /**
     * @brief Finds all elements whose boundary boxes contain the given position.
     * @param position The search point.
     * @return A vector of elements whose boundary boxes contain the given position.
     */
    boost::container::small_vector<const ElementData &, m_STATIC_CHILD_NO>
        findElements(const Eigen::Vector3<Real_t> &position) const;

    /**
     * @brief Finds all elements whose boundary boxes contain the given position.
     * @param position The search point.
     * @return A vector of elements whose boundary boxes contain the given position.
     */
    boost::container::small_vector<ElementData &, m_STATIC_CHILD_NO>
        findElements(const Eigen::Vector3<Real_t> &position);

    /**
     * @brief Finds all elements whose boundary boxes contain the given position.
     * @param position The search point.
     * @return A vector of iterators corresponding to the elements whose boundary boxes contain the
     * given position.
     */
    boost::container::small_vector<ElemIt, m_STATIC_CHILD_NO>
        findElementIts(const Eigen::Vector3<Real_t> &position);

    /**
     * @brief Gets the element corresponding to the given iterator.
     * @param it Iterator to Element.
     * @return The element corresponding to the iterator.
     */
    ElementData &get(ElemIt it);

    /**
     * @brief Gets the element corresponding to the given iterator.
     * @param it Iterator to Element.
     * @return The element corresponding to the iterator.
     */
    const ElementData &get(ElemIt it) const;

    /**
     * @brief Removes the element from the BVH.
     * @param element The element iterator.
     * @return The removed element.
     */
    std::unique_ptr<ElementData> removeElement(ElemIt element);
};

template <class ElementData, std::floating_point Real_t>
struct TreeBVH<ElementData, Real_t>::Node
{
  public:
    Eigen::AlignedBox<Real_t, 3> boundaryBox;
    boost::container::small_vector<ElemIt, m_STATIC_CHILD_NO> children = {};
    ElemIt parent;
    std::unique_ptr<ElementData> elementData {nullptr};

    /**
     * @brief Constructs a new leaf node that will be associated to a set TreeBVH.
     * @param bvh The TreeBVH into which the node will later be inserted.
     * @param boundaryBox The boundary associated to the elementData.
     * @param elementData The elementData of the leaf node.
     */
    Node(TreeBVH<ElementData, Real_t> &bvh, Eigen::AlignedBox<Real_t, 3> &&boundaryBox,
         std::unique_ptr<ElementData> &&elementData)
        : boundaryBox {std::move(boundaryBox)},                   //
          elementData {std::make_unique(std::move(elementData))}, //
          parent {.pos = -1, .bvh = bvh}                          //
    {
    }

    /**
     * @brief Constructs a new internal node or subtree that will be associated to a set TreeBVH.
     * @param bvh The TreeBVH into which the node/subtree will later be inserted.
     * @param boundaryBox The boundary associated to the subtree.
     * @param children The elementData of the leaf node.
     */
    Node(TreeBVH<ElementData, Real_t> &bvh, Eigen::AlignedBox<Real_t, 3> &&boundaryBox = {},
         boost::container::small_vector<ElemIt, m_STATIC_CHILD_NO> &&children = {})
        : boundaryBox {std::move(boundaryBox)}, //
          parent {.pos = -1, .bvh = bvh},       //
          children {std::move(children)}        //
    {
    }
};

template <class ElementData, std::floating_point Real_t>
TreeBVH<ElementData, Real_t>::ElemIt
    TreeBVH<ElementData, Real_t>::insertElement(Eigen::AlignedBox<Real_t, 3> &&boundary,
                                                ElementData &&data)
{
    Node node {.boundaryBox = std::move(boundary), .children = {}, .elementData = std::move(data)};
    if (m_tree.empty())
        m_tree.emplace_back()
}
} // namespace ELN
