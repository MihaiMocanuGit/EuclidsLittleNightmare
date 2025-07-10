#pragma once

#include "Eigen/Geometry"
#include "Utils/utils.hpp"

#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <execution>
#include <functional>
#include <ranges>
#include <stack>
#include <variant>
#include <vector>

namespace ELN
{

namespace BVH
{
constexpr size_t CHILDREN_NO {4};
constexpr size_t npos {static_cast<size_t>(-1)};

template <std::floating_point Real_t = float>
using Boundary_t = Eigen::AlignedBox<Real_t, 3>;

template <class ElementData, std::floating_point Real_t = float>
class CompactTree
{
  private:
    struct Node
    {
        using Children_t = std::array<size_t, CHILDREN_NO>;
        using Data_t = size_t;

        Boundary_t<Real_t> boundary {};
        std::variant<Children_t, Data_t> downstream {Utils::array_fill<CHILDREN_NO>(npos)};
        size_t parent {npos};
    };

    std::vector<Node> m_tree;
    size_t m_elementsIndexBegin {npos};
    size_t m_elementsIndexEnd {npos};
    static constexpr size_t m_ROOT_INDEX {0};

    std::vector<ElementData> m_elements;

    constexpr static uint64_t m_mortonOrder(Real_t xPos, Real_t yPos, Real_t zPos,
                                            unsigned fractionDigits = 1) noexcept;
    bool m_isNodeEmpty(size_t nodeIndex) const;
    bool m_isNodeLeaf(size_t nodeIndex) const;

  public:
    struct ElemBdrPair
    {
        ElementData elementData;
        Boundary_t<Real_t> boundary;
    };
    CompactTree(std::vector<ElemBdrPair> &&elementsWithBoundaries);

    // TODO:: boost::container::small_vector<>?
    using QueryResult_t = std::vector<std::reference_wrapper<const ElementData>>;

    QueryResult_t queryPosition(const Eigen::Vector3<Real_t> &position) const;

    const std::vector<ElementData> &elements() const noexcept;
};

template <class ElementData, std::floating_point Real_t>
constexpr uint64_t CompactTree<ElementData, Real_t>::m_mortonOrder(Real_t xPos, Real_t yPos,
                                                                   Real_t zPos,
                                                                   unsigned fractionDigits) noexcept
{
    Real_t multiplier {static_cast<Real_t>(Utils::int_pow(10U, fractionDigits))};
    const uint64_t x {static_cast<uint64_t>(xPos * multiplier)};
    const uint64_t y {static_cast<uint64_t>(yPos * multiplier)};
    const uint64_t z {static_cast<uint64_t>(zPos * multiplier)};

    // The mordon code for 3d coords looks like this:
    // __00____01____02____03____04____05__....___61____62____63_
    // x[00]_y[00]_z[00]_x[01]_y[01]_z[01]_...._x[20]_y[20]_z[20]
    // TODO: extend this to a 64+64+64 byte integer? (Endianness will become important)

    uint64_t coordMask {0x1};
    uint64_t coordByte {0x0};
    uint64_t mordonByte {0x0};
    uint64_t mordon {0x0};
    // TODO: Optimisation: make this O(1)
    while (mordonByte < 63)
    {
        mordon |= ((coordMask & x) >> coordByte) << (mordonByte++);
        mordon |= ((coordMask & y) >> coordByte) << (mordonByte++);
        mordon |= ((coordMask & z) >> coordByte) << (mordonByte++);
        coordByte++;
        coordMask <<= 1;
    }
    return mordon;
}

template <class ElementData, std::floating_point Real_t>
inline bool CompactTree<ElementData, Real_t>::m_isNodeEmpty(size_t nodeIndex) const
{
    assert(nodeIndex < m_tree.size());
    return m_tree[nodeIndex].boundary.isEmpty();
}

template <class ElementData, std::floating_point Real_t>
inline bool CompactTree<ElementData, Real_t>::m_isNodeLeaf(size_t nodeIndex) const
{
    assert(nodeIndex < m_tree.size());
    return m_elementsIndexBegin <= nodeIndex && nodeIndex < m_elementsIndexEnd;
}

template <class ElementData, std::floating_point Real_t>
CompactTree<ElementData, Real_t>::CompactTree(std::vector<ElemBdrPair> &&elementsWithBoundaries)
{
    // sort the elements relative to any inclusion relation and the center of their bounding boxes
    std::sort(
        std::execution::par_unseq, elementsWithBoundaries.begin(), elementsWithBoundaries.end(),
        [&](const auto &left, const auto &right)
        {
            if (right.boundary.contains(left.boundary)) // if left < right
                return true;
            else if (left.boundary.contains(right.boundary)) // if left > right
                return false;
            //  else use the mordon parametrization of the center coords.
            const auto &centerLeft = left.boundary.center();
            const auto &centerRight = right.boundary.center();

            // The Real_t values will be converted to unsigned integers in order to apply bit
            // operations. Thus, the real coordinates will be translated such that they all are
            // greater than 0

            Eigen::Vector3f offset {{0.0f}, {0.0f}, {0.0f}};
            for (unsigned dim {0u}; dim < 3u; ++dim)
            {
                Real_t minValue {std::min(centerLeft[dim], centerRight[dim])};
                Real_t offsetValue {0.0};
                if (minValue < 0.0)
                    offsetValue =
                        -minValue + 1.0; // Adding 1 to counteract any small floating point errors
                offset[dim] = offsetValue;
            }

            uint64_t mordonLeft {m_mortonOrder(centerLeft[0] + offset[0], centerLeft[1] + offset[1],
                                               centerLeft[2] + offset[2])};

            uint64_t mordonRight {m_mortonOrder(centerRight[0] + offset[0],
                                                centerRight[1] + offset[1],
                                                centerRight[2] + offset[2])};

            return mordonLeft < mordonRight;
        });

    const size_t ELEMENT_SIZE {elementsWithBoundaries.size()};
    m_elements = std::vector<ElementData>(ELEMENT_SIZE);
    const auto ELEM_VIEW = std::ranges::iota_view {(size_t)0, ELEMENT_SIZE};
    // an array of ordered indices (0, 1, 2, ... n-1) is useful for parallelisation.
    const std::vector<size_t> ELEM_INDICES(ELEM_VIEW.begin(), ELEM_VIEW.end());
    std::for_each(
        std::execution::par_unseq, ELEM_INDICES.begin(), ELEM_INDICES.end(),
        [this, &elementsWithBoundaries](size_t elemIndex)
        { m_elements[elemIndex] = std::move(elementsWithBoundaries[elemIndex].elementData); });

    // Find the n-th term of the geometric series with base = CHILDREN_NO. The last term holds
    // the leaf nodes in the CHILDREN_NO-th tree.
    size_t nTerm {1}, n {0};
    while (nTerm < ELEMENT_SIZE)
    {
        nTerm *= CHILDREN_NO;
        ++n;
    }
    // The sum of the geometric series of order n - 1 and base CHILDREN_NO
    size_t internalNodes {(Utils::int_pow(CHILDREN_NO, n) - 1) / (CHILDREN_NO - 1)};

    m_elementsIndexBegin = internalNodes;
    m_elementsIndexEnd = m_elementsIndexBegin + ELEMENT_SIZE;
    // Fill the leaf nodes first and leave the internalNodes blank
    m_tree.resize(m_elementsIndexEnd);
    std::for_each(std::execution::par_unseq, ELEM_INDICES.begin(), ELEM_INDICES.end(),
                  [&, this](size_t elemIndex)
                  {
                      Node node {
                          .boundary = std::move(elementsWithBoundaries[elemIndex].boundary),
                          .downstream = elemIndex, // only contains an iterator to its data, no kids
                          .parent = npos,          // to be filled later
                      };
                      m_tree[m_elementsIndexBegin + elemIndex] = std::move(node);
                  });

    // Now fill the internal nodes backwards
    for (size_t depth {n - 1}; depth < n; --depth)
    {
        // The current depth level starts at position k_1Sum and stops at kSum - 1.
        size_t kSum {(Utils::int_pow(CHILDREN_NO, depth + 1) - 1) / (CHILDREN_NO - 1)};
        size_t k_1Sum {(Utils::int_pow(CHILDREN_NO, depth) - 1) / (CHILDREN_NO - 1)};
        // TODO:Optimisation: this could be parallelized.
        for (size_t parent {k_1Sum}; parent < kSum; ++parent)
        {
            // init with npos for the case when the internal node does not hold the max amount
            // of children
            std::array<size_t, CHILDREN_NO> children {Utils::array_fill<CHILDREN_NO>(npos)};
            Boundary_t<Real_t> parentBoundary {};
            size_t indexInLevel {parent - k_1Sum};
            for (size_t child {kSum + indexInLevel * CHILDREN_NO}, i {0};
                 child < kSum + (indexInLevel + 1) * CHILDREN_NO and child < m_tree.size(); ++child)
            {
                children[i++] = child;
                m_tree[child].parent = parent;
                if (parentBoundary.isEmpty())
                    parentBoundary = m_tree[child].boundary;
                else
                    parentBoundary = parentBoundary.merged(m_tree[child].boundary);
            }

            Node parentNode {
                .boundary = parentBoundary,
                .downstream = children, // only contains kids, no iterator to data
                .parent = npos,         // to be filled in the next iteration
            };
            m_tree[parent] = std::move(parentNode);
        }
    }
}

template <class ElementData, std::floating_point Real_t>
std::vector<std::reference_wrapper<const ElementData>>
    CompactTree<ElementData, Real_t>::queryPosition(const Eigen::Vector3<Real_t> &position) const
{
    assert(m_tree.size() > 0);
    assert(not m_isNodeEmpty(m_ROOT_INDEX));
    QueryResult_t result {};
    // special case for when the tree contains only one elementData. In this context, one cannot
    // iterate through the children as there are no such elements
    if (m_tree.size() == 1)
    {
        assert(m_isNodeLeaf(m_ROOT_INDEX));
        if (m_tree[m_ROOT_INDEX].boundary.contains(position))
        {
            const auto &dataIndex =
                std::get<typename Node::Data_t>(m_tree[m_ROOT_INDEX].downstream);
            result.push_back(std::ref(m_elements[dataIndex]));
        }
        return result;
    }

    std::stack<size_t> visitIndexes;
    visitIndexes.push(m_ROOT_INDEX);
    // TODO: Optimisation: Write a better parallel search algorithm
    while (not visitIndexes.empty())
    {
        size_t current {visitIndexes.top()};
        visitIndexes.pop();
        std::for_each(std::execution::unseq,
                      std::get<typename Node::Children_t>(m_tree[current].downstream).begin(),
                      std::get<typename Node::Children_t>(m_tree[current].downstream).end(),
                      [&](size_t child)
                      {
                          if (child == npos)
                              return;
                          if (not m_isNodeEmpty(child) and
                              m_tree[child].boundary.contains(position))
                          {
                              if (m_isNodeLeaf(child))
                              {
                                  const auto &dataIndex =
                                      std::get<typename Node::Data_t>(m_tree[child].downstream);
                                  result.push_back(std::ref(m_elements[dataIndex]));
                              }
                              else
                                  visitIndexes.push(child);
                          }
                      });
    }
    return result;
}

template <class ElementData, std::floating_point Real_t>
inline const std::vector<ElementData> &CompactTree<ElementData, Real_t>::elements() const noexcept
{
    return m_elements;
}

} // namespace BVH
} // namespace ELN
