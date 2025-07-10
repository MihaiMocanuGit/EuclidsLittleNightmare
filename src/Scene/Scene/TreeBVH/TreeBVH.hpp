#pragma once

#include "Eigen/Geometry"
#include "Eigen/src/Geometry/AlignedBox.h"
#include "boost/container/small_vector.hpp"

#include <concepts>
#include <memory>
#include <numeric>
#include <vector>

namespace ELN
{

namespace BVH
{
constexpr int CHILDREN_NO {4};
constexpr size_t npos {static_cast<size_t>(-1)};

template <std::floating_point Real_t = float>
using Boundary_t = Eigen::AlignedBox<Real_t, 3>;

template <class ElementData, std::floating_point Real_t>
class CompactTree;

template <class ElementData, std::floating_point Real_t = float>
class DiffuseTree
{
  private:
    struct Node
    {
        Eigen::AlignedBox<Real_t, 3> boundary;
        std::vector<std::unique_ptr<Node>> children;
        Node *parent {nullptr}; // non owning pointer
        std::unique_ptr<ElementData> data {nullptr};
    };

    std::unique_ptr<Node> m_root {nullptr};
    size_t m_nodeCount {0};

    friend class CompactTree<ElementData, Real_t>;

  public:
    DiffuseTree() = default;

    void build(std::vector<ElementData> &&elements);
};

template <class ElementData, std::floating_point Real_t = float>
class CompactTree
{
  private:
    struct Node
    {
        Boundary_t<Real_t> boundary;
        std::vector<size_t> children;
        size_t parent {npos};
        size_t data {npos};
    };

    std::vector<Node> m_tree;
    std::vector<ElementData> m_elements;
    std::vector<std::reference_wrapper<Boundary_t<Real_t>>> m_boundaries;

    uint64_t m_mortonOrder(Real_t xPos, Real_t yPos, Real_t zPos, unsigned fractionDigits = 1);

  public:
    CompactTree() = default;

    void build(std::vector<ElementData> &&elements, std::vector<Boundary_t<Real_t>> &&boundaries);

    std::vector<std::reference_wrapper<ElementData>>
        queryPosition(const Eigen::Vector3<Real_t> &position);
};

template <class ElementData, std::floating_point Real_t>
uint64_t CompactTree<ElementData, Real_t>::m_mortonOrder(Real_t xPos, Real_t yPos, Real_t zPos,
                                                         unsigned fractionDigits)
{
    Real_t multiplier = 1;
    for (unsigned i {0}; i < fractionDigits; ++i)
        multiplier *= 10;
    uint64_t x {static_cast<uint64_t>(xPos * multiplier)};
    uint64_t y {static_cast<uint64_t>(yPos * multiplier)};
    uint64_t z {static_cast<uint64_t>(zPos * multiplier)};

    // The mordon code for 3d coords looks like this:
    // __00____01____02____03____04____05__....___61____62____63_
    // x[00]_y[00]_z[00]_x[01]_y[01]_z[01]_...._x[20]_y[20]_z[20]
    // TODO: extend this to a 64+64+64 byte integer? (Endianness will become important)

    uint64_t coordMask {0x1};
    uint64_t coordByte {0};
    uint64_t mordonByte {0};
    uint64_t mordon {0x0};
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
void CompactTree<ElementData, Real_t>::build(std::vector<ElementData> &&elements,
                                             std::vector<Boundary_t<Real_t>> &&boundaries)
{
    std::vector<size_t> permIndexes(elements.size(), 0);
    std::iota(permIndexes.begin(), permIndexes.end(), 0);

    // Morton sort the elements relative to the center of their bounding boxes
    std::sort(permIndexes.begin(), permIndexes.end(),
              [&](auto left, auto right)
              {
                  const auto &centerLeft = boundaries[left].center();
                  const auto &centerRight = boundaries[right].center();

                  uint64_t mordonLeft {m_mortonOrder(centerLeft[0], centerLeft[1], centerLeft[2])};

                  uint64_t mordonRight {
                      m_mortonOrder(centerRight[0], centerRight[1], centerRight[2])};

                  return mordonLeft < mordonRight;
              });
}
} // namespace BVH
} // namespace ELN
