#include "TreeBVH.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace ELN::BVH;

TEST_CASE("BVH single element", "[BVH]")
{
    Eigen::Vector3f min {{-1, -1, -1}};
    Eigen::Vector3f max {{1, 1, 1}};
    std::vector<CompactTree<size_t, float>::ElemBdrPair> elements = {
        {.elementData = 0, .boundary = {min, max}},
    };

    const CompactTree<size_t> tree {std::move(elements)};

    auto result = tree.queryPosition(min);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition(max);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition((min + max) / 2.0f);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition(max * 1.01f);
    REQUIRE(result.size() == 0);
}

TEST_CASE("BVH two disjoint elements", "[BVH]")
{

    Eigen::Vector3f min1 {{-2, -2, -2}};
    Eigen::Vector3f max1 {{-1, -1, -1}};
    Boundary_t<float> boundary1 {min1, max1};

    Eigen::Vector3f min2 {{1, 1, 1}};
    Eigen::Vector3f max2 {{2, 2, 2}};
    Boundary_t<float> boundary2 {min2, max2};

    std::vector<CompactTree<size_t, float>::ElemBdrPair> elements = {
        {.elementData = 0, .boundary = boundary1},
        {.elementData = 1, .boundary = boundary2},
    };
    const CompactTree<size_t> tree {std::move(elements)};

    auto result = tree.queryPosition(min1);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition(max1);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition((min1 + max1) / 2.0f);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition(min2);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 1);

    result = tree.queryPosition(max2);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 1);

    result = tree.queryPosition((min2 + max2) / 2.0f);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 1);

    result = tree.queryPosition(Eigen::Vector3f {{0.0f, 0.0f, 0.0f}});
    REQUIRE(result.size() == 0);
}

TEST_CASE("BVH two intersecting elements - one point", "[BVH]")
{

    Eigen::Vector3f min1 {{-2, -2, -2}};
    Eigen::Vector3f max1 {{0, 0, 0}};
    Boundary_t<float> boundary1 {min1, max1};

    Eigen::Vector3f min2 {{0, 0, 0}};
    Eigen::Vector3f max2 {{2, 2, 2}};
    Boundary_t<float> boundary2 {min2, max2};

    std::vector<CompactTree<size_t, float>::ElemBdrPair> elements = {
        {.elementData = 0, .boundary = boundary1},
        {.elementData = 1, .boundary = boundary2},
    };

    const CompactTree<size_t> tree {std::move(elements)};

    auto result = tree.queryPosition(min1);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition(max1);
    REQUIRE(result.size() == 2);
    REQUIRE((result[0] == 0 or result[1] == 0));
    REQUIRE((result[0] == 1 or result[1] == 1));

    result = tree.queryPosition((min1 + max1) / 2.0f);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition(min2);
    REQUIRE(result.size() == 2);
    REQUIRE((result[0] == 1 or result[1] == 1));
    REQUIRE((result[0] == 0 or result[1] == 0));

    result = tree.queryPosition(max2);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 1);

    result = tree.queryPosition((min2 + max2) / 2.0f);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 1);

    result = tree.queryPosition(Eigen::Vector3f {{10.0f, 10.0f, 10.0f}});
    REQUIRE(result.size() == 0);
}

TEST_CASE("BVH two intersecting elements - 3d", "[BVH]")
{

    Eigen::Vector3f min1 {{-2, -2, -2}};
    Eigen::Vector3f max1 {{1, 1, 1}};
    Boundary_t<float> boundary1 {min1, max1};

    Eigen::Vector3f min2 {{-1, -1, -1}};
    Eigen::Vector3f max2 {{2, 2, 2}};
    Boundary_t<float> boundary2 {min2, max2};

    std::vector<CompactTree<size_t, float>::ElemBdrPair> elements = {
        {.elementData = 0, .boundary = boundary1},
        {.elementData = 1, .boundary = boundary2},
    };
    const CompactTree<size_t> tree {std::move(elements)};

    auto result = tree.queryPosition(min1);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 0);

    result = tree.queryPosition(max1);
    REQUIRE(result.size() == 2);
    REQUIRE((result[0] == 0 or result[1] == 0));
    REQUIRE((result[0] == 1 or result[1] == 1));

    result = tree.queryPosition((min1 + max1) / 2.0f);
    REQUIRE(result.size() == 2);
    REQUIRE((result[0] == 0 or result[1] == 0));
    REQUIRE((result[0] == 1 or result[1] == 1));

    result = tree.queryPosition(min2);
    REQUIRE(result.size() == 2);
    REQUIRE((result[0] == 1 or result[1] == 1));
    REQUIRE((result[0] == 0 or result[1] == 0));

    result = tree.queryPosition(max2);
    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == 1);

    result = tree.queryPosition((min2 + max2) / 2.0f);
    REQUIRE(result.size() == 2);
    REQUIRE((result[0] == 1 or result[1] == 1));
    REQUIRE((result[0] == 0 or result[1] == 0));

    result = tree.queryPosition(Eigen::Vector3f {{10.0f, 10.0f, 10.0f}});
    REQUIRE(result.size() == 0);
}

TEST_CASE("BVH boundary inside boundary", "[BVH]")
{

    constexpr size_t noElements {100};

    std::vector<CompactTree<size_t, float>::ElemBdrPair> elements;
    elements.reserve(noElements);
    for (size_t i {1}; i <= noElements; ++i)
    {
        Eigen::Vector3f min {{
            -1.0f * static_cast<float>(i),
            -1.0f * static_cast<float>(i),
            -1.0f * static_cast<float>(i),
        }};
        Eigen::Vector3f max {{
            static_cast<float>(i),
            static_cast<float>(i),
            static_cast<float>(i),
        }};
        Boundary_t<float> boundary {min, max};
        elements.emplace_back(i, boundary);
    }

    const CompactTree<size_t> tree {std::move(elements)};

    for (size_t k {0}; k < noElements; ++k)
    {
        auto queryRes = tree.queryPosition(Eigen::Vector3f {{k * 1.0f + 0.5f, 0.0f, 0.0f}});
        REQUIRE(queryRes.size() == noElements - k);
    }
}

TEST_CASE("BVH medium amount of elements", "[BVH]")
{

    constexpr size_t width {7}, height {11}, depth {13};
    constexpr float size {1.0f};

    std::vector<CompactTree<size_t, float>::ElemBdrPair> elements;
    for (size_t z {0}, i {0}; z < depth; ++z)
        for (size_t y {0}; y < height; ++y)
            for (size_t x {0}; x < width; ++x, ++i)
            {
                Eigen::Vector3f min {{
                    (x - 0.55f) * size,
                    (y - 0.55f) * size,
                    (z - 0.55f) * size,
                }};
                Eigen::Vector3f max {{
                    (x + 0.55f) * size,
                    (y + 0.55f) * size,
                    (z + 0.55f) * size,
                }};

                Boundary_t<float> boundary {min, max};
                elements.emplace_back(i, boundary);
            }

    const CompactTree<size_t> tree {std::move(elements)};

    for (size_t z {1}, data {width + height}; z < depth - 1; ++z)
        for (size_t y {1}; y < height - 1; ++y)
            for (size_t x {1}; x < width - 1; ++x, ++data)
            {
                Eigen::Vector3f middle {{
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z),
                }};
                Eigen::Vector3f minMid {{
                    (x - 0.50f) * size,
                    (y - 0.50f) * size,
                    (z - 0.50f) * size,
                }};
                Eigen::Vector3f maxMid {{
                    (x + 0.50f) * size,
                    (y + 0.50f) * size,
                    (z + 0.50f) * size,
                }};

                auto queryRes = tree.queryPosition(middle);
                REQUIRE(queryRes.size() == 1);

                queryRes = tree.queryPosition(minMid);
                REQUIRE(queryRes.size() == 8);

                queryRes = tree.queryPosition(maxMid);
                REQUIRE(queryRes.size() == 8);
            }
}
