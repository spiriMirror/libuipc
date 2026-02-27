#pragma once
#include <iostream>
#include <muda/muda_def.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>

namespace muda::spatial_hash
{
class CollisionPair
{
    Eigen::Vector2i id;

  public:
    MUDA_GENERIC Eigen::Vector2i IDs() const { return id; }

    MUDA_GENERIC CollisionPair(int i, int j)
    {
        if(i > j)
        {
            id[0] = j;
            id[1] = i;
        }
        else
        {
            id[0] = i;
            id[1] = j;
        }
    }

    MUDA_GENERIC CollisionPair()
        : id(-1, -1)
    {
    }

    MUDA_GENERIC friend bool operator<(const CollisionPair& l, const CollisionPair& r)
    {
        return (l.id[0] < r.id[0]) || (l.id[0] == r.id[0] && l.id[1] < r.id[1]);
    }

    MUDA_GENERIC friend bool operator==(const CollisionPair& l, const CollisionPair& r)
    {
        return (l.id[0] == r.id[0] && l.id[1] == r.id[1]);
    }

    MUDA_GENERIC bool is_valid() const { return id[0] != -1 && id[1] != -1; }

    friend std::ostream& operator<<(std::ostream& os, const CollisionPair& c)
    {
        os << "(" << c.id[0] << "," << c.id[1] << ")";
        return os;
    }

    MUDA_GENERIC static CollisionPair invalid()
    {
        return CollisionPair(-1, -1);
    }
};
}  // namespace muda::spatial_hash
