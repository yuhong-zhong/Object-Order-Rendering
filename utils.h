#ifndef OBJECT_ORDER_RENDERING_UTILS_H
#define OBJECT_ORDER_RENDERING_UTILS_H

#include <Eigen/Dense>

using namespace Eigen;

Vector2f getOrthogonal(Vector2f v) {
    Vector2f result(v[1], (-1 * v[0]));
    return result;
}


#endif
