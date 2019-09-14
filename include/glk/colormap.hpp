#ifndef GLK_COLORMAP_HPP
#define GLK_COLORMAP_HPP

#include <Eigen/Core>

namespace glk {

enum COLORMAP_TYPE { TURBO = 0, NUM_COLORMAPS };

Eigen::Vector4i colormap(COLORMAP_TYPE type, int x);
Eigen::Vector4f colormapf(COLORMAP_TYPE type, float x);
Eigen::Vector4i colormap_categorical(COLORMAP_TYPE type, int x, int num_categories);
Eigen::Vector4f colormap_categoricalf(COLORMAP_TYPE type, int x, int num_categories);
}  // namespace glk

#endif