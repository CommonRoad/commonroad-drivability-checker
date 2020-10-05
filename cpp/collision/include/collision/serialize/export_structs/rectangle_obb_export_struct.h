#pragma once
namespace collision {
namespace serialize {
struct RectangleOBBExportStruct {
 public:
  double rx;
  double ry;
  double center_x;
  double center_y;
  double local_axis_x_1;
  double local_axis_y_1;
  double local_axis_x_2;
  double local_axis_y_2;
};
}  // namespace serialize
}  // namespace collision
