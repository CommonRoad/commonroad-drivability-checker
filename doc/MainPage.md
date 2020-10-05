Introduction

The collision checker provides you the functionality to check if basic geometric shapes and groups of shapes collide. Currently several basic shapes are available: axis-aligned rectangles (RectangleAABB), oriented rectangles (RectangleOBB), triangles (Triangle), circles (Sphere), and polygons (Polygon). The most basic intersection test can be performed between these primitive shapes. 

The shapes can be grouped into a ShapeGroup. Grouping objects builds an accelerator structure, such as a binary tree, to efficiently filter candidate objects for collision.

All primitive shapes and a ShapeGroup can be added into a TimeVariantCollisionObject that has one enity per time step.

Trajectory is kind of a TimeVariantCollisionObject that consists of oriented rectangles and allows to interpolate between their positions and orientations at different time steps.

CollisionChecker can contain objects of all above mentioned types. It builds an accelerator structure to efficiently filter candidate objects for collision.

Warning: the library is intended for research.
API for internally used functions at deeper level of abstraction is subject to change in coming versions.

The CollisionChecker library is single-threaded and not thread-safe.
The FCL library is not thread-safe, unless a separate object descriptor is created for each thread.





