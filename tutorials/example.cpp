

#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>

#include "collision/sphere.h"
#include "collision/rectangle_aabb.h"
#include "collision/rectangle_obb.h"
#include "collision/point.h"

#include "collision/collision_checker.h"

using namespace collision;

#include "example.h"

int example_main() {
  using SpherePtr_t = std::shared_ptr<collision::Sphere>;
  SpherePtr_t Sphere1(new collision::Sphere(1, 100, 100));
  SpherePtr_t Sphere2(new collision::Sphere(5, 0, 5));
  std::cout << "Performing collision btw. two spheres: " << "\n";
  std::cout << Sphere1->collide(*Sphere2);
  std::cout << Sphere1->collide(*Sphere2);

  Sphere2->set_radius(6);
  std::cout << Sphere1->collide(*Sphere2);

  using AABBPtr_t = std::shared_ptr<collision::RectangleAABB>;
  using OBBPtr_t = std::shared_ptr<collision::RectangleOBB>;
  using TrianglePtr_t = std::shared_ptr<collision::Triangle>;

  double rad1 = 1;

  AABBPtr_t Box1(new collision::RectangleAABB(rad1, rad1, Eigen::Vector2d(0, 0)));
  AABBPtr_t Box7(new collision::RectangleAABB(rad1, rad1, Eigen::Vector2d(0, 1)));

  const double pi = std::acos(-1);
  OBBPtr_t Box2(new collision::RectangleOBB(2*rad1, 2*rad1, pi/12, Eigen::Vector2d(3*rad1 + 0.1, -1*rad1)));

  TrianglePtr_t
      triangle1(new collision::Triangle(Eigen::Vector2d(0, 1), Eigen::Vector2d(10, 0), Eigen::Vector2d(4, 5)));
  TrianglePtr_t
      triangle2(new collision::Triangle(Eigen::Vector2d(0, 2), Eigen::Vector2d(10, 1), Eigen::Vector2d(4, 6)));

  bool res;

  res = triangle1->collide(*Box2);

  SpherePtr_t sphr_1(new collision::Sphere(2.5, 6, 7));

  collision::CollisionChecker cc11;
  RectangleAABBPtr aabb11(new collision::RectangleAABB(2, 3, Eigen::Vector2d(8, 10)));
  RectangleOBBPtr obb11(new collision::RectangleOBB(1, 2, 0.3, Eigen::Vector2d(8, 10)));

  TrianglePtr_t triang3(new collision::Triangle(Eigen::Vector2d(0, 1), Eigen::Vector2d(10, 0), Eigen::Vector2d(4, 5)));
  TrianglePtr_t triang2(new collision::Triangle(Eigen::Vector2d(10, 2), Eigen::Vector2d(2, 2), Eigen::Vector2d(5, 5)));

  std::vector<TriangleConstPtr> triangles;
  triangles.push_back(triang3);
  triangles.push_back(triang2);
  std::vector<std::vector<Eigen::Vector2d> > hole_vert;
  std::vector<Eigen::Vector2d> vert;
  PolygonPtr polyg11(new collision::Polygon(vert, hole_vert, triangles));

  cc11.addCollisionObject(polyg11);
  cc11.addCollisionObject(sphr_1);
  cc11.addCollisionObject(aabb11);
  cc11.addCollisionObject(obb11);

  std::vector<LineSegment> v1;

  res = cc11.RayTrace(Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 10), v1);

  res = Box1->collide(*Box2);

  collision::RectangleAABBConstPtr aabcptr = Box1->getAABB();

  std::cout << "\n" << res << "\n";

  collision::CollisionChecker col_checker;
  long int numtests = 0;

  TimeVariantCollisionObjectPtr tvo1(new TimeVariantCollisionObject(1));

  tvo1->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(2.0, 5))));
  tvo1->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(2.5, 5))));
  tvo1->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(3, 5))));
  tvo1->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(3.5, 5))));
  tvo1->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(4, 5))));
  tvo1->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(4.5, 5))));
  tvo1->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(5, 5))));
  tvo1->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(5.5, 5))));

  TimeVariantCollisionObjectPtr tvo2(new TimeVariantCollisionObject(4));
  tvo2->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 1.5, Eigen::Vector2d(6.0, 0))));
  tvo2->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 1.5, Eigen::Vector2d(6, 2))));
  tvo2->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 1.5, Eigen::Vector2d(6, 3))));
  tvo2->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 1.5, Eigen::Vector2d(6, 4))));
  tvo2->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 1.5, Eigen::Vector2d(6, 5))));
  tvo2->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 1.5, Eigen::Vector2d(6, 6))));
  tvo2->appendObstacle(CollisionObjectConstPtr(new collision::RectangleOBB(2, 1, 1.5, Eigen::Vector2d(6, 7))));

  CollisionChecker cc;
  ShapeGroupPtr sg10(new ShapeGroup);
  RectangleOBBConstPtr robb1(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(2.0, 5)));
  sg10->addToGroup(robb1);
  sg10->addToGroup(RectangleOBBConstPtr(new collision::RectangleOBB(2, 1, 0.0, Eigen::Vector2d(2.5, 5))));
  cc.addCollisionObject(robb1);
  cc.addCollisionObject(sg10);

  cc.addCollisionObject(tvo2);
  std::vector<CollisionObjectConstPtr> obsts;
  res = cc.collide(tvo1, obsts, true);
  res = cc.collide(tvo1, obsts, true, true);
  CollisionObjectConstPtr ab;
  res = cc.collide(tvo1, ab, true, true);
  res = cc.collide(tvo1);
  res = tvo1->collide(*tvo2);

  SpherePtr_t Sphere3(new collision::Sphere(2.5, Eigen::Vector2d(5, 6)));

  SpherePtr_t Sphere8(new collision::Sphere(2.5, Eigen::Vector2d(5, 6)));

  SpherePtr_t Sphere9(new collision::Sphere(2.5, Eigen::Vector2d(5, 7)));

  OBBPtr_t Box3(new collision::RectangleOBB(1, 2, 0.1, Eigen::Vector2d(9, 10)));
  OBBPtr_t Box4(new collision::RectangleOBB(1, 1.3, 0.6, Eigen::Vector2d(10.6, 1)));


  using PointPtr_t = std::shared_ptr<collision::Point>;
  PointPtr_t point1(new collision::Point(Eigen::Vector2d(9, 10)));
  res = point1->collide(*Box3);

  AABBPtr_t Box_aabb1(new collision::RectangleAABB(2, 3, Eigen::Vector2d(3, 1.8)));

  using ShapeGroupPtr_t = std::shared_ptr<collision::ShapeGroup>;
  ShapeGroupPtr_t sh_g1(new ShapeGroup());
  sh_g1->addToGroup(Sphere3);
  sh_g1->addToGroup(Box_aabb1);

  ShapeGroupPtr_t sh_g2(new ShapeGroup());
  sh_g2->addToGroup(Box2);
  sh_g2->addToGroup(Box4);

  auto res2 = sh_g1->overlap(*sh_g2);
  auto res3 = sh_g1->overlap_map(*sh_g2);

  res = sh_g1->collide(*sh_g2);

  col_checker.addCollisionObject(Box2);
  col_checker.addCollisionObject(Box3);
  col_checker.addCollisionObject(Box4);
  col_checker.addCollisionObject(Sphere3);

  return 0;
}
