# import matplotlib.pyplot as plt

import commonroad_dc.pycrcc as pycrcc

import unittest


# Test TVObstacles with each other and ShapeGroups
# Test CollisionChecker
# Test ShapeGroups

class TestCollision(unittest.TestCase):

    def test_AABBCircle(self):
        aabb = pycrcc.RectAABB(2, 3, 3, 1.8)
        aabb2 = pycrcc.RectAABB(2, 3, 3, 1.7)
        circ = pycrcc.Circle(2.5, 6, 7)

        self.assertEqual(aabb.collide(circ), 1)
        self.assertEqual(aabb2.collide(circ), 0)

    def test_OBBCircle(self):
        obb1 = pycrcc.RectOBB(1, 2, 0.2, 9, 10)
        obb2 = pycrcc.RectOBB(1, 2, 0.1, 9, 10)
        circ = pycrcc.Circle(2.5, 6, 7)

        self.assertEqual(obb1.collide(circ), 0)
        self.assertEqual(obb2.collide(circ), 1)

    def test_OBBAABB(self):
        obb2 = pycrcc.RectOBB(1, 2, 0, 9, 14)
        aabb2 = pycrcc.RectAABB(1, 1, 11.1, 16)
        obb3 = pycrcc.RectOBB(1, 2, 0.2, 9, 14)
        obb4 = pycrcc.RectOBB(1, 2, -0.2, 9, 14)

        self.assertEqual(obb2.collide(aabb2), 0)
        self.assertEqual(obb3.collide(aabb2), 0)
        self.assertEqual(obb4.collide(aabb2), 1)

    def test_triangle(self):
        triang = pycrcc.Triangle(0, 0, 10, 0, 4, 5)
        circ1 = pycrcc.Circle(2.5, 6, 7)
        self.assertEqual(triang.collide(circ1), 0)
        circ2 = pycrcc.Circle(2.5, 5, 7)
        self.assertEqual(triang.collide(circ2), 1)
        obb3 = pycrcc.RectOBB(1, 2, 0.6, 9, 3)
        self.assertEqual(triang.collide(obb3), 0)
        obb4 = pycrcc.RectOBB(1, 2, 0.4, 9, 3)
        self.assertEqual(triang.collide(obb4), 1)

        triang = pycrcc.Triangle(0, 0, 10, 0, 4, 5)

        triang2 = pycrcc.Triangle(10, 2, 2, 2, 5, 5)

        obb5 = pycrcc.RectOBB(1, 1.3, 0.6, 10.8, 0.8)

        self.assertEqual(triang.collide(obb5), 0)
        self.assertEqual(triang2.collide(obb5), 0)

        obb6 = pycrcc.RectOBB(1, 1.3, 0.6, 10.6, 0.6)
        self.assertEqual(triang.collide(obb6), 1)

        obb7 = pycrcc.RectOBB(1, 1.3, 0.6, 10.8, 0.6)
        self.assertEqual(triang.collide(obb7), 1)
        self.assertEqual(triang2.collide(obb7), 0)

        obb8 = pycrcc.RectOBB(1, 1.3, 0.6, 10.6, 1)

        self.assertEqual(triang.collide(obb8), 0)
        self.assertEqual(triang2.collide(obb8), 1)

    def test_polyg(self):
        triang = pycrcc.Triangle(0, 0, 10, 0, 4, 5)

        triang2 = pycrcc.Triangle(10, 2, 2, 2, 5, 5)
        triangles = list()
        triangles.append(triang)
        triangles.append(triang2)

        vertices = list()
        vertices.append([0, 0])
        vertices.append([10, 0])
        vertices.append([4, 5])
        vertices.append([10, 2])
        vertices.append([2, 2])
        vertices.append([5, 5])

        polyg = pycrcc.Polygon(vertices, list(), triangles)

        obb5 = pycrcc.RectOBB(1, 1.3, 0.6, 10.8, 0.8)

        self.assertEqual(polyg.collide(obb5), 0)

        obb6 = pycrcc.RectOBB(1, 1.3, 0.6, 10.6, 0.6)
        self.assertEqual(polyg.collide(obb6), 1)

        obb7 = pycrcc.RectOBB(1, 1.3, 0.6, 10.8, 0.6)
        self.assertEqual(polyg.collide(obb7), 1)

        obb8 = pycrcc.RectOBB(1, 1.3, 0.6, 10.6, 1)

        self.assertEqual(polyg.collide(obb8), 1)

    def test_cc(self):
        aabb = pycrcc.RectAABB(2, 3, 3, 1.8)
        aabb2 = pycrcc.RectAABB(2, 3, 3, 1.7)
        circ = pycrcc.Circle(2.5, 6, 7)

        cc = pycrcc.CollisionChecker()
        cc.add_collision_object(aabb)
        obb1 = pycrcc.RectOBB(2, 1, 1.5, 6.0, 0)
        obb2 = pycrcc.RectOBB(2, 1, 1.5, 6.0, 20)

        self.assertEqual(cc.collide(circ), 1)

        tvo1 = pycrcc.TimeVariantCollisionObject(1)
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 2.0, 5))  # time step 1
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 2.5, 5))  # time step 2
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 3, 5))  # time step 3
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 3.5, 5))  # time step 4
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 4, 5))  # time step 5
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 4.5, 5))  # time step 6
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 5, 5))  # time step 7
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 5.5, 5))  # time step 8

        # Time variant obstacle that starts at time 4
        tvo2 = pycrcc.TimeVariantCollisionObject(4)
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 0))  # time step 4
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 2))  # time step 5
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 3))  # time step 6
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 4))  # time step 7
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 5))  # time step 8
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 6))  # time step 9
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 7))  # time step 10

        cc = pycrcc.CollisionChecker()
        cc.add_collision_object(tvo2)
        obb1 = pycrcc.RectOBB(2, 1, 1.5, 6.0, 0)
        obb2 = pycrcc.RectOBB(2, 1, 1.5, 6.0, 20)
        self.assertEqual(tvo2.collide(obb1), 1)
        self.assertEqual(cc.collide(obb1), 1)
        self.assertEqual(cc.collide(obb2), 0)

        cc2 = pycrcc.CollisionChecker()

        cc2.add_collision_object(obb2)

        self.assertEqual(cc2.collide(tvo2), 0)

        cc2.add_collision_object(obb1)

        self.assertEqual(cc2.collide(tvo2), 1)

        cc3 = pycrcc.CollisionChecker()

        cc3.add_collision_object(tvo1)

        self.assertEqual(cc3.collide(tvo2), 1)

        tvo3 = pycrcc.TimeVariantCollisionObject(12)
        tvo3.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 0))
        tvo3.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 2))

        cc4 = pycrcc.CollisionChecker()
        cc4.add_collision_object(tvo2)
        cc4.add_collision_object(pycrcc.RectOBB(2, 1, 1.5, 6.0, 0))
        self.assertEqual(cc4.collide(tvo3), 1)

    def test_group(self):
        tvo2 = pycrcc.TimeVariantCollisionObject(4)
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 0))  # time step 4
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 2))  # time step 5
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 3))  # time step 6
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 4))  # time step 7
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 5))  # time step 8
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 6))  # time step 9
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 7))  # time step 10

        cc = pycrcc.CollisionChecker()
        cc.add_collision_object(tvo2)
        obb1 = pycrcc.RectOBB(2, 1, 1.5, 6.0, 0)
        obb2 = pycrcc.RectOBB(2, 1, 1.5, 6.0, 20)
        obb3 = pycrcc.RectOBB(2.1, 1, 1.5, 6.0, 20)
        obb4 = pycrcc.RectOBB(2, 1, 1.5, 6.0, 0)

        sg = pycrcc.ShapeGroup()

        sg.add_shape(obb1)
        sg.add_shape(obb2)

        sg2 = pycrcc.ShapeGroup()
        sg2.add_shape(obb3)
        sg2.add_shape(obb4)

        self.assertEqual(sg.collide(sg2), 1)

        self.assertEqual(cc.collide(sg), 1)

    def test_tvo(self):
        tvo1 = pycrcc.TimeVariantCollisionObject(1)
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 2.0, 5))  # time step 1
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 2.5, 5))  # time step 2
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 3, 5))  # time step 3
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 3.5, 5))  # time step 4
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 4, 5))  # time step 5
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 4.5, 5))  # time step 6
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 5, 5))  # time step 7
        tvo1.append_obstacle(pycrcc.RectOBB(2, 1, 0.0, 5.5, 5))  # time step 8

        # Time variant obstacle that starts at time 4
        tvo2 = pycrcc.TimeVariantCollisionObject(4)
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 0))  # time step 4
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 2))  # time step 5
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 3))  # time step 6
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 4))  # time step 7
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 5))  # time step 8
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 6))  # time step 9
        tvo2.append_obstacle(pycrcc.RectOBB(2, 1, 1.5, 6.0, 7))  # time step 10

        self.assertEqual(tvo1.collide(tvo2), 1)

        # obb2=pycrcc.RectOBB(2, 1, 1.5, 6.0, 0)

        # TODO: poly-poly collision test


# def testGroupObj:


# def testGroupGroup:

# def testTV_Obj:
# def testTV_Group:

# def testTrajectory_Group:

# def testTrajectory_Obj_:


if __name__ == '__main__':
    unittest.main()
