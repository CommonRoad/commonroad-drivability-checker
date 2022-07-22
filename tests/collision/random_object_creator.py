import numpy as np
import commonroad_dc.pycrcc as pycrcc


class RandomObjectCreator:
    def __init__(self, grid_x_start, grid_x_end, grid_y_start, grid_y_end, x_steps, y_steps):
        assert (x_steps == int(x_steps) and x_steps > 0)
        assert (y_steps == int(y_steps) and y_steps > 0)
        self.object_creator = dict()
        self.object_creator[0] = self.create_random_sphere
        self.object_creator[1] = self.create_random_aabb
        self.object_creator[2] = self.create_random_obb
        self.object_creator[3] = self.create_random_sphere
        self.object_creator[4] = self.create_random_triangle
        self.object_creator[5] = self.create_random_polygon
        self.grid_x_start = grid_x_start
        self.grid_x_end = grid_x_end
        self.grid_y_start = grid_y_start
        self.grid_y_end = grid_y_end
        self.angles = np.linspace(0, 360, 10)

        if (self.grid_x_end < self.grid_x_start):
            self.grid_x_end, self.grid_x_start = self.grid_x_start, self.grid_x_end

        if (self.grid_y_end < self.grid_y_start):
            self.grid_y_end, self.grid_y_start = self.grid_y_start, self.grid_y_end

        self.grid_x_step_size = (self.grid_x_end - self.grid_x_start) / x_steps
        self.grid_y_step_size = (self.grid_y_end - self.grid_y_start) / y_steps
        self.x_grid_cells = np.linspace(self.grid_x_start, self.grid_x_end, x_steps)
        self.y_grid_cells = np.linspace(self.grid_y_start, self.grid_y_end, y_steps)

    def generate_random_vector(self):
        x = np.random.choice(self.x_grid_cells)
        y = np.random.choice(self.y_grid_cells)

        return [x, y]

    def generate_random_xvalue(self):
        return np.random.choice(self.x_grid_cells)

    def generate_random_yvalue(self):
        return np.random.choice(self.y_grid_cells)

    def generate_random_angle(self):
        return np.random.choice(self.angles)

    def orientation_from_angle(self, angle):
        return ((angle % 360) - 180) / 180 * np.pi

    def create_random_sphere(self):
        center = self.generate_random_vector()
        rad = abs(self.generate_random_xvalue())
        return pycrcc.Circle(rad, center[0], center[1])

    def create_random_aabb(self):
        center = self.generate_random_vector()
        rad_x = abs(self.generate_random_xvalue())
        rad_y = abs(self.generate_random_yvalue())
        return pycrcc.RectAABB(rad_x, rad_y, center[0], center[1])

    def create_random_obb(self):
        center = self.generate_random_vector()
        rad_x = abs(self.generate_random_xvalue())
        rad_y = abs(self.generate_random_yvalue())
        angle = self.generate_random_angle()
        orient = self.orientation_from_angle(angle)

        return pycrcc.RectOBB(rad_x, rad_y, orient, center[0], center[1])

    def create_random_triangle(self):
        p1 = self.generate_random_vector()
        p2 = self.generate_random_vector()
        p3 = self.generate_random_vector()
        return pycrcc.Triangle(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1])

    def create_random_polygon(self, tri_count=-1):
        if tri_count == -1:
            tri_count = np.random.choice(range(1, 50))
        assert ((tri_count > 0) and (tri_count == int(tri_count)))
        triangle_list = list()
        vertex_list = list()
        for i in range(tri_count):

            triang = self.create_random_triangle()
            triangle_list.append(triang)
            verts = triang.vertices()
            for p in verts:
                vertex_list.append(p)

        return pycrcc.Polygon(vertex_list, list(), triangle_list)

    def create_random_shape(self):
        obj_type = np.random.choice(range(6))
        return self.create_random_shape_helper(obj_type)

    def create_random_shape_group(self, shape_count=-1):
        if shape_count == -1:
            shape_count = np.random.choice(range(1, 50))
        assert ((shape_count > 0) and (shape_count == int(shape_count)))
        shapes = list()
        for i in range(shape_count):
            new_shape = self.create_random_shape()
            shapes.append(new_shape)
        sg = pycrcc.ShapeGroup()
        for shape in shapes:
            sg.add_shape(shape)

        return sg

    def create_random_tvobst(self, start_step=-1, num_steps=-1):
        if start_step == -1:
            start_step = np.random.choice(range(1, 50))
        assert ((start_step > 0) and (start_step == int(start_step)))

        if num_steps == -1:
            num_steps = np.random.choice(range(1, 50))
        assert ((num_steps > 0) and (num_steps == int(num_steps)))

        tvobj = pycrcc.TimeVariantCollisionObject(start_step)
        for i in range(num_steps):
            new_obst = self.create_random_static_object()
            tvobj.append_obstacle(new_obst)

        return tvobj

    def create_random_static_object(self):
        obj_type = np.random.choice(range(7))
        if obj_type == 6:
            return self.create_random_shape_group()

        return self.create_random_shape()

    def create_random_object(self):
        obj_type = np.random.choice(range(8))
        if obj_type == 7:
            return self.create_random_tvobst()
        return self.create_random_static_object()

    def create_random_shape_helper(self, obj_type):

        assert (obj_type in self.object_creator.keys())

        return self.object_creator[obj_type]()
