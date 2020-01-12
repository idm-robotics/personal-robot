#!/usr/bin/env python2

from grasp_evaluator import IKSolver

tester = IKSolver()


def main():
    for i in range(5):
        tester.print_current_pose()
        x = float(input("x = "))
        y = float(input("y = "))
        z = float(input("z = "))
        direction_x = float(input("x direction = "))
        direction_y = float(input("y direction = "))
        direction_z = float(input("z direction = "))
        upper_x = float(input("x upper = "))
        upper_y = float(input("y upper = "))
        upper_z = float(input("z upper = "))
        print("Got values:", x, y, z)
        # tester.move_xyz(x, y, z)
        tester.move_pose(
            IKSolver.point_to_pose(x, y, z, direction_x, direction_y, direction_z, upper_x, upper_y, upper_z))


if __name__ == '__main__':
    main()
