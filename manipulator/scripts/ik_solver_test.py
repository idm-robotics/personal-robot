#!/usr/bin/env python2

import grasp_evaluator


def main():
    tester = grasp_evaluator.IKSolver()
    for i in range(5):
        tester.print_current_pose()
        x = float(input("x = "))
        y = float(input("y = "))
        z = float(input("z = "))
        print("Got values:", x, y, z)
        tester.move_xyz(x, y, z)


if __name__ == '__main__':
    main()
