#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 19 14:00:26 2017

@author: christoph
"""

from library.transformations_lib import *


def a01_1():
    print('###### Teilaufgabe 1. ######')
    print('###### Übungsblatt Vorlesung 2.1 a) ######')
    tl_AB = trans((-2, 0, 0))
    rot_AB = rot2trans(rotz(np.pi))
    t_AB = np.dot(tl_AB, rot_AB)
    print('Transformationsmatrix T(A->B)')
    print(t_AB)

    print('Transformationsmatrix T(B->C)')
    tl_BC = trans((-4, -1, 0))
    rot_BC = rot2trans(rotz(-np.pi / 2))
    t_BC = np.dot(tl_BC, rot_BC)
    print(t_BC)

    print('Transformationsmatrix T(A->C)')
    tl_AC = trans((2, 1, 0))
    rot_AC = rot2trans(rotz(np.pi / 2))
    t_AC = np.dot(tl_AC, rot_AC)
    print(t_AC)

    print('###### Übungsblatt Vorlesung 2.1 b) ######')
    t_AC_ = np.dot(t_AB, t_BC)
    diff = t_AC - t_AC_

    print('T(A->C) = T(A->B)*T(B->C): ' + str(np.array_equal(t_AC, t_AC_)))
    print('Differenz:')
    print(diff)

    print('###### Übungsblatt Vorlesung 2.1 c) ######')
    tl_CA = trans((-1, 2, 0))
    rot_CA = rot2trans(rotz(-np.pi/2))
    t_CA = np.dot(tl_CA, rot_CA)
    t_CA_ = np.linalg.inv(t_AC)
    diff = t_CA - t_CA_
    print('T(C->A) = inv(T(A->C): ' + str(np.array_equal(t_CA, t_CA_)))
    print('Differenz')
    print(diff)

    print('###### Übungsblatt Vorlesung 2.1 d) ######')
    tl_CA = trans((-1, 2, 0))
    rot_CA = rot2trans(rotz(-np.pi / 2))
    t_CA = np.dot(tl_CA, rot_CA)

    p_A = np.array([[1], [-1], [0], [1]])
    p_C = np.dot(t_CA, p_A)
    print('Koordinatenwechsel für Punkt P von KS A nach KS C:')
    print(p_C)

    print('###### Übungsblatt Vorlesung 2.2 a) ######')
    tl_OA = trans((1, 1, 0))
    rot_OA = rot2trans(rotz(0))
    t_OA = np.dot(tl_OA, rot_OA)
    print('Transformationsmatrix T(O->A)')
    print(t_OA)

    tl_OB = trans((3, 2, 0))
    rot_OB = rot2trans(rotz(np.deg2rad(30)))
    t_OB = np.dot(tl_OB, rot_OB)
    print('Transformationsmatrix T(O->B)')
    print(t_OB)

    print('###### Übungsblatt Vorlesung 2.2 b) ######')
    p_B = np.array([[1], [1], [0], [1]])
    p_O = np.dot(t_OB, p_B)
    print('Koordinatenwechsel für Punkt P von KS B nach KS O:')
    print(p_O)

    print('###### Übungsblatt Vorlesung 2.2 c) ######')
    print('T(A->B) = inv(T(O->A)*T(O->B)')

    print('###### Übungsblatt Vorlesung 2.2 d) ######')
    t_AB = np.dot(np.linalg.inv(t_OA), t_OB)
    p_A = np.dot(t_AB, p_B)
    print('Punkt p_A:')
    print(p_A)

    print('###### Übungsblatt Vorlesung 2.2 e) ######')
    p = np.dot(t_AB, p_A)
    print(p)


def a01_2():
    print('###### Teilaufgabe 2. ######')
    print('###### a) ######')
    t_OR = np.dot(trans((2,1,0.1)), rot2trans(rotz(np.deg2rad(30))))
    t_RDB = trans((0.3-0.05, 0, 0.2))
    t_DBD = np.dot(np.dot(np.dot(trans((0,0,0.05)), rot2trans(rotz(np.deg2rad(40)))), trans((0,0,0))), rot2trans(rotx(np.pi/2)))
    t_DA1 = np.dot(np.dot(np.dot(trans((0,0,0.05)), rot2trans(rotz(np.deg2rad(30)))), trans((0.5, 0, 0))), rot2trans(rotx(0)))
    t_A1A2 = np.dot(np.dot(np.dot(trans((0, 0, 0)), rot2trans(rotz(-np.deg2rad(10)))), trans((0.5, 0, 0))), rot2trans(rotx(0)))

    p_O = np.dot(np.dot(np.dot(np.dot(np.dot(t_OR, t_RDB), t_DBD),t_DA1), t_A1A2), np.array([[0], [0], [0], [1]]))

    print('Punkt P im globalen KS O:')
    print(p_O)

    t_RDB = trans((0.3, 0, 0.2))
    t_DBD = np.dot(np.dot(np.dot(trans((0, 0, 0)), rot2trans(rotz(np.deg2rad(40)))), trans((0, 0, 0))),
                   rot2trans(rotx(np.pi / 2)))
    t_DA1 = np.dot(np.dot(np.dot(trans((0, 0, 0)), rot2trans(rotz(np.deg2rad(30)))), trans((0.5, 0, 0))),
                   rot2trans(rotx(0)))
    t_A1A2 = np.dot(np.dot(np.dot(trans((0, 0, 0)), rot2trans(rotz(-np.deg2rad(10)))), trans((0.5, 0, 0))),
                    rot2trans(rotx(0)))

    p_R = np.dot(np.dot(np.dot(np.dot(t_RDB, t_DBD), t_DA1), t_A1A2), np.array([[0], [0], [0], [1]]))
    print(p_R)

    angles = inverseKinematics(p_R, np.array([0.6, 0.5, 0.5]), 0.2)
    print(angles)


def main():
    a01_1()
    a01_2()


if __name__ == '__main__':
    main()