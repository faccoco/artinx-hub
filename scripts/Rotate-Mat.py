import imp
import sympy as sym
import cmath as m

the_x, the_y, the_z = sym.symbols('x y z')

Rx = sym.Matrix(
    [[1, 0, 0, 0], 
     [0, sym.cos(the_x), -sym.sin(the_x), 0], 
     [0, sym.sin(the_x), sym.cos(the_x), 0],
     [0, 0, 0, 1]])

Ry = sym.Matrix(
    [[sym.cos(the_y), 0, sym.sin(the_y), 0], 
     [0, 1, 0, 0], 
     [-sym.sin(the_y), 0, sym.cos(the_y), 0],
     [0, 0, 0, 1]])

Rz = sym.Matrix(
    [[sym.cos(the_z), -sym.sin(the_z), 0, 0], 
     [sym.sin(the_z), sym.cos(the_z), 0, 0], 
     [0, 0, 1, 0],
     [0, 0, 0, 1]])

R = Rz * Rx * Ry
print(R)