#!/usr/bin/env python
# coding=latin1

from numpy import array

"""
Recebe um valor e um limite inferior e superior e retorna o próprio valor
ou um dos limites, caso o valor não esteja entre eles.
O limite é inclusivo no mínimo e exclusivo no máximo.
"""
def bound(v, min, max):
    if v < min:
        return min
    elif v > max-1:
        return max-1
    else:
        return v


class Ponto:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def to_array(self):
        return array([self.x, self.y])
