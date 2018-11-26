import treelib
import numpy as np

class Point:
    def __init__(self, x=None, y=None, array=None):
        if array != None:
            self.x = array[0]
            self.y = array[1]
        else:
            self.x = x
            self.y = y
    def __str__(self):
        return (self.x, self.y)
    def to_array(self):
        return np.array([self.x,self.y])

"""
`raios` deve ser um vetor com 12 raios
"""
class StarRegion:
    def __init__(self, centro, raios):
        self.centro = centro
        self.raios = raios
    def is_point_inside(self, ponto):
        vetor = ponto.to_array()-self.centro.to_array()
        linha = Point(array=vetor)
        angulo = np.arctan2(linha.y, linha.x)

        angulo = ((angulo if angulo > 0 else (2*np.pi + angulo)) * 360 / (2*np.pi))%360

        raio = self.raios[int(angulo/30)%12]
        
        return (vetor**2).sum() < raio**2


class SRT:
    def __init__(self, ponto_inicial, raios):
        self.curTag = 0
        self.T = treelib.Tree()
        self.T.create_node(self.curTag, self.curTag, data=StarRegion(ponto_inicial, raios))
    
    def add_node(self, ponto, raios, pai):
        self.curTag+=1
        return self.T.create_node(self.curTag, self.curTag, pai, StarRegion(ponto, raios))

    def get_node(self, nid):
        return self.T.get_node(nid)

    def is_inside_tree(self, ponto):
        esta_dentro = lambda p: p.data.is_point_inside(ponto)
        nodes = self.T.filter_nodes(esta_dentro)
        return len(nodes) != 0
    

def explora():
    pass

def navigate():
    pass
