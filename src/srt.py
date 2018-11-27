import treelib
import numpy as np

class Point:
    def __init__(self, x=None, y=None, array=None):
        if array is not None:
            self.x = array[0]
            self.y = array[1]
            # self.theta = array[2]
        else:
            self.x = x
            self.y = y
            # self.theta = theta
    def __str__(self):
        return (self.x, self.y).__str__()
    def to_array(self):
        return np.array([self.x, self.y])

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
        
        return (vetor[0]**2 + vetor[1]**2) < raio**2
    
    def get_radius_from_degree(self, grau):
        return self.raios[int(grau/30)%12]


class SRT:
    def __init__(self):
        self.cur_id = -1
        self.T = treelib.Tree()
        # self.T.create_node(self.cur_id, self.cur_id, data=StarRegion(ponto_inicial, raios))
    
    def add_node(self, ponto, raios, pai=None):
        node_exists = lambda p: abs(p.data.centro.x-ponto.x) < 0.2 and abs(p.data.centro.y-ponto.y) < 0.2
        nodes = self.T.filter_nodes(node_exists)
        if len(nodes) == 0:
            self.cur_id+=1
            node = self.T.create_node(self.cur_id, self.cur_id, pai, StarRegion(ponto, raios))
            return self.cur_id, node
        else:
            return nodes[0].identifier, nodes[0]

    def get_node(self, nid):
        return self.T.get_node(nid)

    def is_inside_tree(self, ponto, nid_to_exclude=None):
        esta_dentro = lambda p: p.data.is_point_inside(ponto)
        nodes = self.T.filter_nodes(esta_dentro)
        l=len(nodes)
        if l == 0:
            return False
        elif l == 1 and nid_to_exclude is not None and nodes[0].identifier == nid_to_exclude:
            return False
        else:
            return True
    
    def get_parent(self, nid):
        return self.T.parent(nid)

def explora():
    pass

def navigate():
    pass
