import treelib
import numpy as np
from util import Point

"""
`raios` deve ser um vetor com `sector_number` raios
"""
class StarRegion:
    def __init__(self, centro, raios, sector_number=12):
        self.centro = centro
        self.raios = raios

        self.SECTOR_NUMBER = sector_number
        self.ANGLES_PER_SECTOR = 360 / self.SECTOR_NUMBER

    def get_radius_from_degree(self, angulo):
        return self.raios[int(angulo/self.ANGLES_PER_SECTOR) % self.SECTOR_NUMBER]

    def is_point_inside(self, ponto):
        vetor = ponto.to_array()-self.centro.to_array()
        linha = Point(array=vetor)
        angulo = np.arctan2(linha.y, linha.x)

        angulo = ((angulo if angulo > 0 else (2*np.pi + angulo)) * 360 / (2*np.pi))%360

        raio = self.get_radius_from_degree(angulo)
        
        return (vetor**2).sum() < raio**2
    


class SRT:
    def __init__(self, sector_number=12):
        self.cur_id = -1
        self.T = treelib.Tree()
        self.SECTOR_NUMBER = sector_number
    
    def add_node(self, ponto, raios, pai=None):
        node_exists = lambda p: ((p.data.centro.to_array()-ponto.to_array())**2).sum() <= 0.0324 # 0.0324 = (0.18)^2; 0.18 ~ 0.35/2; 0.35 ~ 0.33 = largura do robô
        nodes = self.T.filter_nodes(node_exists)
        if len(nodes) == 0:
            self.cur_id += 1
            node = self.T.create_node(self.cur_id, self.cur_id, pai, StarRegion(ponto, raios, self.SECTOR_NUMBER))
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
