import numpy as np
import pandas as pd

class Mapa:
    """Clase mapa  donde se almacenan las caracteristicas basicas del mapa
    an/alt: ancho y alto del mapa
    pared: es una lista de tuplas con las posiciones donde se ubica pared
    mapa: descripcion del mapa 
    mercas: un diccionario donde key en la mercancia y value una lista de tuplas con posicion inicial y objectivo
    robot: tupla con la posicion del robot
    """
    def __init__(self, mapa = np.matrix([['M1', '#', 1, 'M3'], [1, '#', 1, 1], ['M2', 1, 'R', 1], [1, 1, 1, 1]])):
        self.an = 4
        self.alt = 4
        self.pared = tuple
        self.mapa = mapa
        self.mercas = dict
        self.robot = tuple
        assert (mapa.shape == (self.alt, self.an))

    def loc_pared(self):
        return np.where(self.mapa == '#')

    def loc_robot(self):
        aux = np.where(self.mapa == 'R')
        return (aux[0][0],aux[1][0])
    
    def es_obstac(self, pos, merc, cargado):
        """Funcion que comprueba dada una posicion si esta en un obstaculo o no
        pos: tupla, posicion a comprobar
        merc: str, la mercancia que estamos buscando/recogiendo
        cargado: Bool si True el robot carga mercancia si False no
        retorna Bool indicando si la posicion es o no obstaculo
        """
        #comprueba pared
        aux1 = any([True if tuple(i) == pos else False for i in self.pared])
        
        if not cargado:
            return (aux1 == True)
        #Si el robot va cargado, comprueva mercancias como obstaculos
        elif cargado:
            mercs = [key for key in self.mercas if merc not in key]
            aux2 = [m for m in mercs if pos == self.mercas[m][0]]
            return (aux1 == True) or (len(aux2)!=0)

    def es_merc(self, pos, merc):
        act_merc_pos = self.mercas[merc][0]
        return pos == act_merc_pos

    def cambiar_obj(self, val, merc):
        self.mercas[merc][0] = self.mercas[merc][1]

    def contiguo(self, pos):
        (x, y) = pos.pos
        return [(x, y + 1), (x, y - 1), (x + 1, y), (x - 1, y)]
    
    def dentromapa(self, pos):
        """Se comprueva si la posicion esta dentro de los limites del mapa
        pos: tupla, posicion a comprobar
        retorna Bool indicando si esta dentro o fuera de los limites
        """
        return (pos[0] >= 0) and (pos[0] < self.alt) and (pos[1] >= 0) and (pos[1] < self.an)

    def selecion_orden(self, hechas):
        """Funcion creada para seleccionar la mercancia a coger
        Es la suma de dos distancias de manhattan
        dist(Robot, merc) + dist(merc_inicio. merc_fin)
        input: Dataframe para guardar los datos
        retorna: dataaframe actualizado"""
        i = 0
        h = pd.DataFrame(index=np.arange(1), columns=['Mercancia', 'heuristica', 'Posicion inicial', 'Posicion fin'])
        iter = self.mercas.keys() - set(hechas)
        for merca in iter:
            a = Aestrella(self.robot, self.mercas[merca][0], merca)
            dest = a.Nodo(self.mercas[merca][1], None)
            d1 = a.manhatan(dest)
            a = Aestrella(self.robot, self.mercas[merca][0], merca)
            dest = a.Nodo(self.robot, None)
            d2 = a.manhatan(dest)
            c = d1 + d2
            h.loc[i, 'Mercancia'] = merca
            h.loc[i, 'heuristica'] = c
            h.loc[i, 'Posicion inicial'] = self.robot
            h.loc[i, 'Posicion fin'] = self.mercas[merca][1]
            i += 1
        h = h.sort_values(by=['heuristica'])
        return h


class Aestrella:
    """Classe de algoritmo del A* donde se genera el camino optino dado un inicio y fin
    ini: tupla posicion inicial
    fin: tupla posicion fin
    merc: str Mercancia que se quiere cargar/descargar
    l_ab: Lista de Nodos a explorar
    l_cer: Lista de Nodos cerrados
    cargado: Bool. Indica si el robot va cargado o no
    """
    class Nodo:
        """ Clase para almacenar la informacion de las posiciones visitadas
        pos: tupla con la posicion actual
        ances: tupla con la posicion anterior (padre)
        g: int coste de este nodo
        h: int calculo heuristico de este nodo
        f: int g+h
        """
        def __init__(self, pos=tuple, ances=tuple):
            self.pos = pos
            self.ances = ances
            self.g = 0
            self.h = 0
            self.f = 0

        def __str__(self):
            """Metodo implementado para retornar de manera legible los Nodos"""
            return "pos: {}, ances: {}, g:{}, h:{}, f{}\n".format(self.pos, self. ances, self.g,\
                                                       self.h, self.f)

    def __init__(self, ini: tuple = tuple, fin: tuple = tuple, merc: str = str, cargado: bool = bool):
        assert merc in ["M1", "M2", "M3"]
        self.ini = ini
        self.fin = fin
        self.merc = merc
        self.l_ab = []
        self.l_cer = []
        self.cargado = cargado

    def manhatan(self, nodox):
        return np.absolute(nodox.pos[0] - self.fin[0]) + np.absolute(nodox.pos[1] - self.fin[1])

    def coste(self, actual):
        return actual + 1

    def tratar_nodo(self, mapa, n_actual):
        for pos in mapa.contiguo(n_actual):
            if mapa.dentromapa(pos) and not mapa.es_obstac(pos, self.merc, self.cargado):
                n_nuevo = self.Nodo(pos, n_actual.pos)
                pos_lab = [i.pos for i in self.l_ab]
                pos_lcerr = [i.pos for i in self.l_cer]
                n_nuevo.h = self.manhatan(n_nuevo)
                n_nuevo.g = self.coste(n_actual.g)
                n_nuevo.f = n_nuevo.g + n_nuevo.h
                if n_nuevo.pos not in pos_lab and n_nuevo.pos not in pos_lcerr:
                    self.l_ab.append(n_nuevo)
                elif n_nuevo.pos in pos_lab:
                    if n_nuevo.g < n_actual.g:
                        n_nuevo.ances = n_actual.pos
        self.l_ab.sort(key=lambda x: x.f, reverse=True)

    def rehacer_camino(self):
        """Dada una lista cerrada y un futuro estado se devuleve la ruta optima
        y el coste final
        Se hace recoriendo la lista en orden inverso y escogiendo siempre por antecesor"""
        camino = []
        camino.append(self.l_cer[-1])
        #sumamos uno al coste para tener en cuenta el coste de cargar o descrgar mercancia
        coste_t = self.l_cer[-1].f +1
        for nodo in reversed(self.l_cer[:-1]):
            if camino[-1].ances == nodo.pos:
                camino.append(nodo)
        return [camino, coste_t]


    def aestrella(self, mapa):
        # iniciamos con la lista abierta y los posibles vecinos
        Bool = False
        n_ini = self.Nodo(self.ini, None)
        self.l_ab.append(n_ini)

        while not Bool:
            if not self.l_ab:
                raise("La lista abierta esta vacia, no se han encontrado caminos posibles")
            n_actual = self.l_ab.pop()
            Bool = mapa.es_merc(n_actual.pos, self.merc)
            self.l_cer.append(n_actual)
            self.tratar_nodo(mapa, n_actual)

        result = self.rehacer_camino()

        #Imprimimos camino como se solicia n la practica
        [print('Mover R fila: {} columna {}'.format(n.pos[0], n.pos[1])) for n in reversed(result[0])]
        n = result[0][0]
        if mapa.mercas[self.merc][0] != mapa.mercas[self.merc][1]:
            print('Cargar en R mercancia: {} fila: {} columna {}'.format(self.merc, n.pos[0], n.pos[1]))
        else:
            print('Descargar en R mercancia: {} fila: {} columna {}'.format(self.merc, n.pos[0], n.pos[1]))
        print('Este recorrido ha tenido un coste de: {}'.format(result[1]))

        mapa.cambiar_obj(n_actual.pos, self.merc)
        print('Lista cerrada:')
        print([str(n) for n in self.l_cer])
        print('Lista abierta:')
        print([str(n) for n in self.l_ab])

        return result


if __name__ == "__main__":
    #Flag creada pos is se quiere reiniciar la posicion del robot (2,2) despues de dejar la mercancia
    restart_robot = False
    #generamos el mapa y guardamos las posiciones del robot, pared y mercancias
    m = Mapa()
    m.pared = m.loc_pared()
    m.robot= m.loc_robot()
    m.mercas = {'M1': [(0, 0), (3, 3)], 'M2': [(2, 0), (3, 2)], 'M3': [(0, 3), (3, 1)]}

    #controlador de mercancias finalizadas
    m_hechas = []
    for i in range(len(m.mercas.keys())):
        h = m.selecion_orden(m_hechas)
        #Seleccionamos la primera mercancia con la mejor heuristica
        sel = h.iloc[0]
        
        print("*********************************************************")
        print("Comienzo A* con mercancia {}. Origen robot: {}, destino{}".format(\
            sel['Mercancia'], m.robot, m.mercas[sel['Mercancia']][1]))
        print("*********************************************************")
        
        #generacion de objeto y ejecucion del a*
        a = Aestrella(m.robot, m.mercas[sel['Mercancia']][0], sel['Mercancia'], cargado = False)
        result = a.aestrella(m)
        # Se captura la ultima posicion del robot y se reace el algoritmo per hacia el destino de la mercancia
        m.robot = result[0][0].pos
        a = Aestrella(m.robot, m.mercas[sel['Mercancia']][1], sel['Mercancia'], cargado = True)
        result = a.aestrella(m)
        
        #Comprobacion de si la siguiente mercancia se va a buscar desde la ultima posicion o desde la inicial del robot
        if restart_robot:
            m.robot = m.loc_robot()
        else:
            m.robot = result[0][0].pos
        #controlador de mercancias finalizadas
        m_hechas.append(sel['Mercancia'])
       
