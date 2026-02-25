from statemachine import StateMachine, State, Event
from collections import namedtuple

Situation = namedtuple('Situation', ['B', 'D', 'E', 'F', 'H'])

class SearchAndDeliverMachine(StateMachine):
    """
    A state machine for a robot that searches for objects and delivers them.
    """
    # Define States
    A = State('Initial State',                            value = Situation(True,  False, False, False, False), initial=True)                   # Situation: {'B': True,  'D': False, 'E': False, 'F': False, 'H': False}
    B = State('Patroling',                                value = Situation(False, False, False, False, False))                                 # Situation: {'B': False, 'D': False, 'E': False, 'F': False, 'H': False}
    C = State('Searching -> Object found',                value = Situation(False, True,  False, False, False))                                 # Situation: {'B': False, 'D': True,  'E': False, 'F': False, 'H': False}
    D = State('Arrived at object location',               value = Situation(False, True,  True,  False, False))                                 # Situation: {'B': False, 'D': True,  'E': True,  'F': False, 'H': False}
    E = State('Object Picked',                            value = Situation(False, False, False, True,  False))                                 # Situation: {'B': False, 'D': False, 'E': False, 'F': True,  'H': False}
    F = State('Initial State with object in hand',        value = Situation(True,  False, False, True , False))                                 # Situation: {'B': True,  'D': False, 'E': False, 'F': True,  'H': False}
    G = State('Initial State with known object location', value = Situation(True,  True,  False, False, False))                                 # Situation: {'B': True,  'D': True,  'E': False, 'F': False, 'H': False}
    H = State('Final State',                              value = Situation(False, False, False, False, True ), final=True)                     # Situation: {'B': False, 'D': False, 'E': False, 'F': False, 'H': True }

    # Define Events
    a  = Event(A.to(B)       | B.to.itself() | C.to.itself() | D.to(C)       | E.to.itself() | F.to(E)      | G.to(C)      , name='Patrol')
    b  = Event(A.to.itself() | B.to.itself() | C.to.itself() | D.to.itself() | E.to.itself() |F.to.itself() | G.to.itself(), name='Look for object')
    c  = Event(                                C.to(D)       | D.to.itself() |                                G.to(D)      , name='Go to object')
    e  = Event(A.to.itself() | B.to(A)       | C.to(G)       | D.to(G)       | E.to(F)       |F.to.itself() | G.to.itself(), name='Go to start')
    f  = Event(                                                D.to(E)                                                     , name='Pick up object')
    g  = Event(                                                                E.to(D)       |F.to(H)                      , name='Drop object')

    ab = Event(A.to(C)       | B.to(C)       | C.to.itself() | D.to(C)       | E.to.itself() | F.to(E)      | G.to(C)      , name='Patrol & Look for object')
    bc = Event(                                C.to(D)       | D.to.itself()                                | G.to(D)      , name='Look for object & Go to object')
    be = Event(A.to.itself() | B.to(A)       | C.to(G)       | D.to(G)       | E.to(F)       |F.to.itself() | G.to.itself(), name='Look for object & Go to start')
    bf = Event(                                                D.to(E)                                                     , name='Look for object & Pick up object')
    bg = Event(                                                                E.to(D)       |F.to(H)                      , name='Look for object & Drop object')

    def __init__(self):
        super().__init__()

    def send(self, event_name: str):
        event_name = ''.join(map(str, sorted(set(event_name))))
        return super().send(event_name)