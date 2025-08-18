from statemachine import StateMachine, State, Event

class SearchAndDeliverMachine(StateMachine):
    """
    A state machine for a robot that searches for objects and delivers them.
    """
    # Define States
    A = State('Initial State', initial=True)
    B = State('Patroling')
    C = State('Searching -> Object found')
    D = State('Arrived at object location')
    E = State('Object Picked')
    F = State('Initial State with object in hand')
    G = State('Initial State with known object location')
    H = State('Final State', final=True)

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
        event_name = ''.join(map(str, sorted(event_name)))
        return super().send(event_name)