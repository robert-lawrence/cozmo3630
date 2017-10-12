import asyncio

import cozmo
from fysom import *


def init_fsm():
    return Fysom({
        'initial': 'search_for_AR_cube',
        'events': [
            {'name': 'found_cube', 'src': 'search_for_AR_cube', 'dst': 'go_to_cube'},
            {'name': 'at_cube', 'src': 'go_to_cube', 'dst': 'reached_cube'},
            {'name': 'switch_to_color', 'src': 'reached_cube', 'dst': 'go_to_colored_cube'},
            {'name': 'found_colored_cube', 'src': 'go_to_colored_cube', 'dst': 'at_colored_cube'},
            {'name': 'cube_moved', 'src': 'at_colored_cube', 'dst': 'go_to_colored_cube'}
        ]
    })

def trigger(fsm, event_str):
    if not fsm.can(event_str):
        return # throw error?
    fsm.trigger(event_str) #this moves into new state defined by event def above
    print("Now in state"+fsm.current())
    #TODO - beep and update screen display

