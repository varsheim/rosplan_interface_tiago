(define (problem task)

(:domain wandering)

(:objects
    rico - robot
    luke john - human
    glass sandwich - item
    glass-location sandwich-location - item-location
    initial - robot-location
    luke-location john-location - human-location
    nod look-around - robot-greet
)

(:init
    (empty_robot)
    (obj_at rico initial)
    (obj_at glass glass-location)
    (obj_at sandwich sandwich-location)
    (obj_at luke luke-location)
    (obj_at john john-location)

    (linked_to_locatable nod luke)
    (linked_to_locatable look-around john)
)

(:goal (and
    (greeted luke)
    (greeted john)
    (gave glass luke)
    (gave sandwich john)
    (obj_at rico initial)
    )
)

)

