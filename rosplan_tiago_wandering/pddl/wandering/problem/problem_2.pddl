(define (problem task)

(:domain wandering)

(:objects
    rico - robot
    luke john - human
    glass sandwich - item
    initial - robot-location
    luke-location john-location - human-location
    glass-location sandwich-location - item-location
    nod look-around - robot-greet
)

(:init
    (empty_robot)
    (obj_at rico initial)

    (linked_to_location luke luke-location)
    (linked_to_location john john-location)
    (linked_to_location glass glass-location)
    (linked_to_location sandwich sandwich-location)
)

(:goal (and
    (greeted nod luke)
    (greeted look-around john)
    (gave glass john)
    (gave sandwich luke)
    (obj_at rico initial)
    )
)

)

