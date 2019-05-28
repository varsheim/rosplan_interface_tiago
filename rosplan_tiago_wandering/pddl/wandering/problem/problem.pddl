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

    (linked_to_location luke luke-location)
    (linked_to_location john john-location)
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

