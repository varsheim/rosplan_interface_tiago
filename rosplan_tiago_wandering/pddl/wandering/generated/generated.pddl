(define (problem task)
(:domain wandering)
(:objects
    nod look-around - robot-greet
    rico - robot
    luke john - human
    glass sandwich - item
    luke-greet-location john-greet-location - greet-location
    luke-location john-location - human-location
    glass-location sandwich-location - item-location
    initial - robot-location
)
(:init
    (empty_robot)





    (at rico initial)

    (linked_to_location luke luke-location)
    (linked_to_location john john-location)
    (linked_to_location glass glass-location)
    (linked_to_location sandwich sandwich-location)

    (linked_to_greet_location luke luke-greet-location)
    (linked_to_greet_location john john-greet-location)

)
(:goal (and
    (greeted nod luke)
    (greeted look-around john)
    (gave glass john)
    (gave sandwich luke)
))
)
