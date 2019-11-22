(define (problem task)
(:domain transportationattendant)
(:objects
    rico - robot
    luke john - human
    glass sandwich - item
    luke-location john-location wp0 wp1 - human-location
    glass-location sandwich-location spaghetti-location luke-greet-location - item-location
    initial - robot-location
)
(:init
    (empty_robot)

    (not (not_empty_robot))



    (at rico initial)
    (at john wp1)
    (at luke luke-greet-location)


    (linked_to_location glass glass-location)
    (linked_to_location sandwich luke-greet-location)


)
(:goal (and
    (load_left sandwich john wp1)
))
)
