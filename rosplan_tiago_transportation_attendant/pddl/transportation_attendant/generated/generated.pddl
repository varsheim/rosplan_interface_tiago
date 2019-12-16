(define (problem task)
(:domain transportationattendant)
(:objects
    rico - robot
    luke john mark - human
    tea coffee sandwich plate - item
    luke-location john-location mark-location wp0 wp1 wp2 wp3 wp4 wp5 - human-location
    glass-location sandwich-location spaghetti-location luke-greet-location kitchen - item-location
    initial - robot-location
)
(:init
    (empty_robot)

    (not (not_empty_robot))



    (at rico initial)
    (at john john-location)
    (at luke luke-location)


    (linked_to_location glass glass-location)
    (linked_to_location sandwich luke-greet-location)


)
(:goal (and
    (load_left sandwich john john-location)
))
)
