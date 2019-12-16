(define (problem task)
(:domain transportationattendant)
(:objects
    rico - robot
    luke john mark - human
    tea coffee sandwich plate glass - item
    initial - robot-location
    luke-location john-location mark-location wp0 wp1 wp2 wp3 wp4 wp5 - human-location
    glass-location sandwich-location spaghetti-location luke-greet-location kitchen - item-location
)

(:init
    (at rico initial)
    (at john john-location)
    (at luke luke-location)
    ;(at mark mark-location)

    (empty_robot)
    (not (not_empty_robot))

    (linked_to_location glass glass-location)
    (linked_to_location sandwich luke-greet-location)
    ;(linked_to_location tea kitchen)
    ;(linked_to_location coffee kitchen)
    ;(linked_to_location spoon kitchen)
    ;(linked_to_location plate kitchen)
)

(:goal (and
    (load_left sandwich john john-location)
    (load_left glass luke luke-location)
    ;(load_left plate mark kitchen)
    ;(load_left coffee mark mark-location)
    )
)

)

