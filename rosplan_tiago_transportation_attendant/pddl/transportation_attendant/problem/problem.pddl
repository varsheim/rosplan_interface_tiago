(define (problem task)

(:domain transportationattendant)

(:objects
    rico - robot
    luke john alex - human
    glass sandwich spaghetti - item
    initial dock - robot-location
    luke-location john-location - human-location
    glass-location sandwich-location spaghetti - item-location
)

(:init
    (empty_robot)
    (not (not_empty_robot))
    (at rico initial)
    (at john john-location)
    (at luke luke-location)

    (linked_to_location glass glass-location)
    (linked_to_location sandwich sandwich-location)
    (linked_to_location spaghetti spaghetti-location)

)

(:goal (and
        ;(attended john john-location initial)
        ;(item_on_robot glass)
        (load_left glass john dock)
        (load_left sandwich luke initial)
        (attended luke luke-location dock)

    )
)

)

