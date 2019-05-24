(define (problem task)
(:domain wandering)
(:objects
    rico - robot
    luke john - human
    glass sandwich - item
    glass-location sandwich-location - item-location
    door lamp - hazard
    initial lamp-location door-location - hazard-location
)

(:init
    (empty_robot)
    (obj_at rico initial)
    (obj_at glass glass-location)
    (obj_at sandwich sandwich-location)
    (obj_at luke luke-location)
    (obj_at john john-location)

    (linked lamp lamp-location)
    (linked door door-location)

)

(:goal (and
    (greeted luke)
    (greeted john)
    (gave glass luke)
    (gave sandwich john)
    (obj_at rico initial)
))

)

