(define (problem task)
(:domain hazarddetection)
(:objects
    door lamp - hazard
    elektron - robot
    initial lamp-location door-location - hazard-location
)
(:init
    (obj_at elektron initial)


    (linked lamp lamp-location)
    (linked door door-location)

)
(:goal (and
    (checked lamp)
    (checked door)
    (obj_at elektron initial)
))
)
