(define (problem task)
(:domain hazarddetection)
(:objects
    door lamp - hazard
    elektron - robot
    initial lamp-location door-location - hazard-location
    home-system robot-sensor - sensor

)

(:init
    (obj_at elektron initial)

    (linked lamp lamp-location)
    (linked door door-location)

    (sensor_type door-kitchen home-system)
    (sensor_type lamp-kitchen robot-sensor)
)

(:goal (and
    (checked lamp)
    (checked door)
    (obj_at elektron initial)
))
)
