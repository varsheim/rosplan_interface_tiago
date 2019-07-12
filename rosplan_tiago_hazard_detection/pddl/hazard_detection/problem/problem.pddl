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

    (= (is_automated robot
)

(:goal (and
    (checked_light light-kitchen)
    (checked_light light-workshop)
    (checked_door door-kitchen)
    (checked_door door-main)
    (checked_dishwasher dishwasher-kitchen)
))
)
