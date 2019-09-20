(define (domain hazarddetection)

(:requirements :typing :adl :durative-actions :numeric-fluents)

(:types hazard - linkable
        robot - locatable
        hazard-location robot-location - location
        robot-sensor home-system-sensor - sensor)

(:predicates (at ?obj - locatable ?loc - location)
             (checked_door ?hazard - hazard)
             (checked_light ?hazard - hazard)
             (checked_dishwasher ?hazard - hazard)
             (linked ?hazard - linkable ?loc - location)
             (sensor_type ?haz - hazard ?sen - sensor)
             (checking)
             (not_checking))

(:functions (is_robot_sensor ?sen - sensor))

(:durative-action GO
      :parameters (?obj - robot ?start - location ?destination - location)
      :duration (= ?duration 30)
      :condition (and
            (at start (at ?obj ?start))
            (over all (not_checking)))
      :effect (and
            (at end (at ?obj ?destination))
            (at start (not (at ?obj ?start))))
)

(:durative-action CHECK_DOOR
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location ?sen - sensor)
      :duration ( = ?duration (+ (* (is_robot_sensor ?sen) 30) 5))
      :condition
            (and
                (at start (not_checking))
                (over all (sensor_type ?hazard ?sen))
                (over all (or
                            (at ?obj ?hazloc)
                            (= (is_robot_sensor ?sen) 0)))
                (over all (linked ?hazard ?hazloc))
            )
      :effect (and
            (at start (not (not_checking)))
            (at end (not_checking))
            (at end (checked_door ?hazard)))
)

(:durative-action CHECK_LIGHT
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location ?sen - sensor)
      :duration ( = ?duration (+ (* (is_robot_sensor ?sen) 30) 5))
      :condition (and
            (at start (not_checking))
            (over all (sensor_type ?hazard ?sen))
            (over all (or
                        (at ?obj ?hazloc)
                        (= (is_robot_sensor ?sen) 0)))
            (over all (linked ?hazard ?hazloc)))
      :effect (and
            (at start (not (not_checking)))
            (at end (not_checking))
            (at end (checked_light ?hazard)))
)

(:durative-action CHECK_DISHWASHER
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location ?sen - sensor)
      :duration ( = ?duration (+ (* (is_robot_sensor ?sen) 30) 5))
      :condition (and
            (at start (not_checking))
            (over all (sensor_type ?hazard ?sen))
            (over all (or
                        (at ?obj ?hazloc)
                        (= (is_robot_sensor ?sen) 0)))
            (over all (linked ?hazard ?hazloc)))
      :effect (and
            (at start (not (not_checking)))
            (at end (not_checking))
            (at end (checked_dishwasher ?hazard)))
)

)