(define (domain activehumanfallprevention)

(:requirements :typing :adl :durative-actions :numeric-fluents)

(:types linkable
        robot human - locatable
        human-location robot-location - location)

(:predicates (at ?obj - locatable ?loc - location)
             (linked_to_location ?lcbl - locatable ?loc - location)
             (human_coming)
             (not_human_coming))

(:functions (is_robot_sensor ?sen - sensor))

(:durative-action GO
    :parameters (?obj - robot ?start - location ?destination - location)
    :duration (= ?duration 30)
    :condition (and
        (over all (not_human_coming))
        (at start (at ?obj ?start)))
    :effect (and
        (at end (at ?obj ?destination))
        (at start (not (at ?obj ?start))))
)

;return false when human is coming - to do that there should b
;during this action there is new knowledge added to KB - every time some new this is detected

(:durative-action GO_SCANNING
    :parameters (?obj - robot ?start - location ?destination - location)
    :duration (= ?duration 30)
    :condition (and
        (over all (not_human_coming))
        (at start (at ?obj ?start)))
    :effect (and
        ;return that this area has been checked
        (at end (at ?obj ?destination))
        (at start (not (at ?obj ?start))))
)

;at the end of this action fact is added to KB that human is coming
;then replanning should be executed (at the end of smach server
;so it should be
;stop dispatching -> [generate problem -> run planner -> run parser -> dispatch]
;when human detected returns true (at end (is_human_coming))

(:durative-action HUMAN_DETECTION
      :parameters (?obj - robot ?hazard - hazard ?hazloc - hazard-location ?sen - sensor)
      :duration ( = ?duration 1000)
      :condition (and
            (over all (not_human_coming))
            (over all (linked ?hazard ?hazloc)))
      :effect (and
            (at end (human_coming))
            (at end (not (not_human_coming))))
)

;this should read from KB about hazardous items robot found and pass the info to the human after approach

(:durative-action HUMAN_APPROACH
      :parameters (?obj - robot ?humanloc - human-location ?human - human)
      :duration ( = ?duration 30)
      :condition (and
            (at start (human_coming))
            (over all (at ?obj ?humanloc))
            (over all (linked ?human ?humanloc))
            )
      :effect (and
            (at end (human_informed ?human))
            )
)

)