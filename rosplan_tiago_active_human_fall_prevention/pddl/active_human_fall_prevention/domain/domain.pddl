(define (domain activehumanfallprevention)

(:requirements :typing :adl :durative-actions :numeric-fluents)

(:types robot human - locatable
        human-location robot-location waypoint - location)

(:predicates (at ?obj - locatable ?loc - location)
             (linked_to_location ?lcbl - locatable ?loc - location)
             (scanned_area ?loc_from - location ?loc_to - location)
             (human_coming)
             (not_human_coming)
             (human_detection_ongoing ?human - human)
             (human_informed ?human - human))

(:durative-action GO
    :parameters (?obj - robot ?start - location ?destination - location)
    :duration (= ?duration 60)
    :condition (and
        (at start (at ?obj ?start)))
    :effect (and
        (at end (at ?obj ?destination))
        (at end (not (at ?obj ?start))))
)

;return false when human is coming - to do that there should b
;during this action there is new knowledge added to KB - every time some new this is detected

(:durative-action GO_SCANNING
    :parameters (?obj - robot ?start - location ?destination - location)
    :duration (= ?duration 60)
    :condition (and
        (over all (not_human_coming))
        (at start (at ?obj ?start)))
    :effect (and
        ;return that this area has been checked
        (at end (at ?obj ?destination))
        (at end (scanned_area ?start ?destination))
        (at end (not (at ?obj ?start))))
)

;at the end of this action fact is added to KB that human is coming
;then replanning should be executed (at the end of smach server
;so it should be
;stop dispatching -> [generate problem -> run planner -> run parser -> dispatch]
;when human detected returns true (at end (is_human_coming))

(:durative-action HUMAN_APPROACH_DETECT
      :parameters (?obj - robot ?human - human)
      :duration ( = ?duration 1000)
      :condition (and
            (over all (not_human_coming)))
      :effect (and
            (at start (human_detection_ongoing ?human))
            (at end (human_coming))
            (at end (not (not_human_coming))))
)

;this should read from KB about hazardous items robot found and pass the info to the human after approach

(:durative-action HUMAN_INTERACT
      :parameters (?obj - robot ?humanloc - human-location ?human - human)
      :duration ( = ?duration 30)
      :condition (and
            (at start (human_coming))
            (over all (at ?obj ?humanloc))
            (over all (linked_to_location ?human ?humanloc)))
      :effect (and
            (at end (human_informed ?human)))
)

)