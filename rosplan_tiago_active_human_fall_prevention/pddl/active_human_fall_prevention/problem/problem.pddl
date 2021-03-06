(define (problem task)
(:domain activehumanfallprevention)
(:objects
    luke john - human
    rico - robot
    initial - robot-location
    luke-location - human-location
    wp0 wp1 wp2 wp3 wp4 wp5 wp6 - waypoint
)

(:init
    (at rico initial)
    (at luke luke-location)
    (not_human_coming)
    (not (human_coming))

    (linked_to_location luke luke-location)
)

(:goal (and
    (scanned_area wp0 wp1)
    (scanned_area wp1 wp2)
    (scanned_area wp2 wp3)
    (scanned_area wp3 wp4)
    (scanned_area wp4 wp5)
    (scanned_area wp5 wp6)
    (human_detection_ongoing luke)
    )
)

)
