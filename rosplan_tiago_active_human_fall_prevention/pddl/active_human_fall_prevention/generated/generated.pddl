(define (problem task)
(:domain activehumanfallprevention)
(:objects
    rico - robot
    luke john - human
    luke-location - human-location
    initial - robot-location
    wp0 wp1 wp2 wp3 wp4 wp5 wp6 - waypoint
)
(:init
    (at luke luke-location)
    (at rico wp2)

    (linked_to_location luke luke-location)

    (scanned_area wp0 wp1)
    (scanned_area wp1 wp2)

    (human_coming)

    (not (not_human_coming))

    (human_detection_ongoing luke)


)
(:goal (and
    (human_informed luke)
))
)
