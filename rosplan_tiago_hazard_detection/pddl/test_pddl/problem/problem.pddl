(define (problem task)
(:domain turtlebot_example)
(:objects
    kinba - robot
    wp0-0 wp0-1 wp0-2 wp0-3 wp0-4 wp0-5 wp0-6 wp1-6 wp1-7 wp1-8 - waypoint
    fl0 fl1 - floor
)
(:init


    (robot_at kinba wp0-0)


    (docked kinba)



    (outside_elevator kinba)

    (allowed_goto_waypoint kinba)

    (dock_at wp0-0)

    (waypoint_at_floor wp0-0 fl0)
    (waypoint_at_floor wp0-1 fl0)
    (waypoint_at_floor wp0-2 fl0)
    (waypoint_at_floor wp0-3 fl0)
    (waypoint_at_floor wp0-4 fl0)
    (waypoint_at_floor wp0-5 fl0)
    (waypoint_at_floor wp0-6 fl0)
    (waypoint_at_floor wp1-6 fl1)
    (waypoint_at_floor wp1-7 fl1)
    (waypoint_at_floor wp1-8 fl1)

    (elevator_access_waypoint wp0-5)
    (elevator_access_waypoint wp1-7)


    (current_elevator_waypoint wp0-6)

    (elevator_waypoint wp0-6)
    (elevator_waypoint wp1-6)

    (hallway_waypoint wp0-0)
    (hallway_waypoint wp0-1)
    (hallway_waypoint wp0-2)
    (hallway_waypoint wp0-3)
    (hallway_waypoint wp0-4)
    (hallway_waypoint wp0-5)
    (hallway_waypoint wp1-7)
    (hallway_waypoint wp1-8)

    (greeting_waypoint wp0-1)

    (destination_waypoint wp1-8)

)
(:goal (and
    (person_guided kinba)
    (person_greeted kinba)
    (docked kinba)
))
)
