(define (problem guideZdzisiu)
(:domain guide)
(:objects
    zdzisiu - human
    elektron - robot
    init-rob init-hum kuchnia - location
)
(:init
    (at elektron init-rob)
    (at zdzisiu init-hum)
)
(:goal (and
    (at zdzisiu kuchnia)
    (not (interacts elektron zdzisiu))
)
))
