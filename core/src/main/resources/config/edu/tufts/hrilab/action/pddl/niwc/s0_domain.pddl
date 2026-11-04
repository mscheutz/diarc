(define (domain s0)

(:requirements :strips :typing :negative-preconditions)
; (:types thing)

(:predicates
    (route ?loc1 - object ?loc2 - object)
    (at ?what - object ?loc - object)
    (dead ?who - object)
    (have ?who - object ?what - object)
    (is_friendly_plane ?who - object)
    (is_hostile_plane ?who - object)
    (hostile_plane_in ?loc - object)
    (is_missile ?what - object)
    (is_out_of_hostile_range ?who - object)
    (we_are_alive)
)

(:action execute_mission_obj
    :parameters (?who - object ?from - object ?to - object ?target - object ?what - object ?where_target - object)
    :precondition (and
        (is_friendly_plane ?who)
        (route ?from ?to)
        (at ?who ?from)
        (have ?who ?what)
        (is_hostile_plane ?target)
        (is_out_of_hostile_range ?who)
        (we_are_alive)
    )
    :effect (and
        (not (at ?who ?from))
        (at ?who ?to)
        (not (hostile_plane_in ?where_target))
        (not (have ?who ?what))
        (dead ?target)
    )
)

(:action return_home
    :parameters (?who - object ?from - object ?to - object)
    :precondition (and
        (is_friendly_plane ?who)
        (route ?from ?to)
        (at ?who ?from)
        (we_are_alive)
        )
    :effect (and
        (not (at ?who ?from))
        (at ?who ?to)
    )
)

)