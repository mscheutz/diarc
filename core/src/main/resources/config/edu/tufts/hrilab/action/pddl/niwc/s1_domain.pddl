(define (domain s1)

    (:requirements :typing :fluents :negative-preconditions)
    (:types cell)

    (:predicates
        (blue_at ?c - cell)
        (red_dead)
        (blue_dead)
        (red_ammo_at ?c - cell)
        (red_ss_at ?c - cell)
    )
    (:functions 
        (row ?c - cell)
        (col ?c - cell)
        (red_weapon_range)
    )

    (:action move-north
        :parameters (?start - cell ?finish - cell)
        :precondition (and
            (blue_at ?start)
            ; move
            (and
                (= (col ?start) (col ?finish))
                (= (- (row ?finish) (row ?start)) 1)
            )
        )
        :effect (and
            (blue_at ?finish)
            (not (blue_at ?start))
        )
    )

        (:action move-south
        :parameters (?start - cell ?finish - cell)
        :precondition (and
            (blue_at ?start)
            ; move
            (and
                (= (col ?start) (col ?finish))
                (= (- (row ?start) (row ?finish)) 1)
            )
        )
        :effect (and
            (blue_at ?finish)
            (not (blue_at ?start))
        )
    )

        (:action move-east
        :parameters (?start - cell ?finish - cell)
        :precondition (and
            (blue_at ?start)
            ; move
            (and
                (= (row ?start) (row ?finish))
                (= (- (col ?finish) (col ?start)) 1)
            )
        )
        :effect (and
            (blue_at ?finish)
            (not (blue_at ?start))
        )
    )

        (:action move-west
        :parameters (?start - cell ?finish - cell)
        :precondition (and
            (blue_at ?start)
            ; move
            (and
                (= (row ?start) (row ?finish))
                (= (- (col ?start) (col ?finish)) 1)
            )
        )
        :effect (and
            (blue_at ?finish)
            (not (blue_at ?start))
        )
    )

    (:action fire_weapon
        :parameters (?at - cell ?fireat - cell)
        :precondition (and
            (blue_at ?at)
            (red_ammo_at ?fireat)
            (<= (+ (* (- (row ?at) (row ?fireat)) (- (row ?at) (row ?fireat))) (* (- (col ?at) (col ?fireat)) (- (col ?at) (col ?fireat)))) (* 3 3))
        )
        :effect (and
            (red_dead)
        )
    )

    ;(:event blue_shot_down
    ;    :parameters (?ss - cell ?at - cell)
    ;    :precondition (and
    ;        (blue_at ?at)
    ;        (red_ss_at ?ss)
    ;        (<= (+ (^ (^ (- (row ?ss) (row ?at)) 2) 0.5) (^ (^ (- (col ?ss) (col ?at)) 2) 0.5)) (red_weapon_range))
    ;    )
;
    ;    :effect(and
    ;        (blue_dead)
    ;    )
    ;)

)
