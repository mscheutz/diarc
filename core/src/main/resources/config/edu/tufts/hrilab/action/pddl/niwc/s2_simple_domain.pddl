(define (domain s2)

    (:requirements :typing :fluents :negative-preconditions)
    (:types
        concept physical - object
        actor physobj - physical
        cell - concept
        blue red - actor
        red_ss - red
    )

    (:predicates
        (blue_dead ?p - blue)
        (at ?v0 - actor ?v1 - cell)
        (no_fire_zone ?c - cell)
        (no_fly_zone ?c - cell)
        (p_attacked ?p - blue)
    )

    (:functions 
        (row ?c - cell)
        (col ?c - cell)
        (red_weapon_range)
        (red_target_HP)
    )

    (:action move-north
        :parameters (?actor - blue ?start - cell ?finish - cell)
        :precondition (and
            (at ?actor ?start)
            (= (col ?start) (col ?finish))
            (= (- (row ?finish) (row ?start)) 1)
            (not (no_fly_zone ?finish))
            ;(forall (?red_ss - red_ss)
            ;  (exists (?ss - cell)
            ;     (and
            ;        (at ?red_ss ?ss)
            ;        (> (+ (* (- (row ?ss) (row ?finish)) (- (row ?ss) (row ?finish))) (* (- (col ?ss) (col ?finish)) (- (col ?ss) (col ?finish)))) (* (red_weapon_range) (red_weapon_range)))
            ;     )
            ;  )
            ;)
        )
        :effect (and
            (at ?actor ?finish)
            (not (at ?actor ?start))
        )
    )

    (:action move-south
        :parameters (?actor - blue ?start - cell ?finish - cell)
        :precondition (and
            (at ?actor ?start)
            (= (col ?start) (col ?finish))
            (= (- (row ?start) (row ?finish)) 1)
            (not (no_fly_zone ?finish))
            ;(forall (?red_ss - red_ss)
            ;  (exists (?ss - cell)
            ;     (and
            ;        (at ?red_ss ?ss)
            ;        (> (+ (* (- (row ?ss) (row ?finish)) (- (row ?ss) (row ?finish))) (* (- (col ?ss) (col ?finish)) (- (col ?ss) (col ?finish)))) (* (red_weapon_range) (red_weapon_range)))
            ;     )
            ;  )
            ;)
        )
        :effect (and
            (at ?actor ?finish)
            (not (at ?actor ?start))
        )
    )

    (:action move-east
        :parameters (?actor - blue ?start - cell ?finish - cell)
        :precondition (and
            (at ?actor ?start)
            (= (row ?start) (row ?finish))
            (= (- (col ?finish) (col ?start)) 1)
            (not (no_fly_zone ?finish))
            ;(forall (?red_ss - red_ss)
            ;  (exists (?ss - cell)
            ;     (and
            ;        (at ?red_ss ?ss)
            ;        (> (+ (* (- (row ?ss) (row ?finish)) (- (row ?ss) (row ?finish))) (* (- (col ?ss) (col ?finish)) (- (col ?ss) (col ?finish)))) (* (red_weapon_range) (red_weapon_range)))
            ;     )
            ;  )
            ;)
        )
        :effect (and
            (at ?actor ?finish)
            (not (at ?actor ?start))
        )
    )

    (:action move-west
        :parameters (?actor - blue ?start - cell ?finish - cell)
        :precondition (and
            (at ?actor ?start)
            (= (row ?start) (row ?finish))
            (= (- (col ?start) (col ?finish)) 1)
            (not (no_fly_zone ?finish))
            ;(forall (?red_ss - red_ss)
            ;  (exists (?ss - cell)
            ;     (and
            ;        (at ?red_ss ?ss)
            ;        (> (+ (* (- (row ?ss) (row ?finish)) (- (row ?ss) (row ?finish))) (* (- (col ?ss) (col ?finish)) (- (col ?ss) (col ?finish)))) (* (red_weapon_range) (red_weapon_range)))
            ;     )
            ;  )
            ;)
        )
        :effect (and
            (at ?actor ?finish)
            (not (at ?actor ?start))
        )
    )

    (:action fire
        :parameters (?actor - blue ?at - cell ?target - red ?fireat - cell)
        :precondition (and
            (at ?actor ?at)
            (at ?target ?fireat)
            (not (no_fire_zone ?fireat))
            (not (p_attacked ?actor))
            (<= (+ (* (- (row ?at) (row ?fireat)) (- (row ?at) (row ?fireat))) (* (- (col ?at) (col ?fireat)) (- (col ?at) (col ?fireat)))) (* 1 1))
        )
        :effect (and
            (decrease (red_target_HP) 1)
            (p_attacked ?actor)
        )
    )
    
    ;(:event shot_down
    ;    :parameters (?actor - blue ?at - cell ?red_ss - red_ss ?ss - cell)
    ;    :precondition (and
    ;        (at ?actor ?at)
    ;        (at ?red_ss ?ss)
    ;        (<= (+ (* (- (row ?ss) (row ?at)) (- (row ?ss) (row ?at))) (* (- (col ?ss) (col ?at)) (- (col ?ss) (col ?at)))) (* (red_weapon_range) (red_weapon_range)))
    ;    )
;
    ;    :effect(and
    ;        (blue_dead ?actor)
    ;    )
    ;)
)