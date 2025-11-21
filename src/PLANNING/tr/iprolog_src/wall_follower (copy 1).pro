wall_follower(scan(FRONT, FRONT_LEFT, LEFT_FRONT, _, _, _, _, _, _, _, _, FRONT_RIGHT), LV, AV) :-
	near_start -> LV = 0.0, AV = 0.0;
	LEFT_FRONT > 0.9 -> LV = 0.2, AV = 1.5;
	FRONT < 0.7 -> LV = 0.0, AV = -1.5;
	FRONT_LEFT < 0.6 -> LV = 0.3, AV = -1.5;
	FRONT_RIGHT < 0.6 -> LV = 0.3, AV = 1.5;
	LEFT_FRONT > 0.6 -> LV = 0.3, AV = 1.5;
	LV = 0.3, AV = 0.0.
