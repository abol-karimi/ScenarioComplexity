#program boundedtime(maxtime).

%---------------Time domain------------
time(0..maxtime).

%------Geometry (T-intersection)-------
branchOf(Lane, Fork):-
  laneFromTo(Lane, Fork, _).

forkOnContinuingHighway(F):-
  laneFromTo(L, F, _),
  laneCorrectSignal(L, off).

%----------Events (quantified)---------
arrivedAtTime(V, T):-
  arrivedAtForkAtTime(V, _, T).

arrivedAtFork(Vehicle, Fork):-
  arrivedAtForkAtTime(Vehicle, Fork, _).

enteredAtTime(V, T):-
  enteredForkAtTime(V, _, T).
  
enteredByTime(V, T):-
  enteredForkAtTime(V, _, T1),
  time(T),
  T1 <= T.

leftLaneByTime(V, L, T):-
  leftLaneAtTime(V, L, T1),
  time(T),
  T1 <= T.

enteredLaneByTime(V, L, T):-
  enteredLaneAtTime(V, L, T1),
  time(T),
  T1 <= T.

signaledAtFork(V, S, F):-
  signaledAtForkAtTime(V, S, F, _).

signaledLeft(V):-
  signaledAtFork(V, left, _).

%------------------------------------------
%--------Rules auxiliary-definitions-------
%------------------------------------------
arrivedFromOppositeDirection(V1, V2):- % TODO fix for multi-lane highway
  arrivedAtFork(V1, F1),
  forkOnContinuingHighway(F1),
  arrivedAtFork(V2, F2),
  forkOnContinuingHighway(F2),
  F1 != F2.

requestedLane(Vehicle, Lane):-
  signaledAtFork(Vehicle, Signal, Fork),
  branchOf(Lane, Fork),
  laneCorrectSignal(Lane, Signal).
 
%---------------- Rules ---------------
% http://leginfo.legislature.ca.gov/faces/codes_displayText.xhtml?lawCode=VEH&division=11.&title=&part=&chapter=4.&article=
%--------------------------------------
% 21801 (a):
% The driver of a vehicle intending to turn to the left or to complete a U-turn upon a highway,
%  shall yield the right-of-way to all vehicles approaching from the opposite direction
%  which are close enough to constitute a hazard at any time during the turning movement,
%  and shall continue to yield the right-of-way to the approaching vehicles
%  until the left turn or U-turn can be made with reasonable safety.
mustYieldToForRuleAtTime(V1, V2, leftTurnFromContinuingHighway, T):-
  V1 != V2,
  signaledLeft(V1),
  arrivedFromOppositeDirection(V1, V2),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  arrivedAtTime(V2, T1),
  time(T),
  T1 <= T,
  not enteredLaneByTime(V1, L2, T1),
  not leftLaneByTime(V2, L1, T).

violatesRightOfForRule(V1, V2, leftTurnFromContinuingHighway):-
  mustYieldToForRuleAtTime(V1, V2, leftTurnFromContinuingHighway, T),
  requestedLane(V2, L2),
  enteredLaneByTime(V1, L2, T).

% 21801 (b):
% A driver having yielded as prescribed in subdivision (a),
%  and having given a signal when and as required by this code,
%  may turn left or complete a U-turn,
%  and the drivers of vehicles approaching the intersection from the opposite direction
%  shall yield the right-of-way to the turning vehicle.
mustYieldToForRuleAtTime(V2, V1, yieldToLeftTurnFromContinuingHighway, T):-
  V1 != V2,
  signaledLeft(V1),
  arrivedFromOppositeDirection(V1, V2),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  arrivedAtTime(V2, T1),
  enteredLaneByTime(V1, L2, T1),
  time(T),
  T >= T1,
  not leftLaneByTime(V1, L2, T).

violatesRightOfForRule(V2, V1, yieldToLeftTurnFromContinuingHighway):-
  mustYieldToForRuleAtTime(V2, V1, yieldToLeftTurnFromContinuingHighway, T),
  requestedLane(V1, L1),
  enteredLaneByTime(V2, L1, T).

% 22450 (a):
% The driver of any vehicle approaching a stop sign at the entrance to, or within, an intersection
%  shall stop at a limit line, if marked,
%  otherwise before entering the crosswalk on the near side of the intersection.
% If there is no limit line or crosswalk,
%  the driver shall stop at the entrance to the intersecting roadway.
% 21802 (a):
% The driver of any vehicle approaching a stop sign at the entrance to, or within, an intersection
%  shall stop as required by Section 22450.
violatesRule(V, stopAtSign):-
  arrivedAtForkAtTime(V, F, T1),
  hasStopSign(F),
  enteredForkAtTime(V, F, T2),
  T2 - T1 < 4. % i.e. < 4*0.5 = 2 seconds
 
% The driver shall then yield the right-of-way
%  to any vehicles which have approached from another highway,
%  or which are approaching so closely as to constitute an immediate hazard,
%  and shall continue to yield the right-of-way to those vehicles until he or she can proceed with reasonable safety.
mustYieldToForRuleAtTime(V1, V2, fromTerminatingHighway, T):-
  V1 != V2,
  arrivedAtFork(V1, F),
  hasStopSign(F),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  arrivedAtTime(V2, T1),
  time(T),
  T1 <= T,
  not enteredByTime(V1, T1),
  not leftLaneByTime(V2, L1, T).

violatesRightOfForRule(V1, V2, fromTerminatingHighway):-
  mustYieldToForRuleAtTime(V1, V2, fromTerminatingHighway, T),
  requestedLane(V2, L2),
  enteredLaneByTime(V1, L2, T).
% 21802 (b):
% A driver having yielded as prescribed in subdivision (a) may proceed to enter the intersection,
%  and the drivers of all other approaching vehicles shall yield the right-of-way
%  to the vehicle entering or crossing the intersection.
% 21802 (c):
% This section does not apply where stop signs are erected upon all approaches to an intersection.


%-------------------------------------------------
violatesRightOf(V1, V2):-
  violatesRightOfForRule(V1, V2, _).

