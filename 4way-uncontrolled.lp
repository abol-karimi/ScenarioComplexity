#program boundedtime(maxtime).

%---------------Time domain------------
time(0..maxtime).

%-----------Geometry (4-way)-----------
onSameHighway(Fork1, Fork2):-
  isOnRightOf(Fork1, F),
  isOnRightOf(F, Fork2).

onSameHighway(Fork1, Fork2):-
  onSameHighway(Fork2, Fork1).

branchOf(Lane, Fork):-
  laneFromTo(Lane, Fork, _).

%-----------------Traffic--------------
arrivedAtFork(Vehicle, Fork):-
  arrivedAtForkAtTime(Vehicle, Fork, _).

arrivedAtTime(V, T):-
  arrivedAtForkAtTime(V, _, T).

enteredAtTime(V, T):
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

signaledLeft(Vehicle):-
  signaledAtFork(Vehicle, left, _).

arrivedFromDifferentHighways(Vehicle1, Vehicle2):-
  Vehicle1 != Vehicle2,
  arrivedAtFork(Vehicle1, F1),
  arrivedAtFork(Vehicle2, F2),
  not onSameHighway(F1, F2).

arrivedFromSameHighway(Vehicle1, Vehicle2):-
  arrivedAtFork(Vehicle1, F1),
  arrivedAtFork(Vehicle2, F2),
  onSameHighway(F1, F2).

%------------------------------------------
%--------------Rules pre-definitions-------
%------------------------------------------
requestedLane(Vehicle, Lane):-
  signaledAtFork(Vehicle, Signal, Fork),
  branchOf(Lane, Fork),
  laneCorrectSignal(Lane, Signal).

%---------------- Rules ---------------
% http://leginfo.legislature.ca.gov/faces/codes_displayText.xhtml?lawCode=VEH&division=11.&title=&part=&chapter=4.&article=
%--------------------------------------
% 21800 (a):
% The driver of a vehicle approaching an intersection shall yield the right-of-way
%  to any vehicle which has entered the intersection from a different highway.
mustYieldToForRuleAtTime(V1, V2, yieldToInside, T):-
  V1 != V2,
  arrivedAtForkAtTime(V1, F1, T1),
  enteredForkAtTime(V2, F2, T2),
  arrivedFromDifferentHighways(V1, V2),
  T1 > T2,
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  time(T),
  not leftLaneByTime(V2, L1, T).

violatesRightOfForRule(V1, V2, yieldToInside):-
  mustYieldToForRuleAtTime(V1, V2, yieldToInside, T),
  requestedLane(V2, L2),
  enteredLaneByTime(V1, L2, T).


% 21800 (b)(1)
% When two vehicles enter an intersection from different highways at the same time,
%  the driver of the vehicle on the left shall yield the right-of-way
%  to the vehicle on his or her immediate right.
mustYieldToForRuleAtTime(V1, V2, yieldToRight, T):-
  V1 != V2,
  arrivedAtForkAtTime(V1, F1, T1),
  arrivedAtForkAtTime(V2, F2, T1),
  isOnRightOf(F2, F1),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  time(T),
  T >= T1,
  not leftLaneByTime(V2, L1, T).

violatesRightOfForRule(V1, V2, yieldToRight):-
  mustYieldToForRuleAtTime(V1, V2, yieldToRight, T),
  requestedLane(V2, L2),
  enteredLaneByTime(V1, L2, T).

% 21801 (a)
% The driver of a vehicle intending to turn to the left or to complete a U-turn upon a highway,
%  shall yield the right-of-way to all vehicles approaching from the opposite direction
%  which are close enough to constitute a hazard at any time during the turning movement,
%  and shall continue to yield the right-of-way to the approaching vehicles
%  until the left turn or U-turn can be made with reasonable safety.
mustYieldToForRuleAtTime(V1, V2, leftTurn, T):-
  V1 != V2,
  signaledLeft(V1),
  arrivedFromSameHighway(V1, V2),
  requestedLane(V1, Lane1),
  requestedLane(V2, Lane2),
  overlaps(Lane1, Lane2),
  arrivedAtTime(V2, T1),
  time(T),
  T1 <= T,
  not enteredLaneByTime(V1, Lane2, T1),
  not leftLaneByTime(V2, Lane1, T).

violatesRightOfForRule(V1, V2, leftTurn):-
  mustYieldToForRuleAtTime(V1, V2, leftTurn, T),
  requestedLane(V2, L2),
  enteredLaneByTime(V1, L2, T).

% 22526 (a)
% Notwithstanding any official traffic control signal indication to proceed,
%  a driver of a vehicle shall not enter an intersection or marked crosswalk
%  unless there is sufficient space on the other side of the intersection or marked crosswalk
%  to accommodate the vehicle driven without obstructing the through passage of vehicles from either side.
mustYieldToForRuleAtTime(V1, V2, antiGridlock, T):-
  V1 != V2,
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  laneFromTo(L1, _, E),
  laneFromTo(L2, _, E),
  enteredAtTime(V1, T1),
  enteredAtTime(V2, T2),
  T1 > T2,
  time(T),
  T >= T1,
  not leftLaneByTime(V2, L2, T).

violatesRightOfForRule(V1, V2, antiGridlock):-
  mustYieldToForRuleAtTime(V1, V2, antiGridlock, T),
  enteredByTime(V1, T).

%-------------------------------------------------
violatesRightOf(V1, V2):-
  violatesRightOfForRule(V1, V2, _).

#show violatesRightOf/2.
#show violatesRightOfForRule/3.
#show mustYieldToForRuleAtTime/4.
