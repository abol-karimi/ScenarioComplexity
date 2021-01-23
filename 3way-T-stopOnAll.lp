#program boundedtime(maxtime).

%---------------Time domain------------
time(0..maxtime).

%------Geometry (T-intersection)-------
branchOf(Lane, Fork):-
  laneFromTo(Lane, Fork, _).

forkOnThroughRoad(F):-
  laneFromTo(L, F, _),
  laneCorrectSignal(L, off).

%-----------------Traffic--------------
arrivedAtTime(V, T):-
  arrivedAtForkAtTime(V, _, T).

exitedByTime(V, T):-
  exitedFromAtTime(V, _, T1),
  time(T),
  T1 <= T.

inTheIntersectionAtTime(V, T):-
  enteredByTime(V, T),
  not exitedByTime(V, T).

enteredByTime(V, T):-
  enteredForkAtTime(V, _, T1),
  time(T),
  T1 <= T.

atTheIntersectionAtTime(V, T):-
  arrivedByTime(V, T),
  not enteredByTime(V, T).

leftLaneByTime(V, L, T):-
  leftLaneAtTime(V, L, T1),
  time(T),
  T1 <= T.

enteredLaneByTime(V, L, T):-
  enteredLaneAtTime(V, L, T1),
  time(T),
  T1 <= T.

% Does not handle re-entries.
isOnLaneAtTime(V, L, T):-
  enteredLaneAtTime(V, L, _),
  not leftLaneByTime(V, L, T).

signaledAtFork(V, S, F):-
  signaledAtForkAtTime(V, S, F, _).

vehicleOnThroughRoadAtTime(V, T):-
  arrivedAtFork(V, F),
  forkOnThroughRoad(F),
  not exitedByTime(V, T).
%------------------------------------------
%--------------Rules pre-definitions-------
%------------------------------------------
requestedLane(Vehicle, Lane):-
  signaledAtFork(Vehicle, Signal, Fork),
  branchOf(Lane, Fork),
  laneCorrectSignal(Lane, Signal).

% vehicle on the planned lane
reservedLane(Vehicle, Lane1):-
  isOnLane(Vehicle, Lane1),
  requestedLane(Vehicle, Lane1).

reservedLane(Vehicle, Lane1):-
  isOnLane(Vehicle, Lane2),
  requestedLane(Vehicle, Lane2),
  overlaps(Lane2, Lane1),
  not leftTheLane(Vehicle, Lane1).

%---------------- Rules ---------------
% http://leginfo.legislature.ca.gov/faces/codes_displayText.xhtml?lawCode=VEH&division=11.&title=&part=&chapter=4.&article=
%--------------------------------------
% Page 35:
% At intersections without `STOP' or `YIELD' signs,
%  yield to traffic and pedestrians already in the intersection
%  or just entering the intersection.
mustYieldToForRule(Vehicle1, Vehicle2, yieldToInside):-
  atTheIntersection(Vehicle1),
  inTheIntersection(Vehicle2).

mustSlowToYield(Vehicle1):-
  mustYieldToForRule(Vehicle1, Vehicle2, yieldToInside),
  vehicleOnThroughRoad(Vehicle1),
  requestedLane(Vehicle1, Lane),
  reservedLane(Vehicle2, Lane).

mustStopToYield(Vehicle1):-
  mustYieldToForRule(Vehicle1, Vehicle2, yieldToInside),
  requestedLane(Vehicle1, Lane),
  reservedLane(Vehicle2, Lane),
  not vehicleOnThroughRoad(Vehicle1).


% 21800 (b)(1)
% The driver of any vehicle on a terminating highway shall yield the right-of-way
%  to any vehicle on the intersecting continuing highway.

mustYieldToForRuleAtTime(V1, V2, throughRoadFirst, T):-
  atTheIntersection(V1),
  vehicleOnThroughRoad(V2),
  not vehicleOnThroughRoad(V1).

violatesRightOfForRule(V1, V2, throughRoadFirst):-
  mustYieldToForRuleAtTime(V1, V2, throughRoadFirst, T),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  not leftLaneByTime(V2, L1, T).

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
  not enteredLaneByTime(V1, Lane2, T1),
  not leftLaneByTime(V2, Lane1, T).

violatesRightOfForRule(V1, V2, leftTurn):-
  mustYieldToForRuleAtTime(V1, V2, leftTurn, T),
  requestedLane(V2, L2),
  enteredLaneByTime(V1, L2, T).

%-------------------------------------------------

#show mustYieldToForRule/3.
#show mustStopToYield/1.
#show mustSlowToYield/1.
#show needNotStop/1.
#show needNotSlow/1.
