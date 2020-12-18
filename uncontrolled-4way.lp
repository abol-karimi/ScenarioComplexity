%-----------Geometry (4-way)-----------
onSameHighway(Fork1, Fork2):-
  isOnRightOf(Fork1, F),
  isOnRightOf(F, Fork2).

onSameHighway(Fork1, Fork2):-
  onSameHighway(Fork2, Fork1).

%---------------Time domain------------
eventTime(T):-
  arrivedAtForkAtTime(_, _, T).
eventTime(T):-
  enteredForkAtTime(_, _, T).
eventTime(T):-
  enteredLaneAtTime(_, _, T).
eventTime(T):-
  leftLaneAtTime(_, _, T).
eventTime(T):-
  exitedFromAtTime(_, _, T).

%-----------------Traffic--------------
arrivedAtFork(Vehicle, Fork):-
  arrivedAtForkAtTime(Vehicle, Fork, _).

arrivedAtTime(Vehicle, T):-
  arrivedAtForkAtTime(Vehicle, _, T).

arrivedByTime(Vehicle, T):-
  arrivedAtForkAtTime(Vehicle, _, T1),
  eventTime(T),
  T1 <= T.

exitedByTime(Vehicle, T):-
  exitedFromAtTime(Vehicle, _, T1),
  eventTime(T),
  T1 <= T.

inTheIntersectionAtTime(Vehicle, T):-
  enteredByTime(Vehicle, T),
  not exitedByTime(Vehicle, T).

enteredByTime(Vehicle, T):-
  enteredForkAtTime(Vehicle, _, T1),
  eventTime(T),
  T1 <= T.

atTheIntersectionAtTime(Vehicle, T):-
  arrivedByTime(Vehicle, T),
  not enteredByTime(Vehicle, T).

isAtForkAtTime(Vehicle, Fork, T):-
  arrivedAtFork(Vehicle, Fork),
  atTheIntersectionAtTime(Vehicle, T).

arrivedEarlierThan(Vehicle1, Vehicle2):-
  arrivedAtForkAtTime(Vehicle1, _, T1),
  arrivedAtForkAtTime(Vehicle2, _, T2),
  T1 < T2.

arrivedSameTime(Vehicle1, Vehicle2):-
  arrivedAtForkAtTime(Vehicle1, _, T),
  arrivedAtForkAtTime(Vehicle2, _, T).

enteredSameOrEarlierThan(V1, V2):-
  enteredForkAtTime(V1, _, T1),
  enteredForkAtTime(V2, _, T2),
  T1 <= T2.

isOnRightOfAtTime(Vehicle1, Vehicle2, T):-
  isAtForkAtTime(Vehicle1, Fork1, T),
  isAtForkAtTime(Vehicle2, Fork2, T),
  isOnRightOf(Fork1, Fork2).

leftLaneByTime(Vehicle, Lane, T):-
  leftLaneAtTime(Vehicle, Lane, T1),
  eventTime(T),
  T1 <= T.

enteredLaneByTime(Vehicle, Lane, T):-
  enteredLaneAtTime(Vehicle, Lane, T1),
  eventTime(T),
  T1 <= T.

% Does not handle re-entries.
isOnLaneAtTime(Vehicle, Lane, T):-
  enteredLaneByTime(Vehicle, Lane, T),
  not leftLaneByTime(Vehicle, Lane, T).

branchOf(Lane, Fork):-
  laneFromTo(Lane, Fork, _).

signaledAtFork(Vehicle, Signal, Fork):-
  signaledAtForkAtTime(Vehicle, Signal, Fork, _).

signaledLeft(Vehicle):-
  signaledAtFork(Vehicle, left, _).

goingStraight(Vehicle):-
  signaledAtFork(Vehicle, off, _).

arrivedFromDifferentHighways(Vehicle1, Vehicle2):-
  arrivedAtFork(Vehicle1, F1),
  arrivedAtFork(Vehicle2, F2),
  not onSameHighway(F1, F2).

%------------------------------------------
%--------------Rules pre-definitions-------
%------------------------------------------
requestedLane(Vehicle, Lane):-
  signaledAtFork(Vehicle, Signal, Fork),
  branchOf(Lane, Fork),
  laneCorrectSignal(Lane, Signal).

% vehicle on the planned lane
reservedLaneAtTime(Vehicle, Lane, T):-
  isOnLaneAtTime(Vehicle, Lane, T),
  requestedLane(Vehicle, Lane).

reservedLaneAtTime(Vehicle, Lane1, T):-
  isOnLaneAtTime(Vehicle, Lane2, T),
  requestedLane(Vehicle, Lane2),
  overlaps(Lane2, Lane1),
  not leftLaneByTime(Vehicle, Lane1, T).

%---------------- Rules ---------------
% http://leginfo.legislature.ca.gov/faces/codes_displayText.xhtml?lawCode=VEH&division=11.&title=&part=&chapter=4.&article=
%--------------------------------------
% 21800 (a):
% The driver of a vehicle approaching an intersection shall yield the right-of-way
%  to any vehicle which has entered the intersection from a different highway.
mustYieldToForRuleAtTime(Vehicle1, Vehicle2, yieldToInside, T):-
  atTheIntersectionAtTime(Vehicle1, T),
  enteredByTime(Vehicle2, T),
  arrivedFromDifferentHighways(Vehicle1, Vehicle2),
  requestedLane(Vehicle1, Lane),
  reservedLaneAtTime(Vehicle2, Lane, T).

violatesRightOf(V1, V2):-
  mustYieldToForRuleAtTime(V1, V2, yieldToInside, T),
  requestedLane(V1, Lane),
  reservedLaneAtTime(V2, Lane, T),
  enteredLaneByTime(V1, Lane, T).

% Page 35 of the handbook:
% At intersections without “STOP” or “YIELD” signs,
%  yield to the vehicle or bicycle that arrived first.
mustYieldToForRuleAtTime(Vehicle1, Vehicle2, firstAtFirstIn, T):-
  atTheIntersectionAtTime(Vehicle1, T),
  atTheIntersectionAtTime(Vehicle2, T),
  arrivedEarlierThan(Vehicle2, Vehicle1).

violatesRightOf(V1, V2):-
  mustYieldToForRuleAtTime(V1, V2, firstAtFirstIn, T),
  enteredSameOrEarlierThan(V1, V2).

% 21800 (b)(1)
% When two vehicles enter an intersection from different highways at the same time,
%  the driver of the vehicle on the left shall yield the right-of-way
%  to the vehicle on his or her immediate right.
mustYieldToForRuleAtTime(Vehicle1, Vehicle2, yieldToRight, T):-
  atTheIntersectionAtTime(Vehicle1, T),
  atTheIntersectionAtTime(Vehicle2, T),
  arrivedSameTime(Vehicle1, Vehicle2),
  isOnRightOfAtTime(Vehicle2, Vehicle1, T).

violatesRightOf(V1, V2):-
  mustYieldToForRuleAtTime(V1, V2, yieldToRight, T),
  enteredSameOrEarlierThan(V2, V1).

% 21801 (a)
% The driver of a vehicle intending to turn to the left or to complete a U-turn upon a highway,
%  shall yield the right-of-way to all vehicles approaching from the opposite direction
%  which are close enough to constitute a hazard at any time during the turning movement,
%  and shall continue to yield the right-of-way to the approaching vehicles
%  until the left turn or U-turn can be made with reasonable safety.
mustYieldToForRuleAtTime(Vehicle1, Vehicle2, leftTurn, T):-
  signaledLeft(Vehicle1),
  onSameHighway(Vehicle1, Vehicle2),
  requestedLane(Vehicle1, Lane1),
  requestedLane(Vehicle2, Lane2),
  overlaps(Lane1, Lane2),
  arrivedAtTime(Vehicle2, T1),
  eventTime(T),
  not enteredLaneByTime(Vehicle1, Lane2, T1),
  not leftLaneByTime(Vehicle2, Lane1, T).

violatesRightOf(V1, V2):-
  mustYieldToForRuleAtTime(V1, V2, leftTurn, T),
  requestedLane(V2, L2),
  enteredLaneByTime(V1, L2, T).
%-------------------------------------------------

#show violatesRightOf/2.
#show mustYieldToForRuleAtTime/4.
