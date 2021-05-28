%----------------------------------------------
%---Auxiliary predicates for geometric facts---
%----------------------------------------------
onSameHighway(Fork1, Fork2):-
  isOnRightOf(Fork1, F),
  isOnRightOf(F, Fork2).

onSameHighway(Fork1, Fork2):-
  onSameHighway(Fork2, Fork1).

onDifferentHighways(F1, F2):-
  isOnRightOf(F1, F2).

onDifferentHighways(F1, F2):-
  isOnRightOf(F2, F1).

branchOf(Lane, Fork):-
  laneFromTo(Lane, Fork, _).

%---------------------------------------------
%---Auxiliary predicates for traffic events---
%---------------------------------------------
arrivedAtFork(Vehicle, Fork):-
  arrivedAtForkAtTime(Vehicle, Fork, _).

arrivedAtTime(V, T):-
  arrivedAtForkAtTime(V, _, T).

signaledAtFork(V, S, F):-
  signaledAtForkAtTime(V, S, F, _).

signaledLeft(Vehicle):-
  signaledAtFork(Vehicle, left, _).

arrivedFromSameHighway(V1, V2):-
  arrivedAtFork(V1, F1), arrivedAtFork(V2, F2),
  onSameHighway(F1, F2).

%--------------------------------------------
%---Auxiliary predicates for traffic rules---
%--------------------------------------------
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
violatesRightOfForRule(V1, V2, yieldToInside):-
  V1 != V2,
  enteredForkAtTime(V1, F1, Te1),
  enteredForkAtTime(V2, F2, Te2),
  onDifferentHighways(F1, F2),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  lessThan(Te2, Te1),
  leftLaneAtTime(V2, L1, T1),
  enteredLaneAtTime(V1, L2, T2),
  lessThan(T2, T1).

% If two vehicles arrive from different highways and request overlapping lanes,
% then their entrance times are relevant to the right-of-way.
#count {0:equal(Te1, Te2); 1:lessThan(Te1, Te2); 2:lessThan(Te2, Te1) } = 1 :- 
  V1 != V2,
  enteredForkAtTime(V1, F1, Te1),
  enteredForkAtTime(V2, F2, Te2),
  onDifferentHighways(F1, F2),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2).

% Furthermore,
% if they did not enter the intersection simultaneously,
% then the leaving time, from the overlap, of the earlier enterer to the intersection,
% and the entering time, to the overlap, of the later enterer to the intersection,
% are relevant to the right of way.
#count {0:equal(T1, T2); 1:lessThan(T1, T2); 2:lessThan(T2, T1) } = 1 :-
  V1 != V2,
  enteredForkAtTime(V1, F1, Te1),
  enteredForkAtTime(V2, F2, Te2),
  onDifferentHighways(F1, F2),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  lessThan(Te2, Te1),
  leftLaneAtTime(V2, L1, T1),
  enteredLaneAtTime(V1, L2, T2).

%---------------------------------------------------------------------------------
% 21800 (c)
% When two vehicles enter an intersection from different highways at the same time 
%  and the intersection is controlled from all directions by stop signs, 
%  the driver of the vehicle on the left shall yield the right-of-way 
%  to the vehicle on his or her immediate right.
violatesRightOfForRule(V1, V2, yieldToRight):-
  V1 != V2,
  enteredForkAtTime(V1, F1, Te1),
  enteredForkAtTime(V2, F2, Te2),
  isOnRightOf(F2, F1),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  equal(Te1, Te2; Te2, Te1),
  leftLaneAtTime(V2, L1, T1),
  enteredLaneAtTime(V1, L2, T2),
  lessThan(T2, T1).

% If two vehicles enter an all-way stop from different highways,
% and one highway is on the right of the other,
% then their entrance times are relevant to the right of way.
#count {0:equal(Te1, Te2); 1:lessThan(Te1, Te2); 2:lessThan(Te2, Te1) } = 1 :- 
  V1 != V2,
  enteredForkAtTime(V1, F1, Te1),
  enteredForkAtTime(V2, F2, Te2),
  isOnRightOf(F2, F1),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2).

% Furthermore,
% if they entered the intersection simultaneously,
% then the leaving time, from the overlap, of the vehicle on the right,
% and the entering time, to the overlap, of the other vehicle,
% are relevant to the right of way.
#count {0:equal(T1, T2); 1:lessThan(T1, T2); 2:lessThan(T2, T1) } = 1 :- 
  V1 != V2,
  enteredForkAtTime(V1, F1, Te1),
  enteredForkAtTime(V2, F2, Te2),
  isOnRightOf(F2, F1),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  equal(Te1, Te2; Te2, Te1),
  leftLaneAtTime(V2, L1, T1),
  enteredLaneAtTime(V1, L2, T2).

%--------------------------------------------------------------------------------------------
% 21801 (a)
% The driver of a vehicle intending to turn to the left or to complete a U-turn upon a highway,
%  shall yield the right-of-way to all vehicles approaching from the opposite direction
%  which are close enough to constitute a hazard at any time during the turning movement,
%  and shall continue to yield the right-of-way to the approaching vehicles
%  until the left turn or U-turn can be made with reasonable safety.
violatesRightOfForRule(V1, V2, leftTurn):-
  V1 != V2,
  signaledLeft(V1),
  arrivedFromSameHighway(V1, V2),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  arrivedAtTime(V2, Ta),
  enteredLaneAtTime(V1, L2, Te1),
  lessThan(Ta, Te1),
  leftLaneAtTime(V2, L1, Tl2),
  lessThan(Te1, Tl2).

#count {0:equal(Ta, Te1); 1:lessThan(Ta, Te1); 2:lessThan(Te1, Ta)} = 1 :-
  V1 != V2,
  signaledLeft(V1),
  arrivedFromSameHighway(V1, V2),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  arrivedAtTime(V2, Ta),
  enteredLaneAtTime(V1, L2, Te1).

#count {0:equal(Tl2, Te1); 1:lessThan(Tl2, Te1); 2:lessThan(Te1, Tl2)} = 1 :-
  V1 != V2,
  signaledLeft(V1),
  arrivedFromSameHighway(V1, V2),
  requestedLane(V1, L1),
  requestedLane(V2, L2),
  overlaps(L1, L2),
  arrivedAtTime(V2, Ta),
  enteredLaneAtTime(V1, L2, Te1),
  lessThan(Ta, Te1),
  leftLaneAtTime(V2, L1, Tl2).

% 22450 (a):
% The driver of any vehicle approaching a stop sign at the entrance to, or within, an intersection
%  shall stop at a limit line, if marked,
%  otherwise before entering the crosswalk on the near side of the intersection.
% If there is no limit line or crosswalk,
%  the driver shall stop at the entrance to the intersecting roadway.
% 21802 (a):
% The driver of any vehicle approaching a stop sign at the entrance to, or within, an intersection
%  shall stop as required by Section 22450.
stoppedAtFork(V, F):-
  stoppedAtForkAtTime(V, F, T).

violatesRule(V, stopAtSign):-
  arrivedAtFork(V, F),
  hasStopSign(F),
  not stoppedAtFork(V, F).

% 22526 (a) (Signalized intersection?)
% Notwithstanding any official traffic control signal indication to proceed,
%  a driver of a vehicle shall not enter an intersection or marked crosswalk
%  unless there is sufficient space on the other side of the intersection or marked crosswalk
%  to accommodate the vehicle driven without obstructing the through passage of vehicles from either side.
% mustYieldToForRuleAtTime(V1, V2, antiGridlock, T):-
%   V1 != V2,
%   requestedLane(V1, L1),
%   requestedLane(V2, L2),
%   laneFromTo(L1, _, E),
%   laneFromTo(L2, _, E),
%   enteredAtTime(V1, T1),
%   enteredAtTime(V2, T2),
%   T1 > T2,
%   time(T),
%   T >= T1,
%   not leftLaneByTime(V2, L2, T).

% violatesRightOfForRule(V1, V2, antiGridlock):-
%   mustYieldToForRuleAtTime(V1, V2, antiGridlock, T),
%   enteredByTime(V1, T).

%-------------------------------------------------
violatesRightOf(V1, V2):-
  violatesRightOfForRule(V1, V2, _).

