%----------------------------------------------
%---Auxiliary predicates for geometric facts---
%----------------------------------------------
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
% 21800 (b)(1)
% When two vehicles enter an intersection from different highways at the same time,
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

