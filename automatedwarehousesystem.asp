#include "input_parsing.asp".
#include "input_conversion.asp".

#const n=50.

%OBJECT DECLARATION
move(0,1;0,-1;-1,0;1,0).

{robotMovement(R,move(DX,DY),T):move(DX,DY)}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.
{pickShelf(R,SI,T):shelf(SI)}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.
{dropShelf(R,SI,T):shelf(SI)}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.
{deliverShelf(R,OI,with(SI,PR,DQ),T):orderAt(OI,object(node,ND),contains(PR,OQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), DQ=1..PQ}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.

occurs(object(robot,R),move(DX,DY),T):-robotMovement(R,move(DX,DY),T).
occurs(object(robot,R),pickup,T):-pickShelf(R,_,T).
occurs(object(robot,R),putdown,T):-dropShelf(R,_,T).
occurs(object(robot,R),deliverShelf(OI,PRI,DQ),T):-deliverShelf(R,OI,with(SI,PRI,DQ),T).

%STATE CONSTRAINTS
% Shelves cannot be picked up in a highway
:- pickingStationAt(_,NDI), highway(NDI).

% highway can never contain a shelf.
:- shelfOn(S,object(node,NDI),_), highway(NDI).

% two nodes cannot have the same robot.
:- 2{robotAt(R,object(node,ND),T):node(ND)}, robot(R), T=0..n.

% one node cannot have two robots.
:- 2{robotAt(R,object(node,ND),T):robot(R)}, node(ND), T=0..n.

% no swapping
:- robotAt(R1,object(node,ND1),T), robotAt(R1,object(node,ND2),T+1), robotAt(R2,object(node,ND2),T), robotAt(R2,object(node,ND1),T+1), R1!=R2.

% two robots cannot share the same shelf
:- 2{shelfOn(S,object(robot,NR),T): robot(NR)}, shelf(S), T=0..n.

% one robot cannot have two shelves
:- 2{shelfOn(S,object(robot,NR),T): shelf(S)}, robot(NR), T=0..n.

% two nodes cannot share a shelf.
:- 2{shelfOn(S,object(node,ND),T): node(ND)}, shelf(S), T=0..n.

% a node cannot contain two shelves
:- 2{shelfOn(S,object(node,ND),T): shelf(S)}, node(ND), T=0..n.

% No shelf on 2 locations (robot, node)
:- shelfOn(S,object(node,_),T), shelfOn(S,object(robot,_),T).

%ACTION CONSTRAINTS
% no parallelizing actions.
:- occurs(object(robot,R),A1,T), occurs(object(robot,R),A2,T), A1!=A2.

:- 2{pickShelf(R,S,T): robot(R)}, shelf(S).

% it is not possible to pick a shelf if already having one.
:- pickShelf(RI,S1,T), shelfOn(S2,object(robot,RI),T).
:- pickShelf(R1,S,T), shelfOn(S,object(robot,R2),T).

:- pickShelf(RI,S,T), shelfOn(S,object(node,ND),T), not robotAt(RI,object(node,ND),T). 

% robot and grid restrictions
:- robotAt(RI,object(node,ND),T), robotMovement(R,move(DX,DY),T), nodeAt(ND,pair(X,Y)), X+DX<1.
:- robotAt(RI,object(node,ND),T), robotMovement(R,move(DX,DY),T), nodeAt(ND,pair(X,Y)), Y+DY<1.
:- robotAt(RI,object(node,ND),T), robotMovement(R,move(DX,DY),T), nodeAt(ND,pair(X,Y)), X+DX>NC, numColumns(NC).
:- robotAt(RI,object(node,ND),T), robotMovement(R,move(DX,DY),T), nodeAt(ND,pair(X,Y)), Y+DY>NR, numRows(NR).

% 2 robots cannot drop a shelf
:- 2{dropShelf(R,S,T): robot(R)}, shelf(S).

% robot cannot drop a shelf when it has another shelf.
:- dropShelf(RI,S,T), not shelfOn(S,object(robot,RI),T).

% no dropping on a highway.
:- dropShelf(RI,S,T), robotAt(RI,object(node,ND),T), highway(ND). 

% only a robot in a picking station can deliver.
:- deliverShelf(R,OI,with(_,PR,_),T), orderAt(OI,object(node,ND),contains(PR,_),T), not robotAt(R,object(node, ND),T).

% only a robot having a product can deliver it.
:- deliverShelf(R,OI,with(SI,PR,_),T), productOn(PR,object(shelf,SI),with(quantity,_),T), not shelfOn(SI,object(robot,R),T).

% order count cannot be exceeded by delivery count.
:- deliverShelf(R,OI,with(SI,PR,DQ),T), orderAt(OI,object(node,ND),contains(PR,OQ),T), DQ>OQ.

% delivery count cannot exceed the product count.
:- deliver(R,OI,with(SI,PR,DQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), DQ>PQ.


%EFFECTS AND PRECONDITIONS OF ACTIONS
% result of moving a robot
robotAt(R,object(node,NEW_ND),T+1):- robotAt(R,object(node,ND),T), nodeAt(ND,pair(X,Y)), nodeAt(NEW_ND, pair(X+DX,Y+DY)), robotMovement(R,move(DX,DY),T).

% result of picking a shelf
shelfOn(S,object(robot,RI),T+1):- pickShelf(RI,S,T), shelfOn(S,object(node,ND),T), robotAt(RI,object(node,ND),T).

%result of dropping a shelf
shelfOn(S,object(node,ND),T+1):- dropShelf(RI,S,T), shelfOn(S,object(robot,RI),T), robotAt(RI,object(node,ND),T).

%result of delivering
orderAt(OI,object(node,ND),contains(PR,OU-DQ),T+1):- deliverShelf(R,OI,with(SI,PR,DQ),T), orderAt(OI,object(node,ND),contains(PR,OU),T).
productOn(PR,object(shelf,SI),with(quantity,PQ-DQ),T+1):- deliverShelf(R,OI,with(SI,PR,DQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T).

%DOMAIN INDEPENDENT AXIOMS
robotAt(R,object(node,ND),T+1):- robotAt(R,object(node,ND),T), not robotMovement(R,move(_,_),T), T<n.

% inertial constraints.
shelfOn(S,object(node,ND),T+1):-shelfOn(S,object(node,ND),T), not pickShelf(_,S,T), T<n.
shelfOn(S,object(robot,RI),T+1):-shelfOn(S,object(robot,RI),T), not dropShelf(RI,S,T), T<n.
orderAt(OI,object(node,ND),contains(PR,OU),T+1):- orderAt(OI,object(node,ND),contains(PR,OU),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), not deliverShelf(_,OI,with(SI,PR,_),T), T<n.
productOn(PR,object(shelf,SI),with(quantity,PQ),T+1):- productOn(PR,object(shelf,SI),with(quantity,PQ),T), not deliverShelf(_,_,with(SI,PR,_),T), T<n.

%result
:- not orderAt(OI,object(node,_),contains(PR,0),n), orderAt(OI,object(node,_),contains(PR,_),0).

numActions(N):-N=#sum{1,O,A,T:occurs(O,A,T)}.
timeTaken(N-1):-N=#count{T:occurs(O,A,T)}.
#minimize{1,O,A,T:occurs(O,A,T)}.
#minimize{T:occurs(O,A,T)}.

%#show node/1.
%#show robot/1.
%#show shelf/1.
%#show product/1.
%#show order/1.
%#show nodeAt/2.
%#show robotAt/3.
%#show shelfOn/3.
%#show productOn/4.
%#show orderAt/4.

%#show robotMovement/3.

#show occurs/3.
#show numActions/1.
#show timeTaken/1.