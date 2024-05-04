% Grid row count
numColumns(NR):- NR=#count{X:init(object(node,I),value(at,pair(X,Y)))}.

% grid column count
numRows(NC):- NC=#count{Y:init(object(node,I),value(at,pair(X,Y)))}.

% robot count
numRobots(ND):- ND=#count{I:init(object(robot,I),value(at,pair(X,Y)))}.