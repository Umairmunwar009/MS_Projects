Obstacle    = [-600    -400         200; -150    -150       200; 300    300         200;650    600      200;1 1 1];[X,Y,Z] = peaks(25);

[X,Y,Z] = peaks(25);
CO(:,:,1) = zeros(25); % red
CO(:,:,2) = ones(25).*linspace(0.5,0.6,25); % green
CO(:,:,3) = ones(25).*linspace(0,1,25); % blue
surf(X,Y,Z,CO)
% viscircles(Obstacle(:,1:2),Obstacle(:,3));