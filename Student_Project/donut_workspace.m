%create workspace for half donut
        u=linspace(0,pi,100);
        v=linspace(0,2*pi,100);
        [U,V]=meshgrid(u,v);
        X=(3+cos(V))*cos(U)/400*1000;
        Y=(3+cos(V))*sin(U)/400*1000;
        Z=(sin(V)/4+0.5)*1000;
        surf(X,Y,Z,'FaceAlpha',0.5,'EdgeColor','none');hold on; %creates a semitransparent surface with no edges drawn. 
        %CO(:,:,1) = zeros(25); % red
        %CO(:,:,2) = ones(25).*linspace(0.5,0.6,25); % green
        %CO(:,:,3) = ones(25).*linspace(0,1,25); % blue
        %surf(X,Y,Z,CO);
        figure(1);title('Work Space')
        subplot(2,2,1);surf(X,Y,Z,'FaceAlpha',0.5,'EdgeColor','none'); %creates a semitransparent surface with no edges drawn.
        subplot(2,2,2);plot(X,Y);title('X vs. Y');xlabel('X-Cordinate'); ylabel('Y_cordinate');hold on;
        subplot(2,2,3); plot(X,Z);title('X vs. Z');xlabel('X-Cordinate'); ylabel('Z_cordinate');hold on;
        subplot(2,2,4); plot(Y,Z);title('Y vs. Z');xlabel('Y-Cordinate'); ylabel('Z_cordinate');hold on;
        