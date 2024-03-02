% define data to animate
% this will move a rectangle with fixed width and variable length 
% around to specified positions

time =      out.Pos.Time;
xposition = out.Pos.Data(:,1);
yposition = out.Pos.Data(:,2);
theta = out.Pos.Data(:,3);
r_width=.5;
r_length=1;

% Define shape verticies first element is x position, second element is y
% position
v = [-.5 -.5  0 0  -.25 -.25 .25 .25  0  0  .5  .5   0   0  -.25 -.25  .25  .25   0    0;
     -.4  .4 .4 .45 .45  .55 .55 .45 .45 .4 .3 -.3 -.4 -.45 -.45 -.55 -.55 -.45 -.45 -.4];
v = diag([r_length r_width])*v;
clf
fig=gcf;
ax=axes;
axis([-5 5 -5 5]) % set axis to have specified x and y limits 
    % (type 'help axis' for more info)
xlabel('x position')
ylabel('y position')
hold on
for i=1:length(time),
    starttime=tic;
    % move shape by moving verticies
    % define rotation matrix
    T = [cos(theta(i)) -sin(theta(i));sin(theta(i)) cos(theta(i))];
    % define center position
    pos = [xposition(i);yposition(i)];
    % find position of current verticies
    v_c = T*v+pos*ones(1,size(v,2));
    % draw shape
    cla(ax)
    plot(ax,xposition(1:i),yposition(1:i),'-')
    fill(ax,v_c(1,:),v_c(2,:),'y')
    % make sure matlab draws the figure now
    drawnow
    % if not last drawing, wait
     if i<length(time),
         elapsedtime=toc(starttime);
         if elapsedtime<(time(i+1)-time(i))
           pause(time(i+1)-time(i)-elapsedtime)
        end
     end;
end;
    

    

