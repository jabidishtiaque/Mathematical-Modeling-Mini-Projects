function parking
clear all; clc;

% [x0,x(1:N-1)] position x
% x(N:2*N-1) position y
% x(2*N:3*N-1) theta
% x(3*N:4*N-1) v
% x(4*N:5*N-1) phi
% x(5*N:6*N-1) u1
% x(6*N:7*N-1) u2
 

T      = 10;
N      = 100;
h      = T/N;

% track
% sharp turning for parking
% t1      = -linspace(0,5,50);
% t2 = t1(50)*ones(1,50);
% t = horzcat(t1,t2);
% yd1 = t1(50)*ones(1,50);
% yd2 = linspace(yd1(50),-T,50);
% yd = horzcat(yd1,yd2);

% large turning radius
% t = -linspace(0,T,N);
% yd = -0.1*t.^2;


% parallel parking
t      = -linspace(0,10,100);
yd2 = t(26:75);
yd1 = yd2(1)*ones(1,25);
yd3 = yd2(50)*ones(1,25);
yd = horzcat(yd1,yd2,yd3);
%plot (t,yd)

x0 = t(1);

 
beta1  = 1e-20;
beta2  = 1e-20;
f         = @(x,h) (h/2)*((x(N)-yd(1))^2 + 2*sum((x(N+1:2*N-2) - yd(2:N-1)).^2) + (x(2*N-1) - yd(N))^2) + (h/2)*((x0-t(1))^2 + 2*sum((x(1:N-2) - t(2:N-1)).^2) + (x(N-1) - t(N))^2) + beta1*(h/2)*(x(5*N)^2+2*sum(x(5*N:6*N-2).^2) + x(6*N-1)^2) + beta2*(h/2)*(x(6*N)^2+2*sum(x(6*N:7*N-2).^2) + x(7*N-1)^2);
%f         = @(x,h) (x(N)-yd(1))^2 + (h/2)*sum((x(N+1:2*N-2) - yd(2:N-1)).^2) + (x(2*N-1) - yd(N))^2 + sqrt((x0-t(1))^2 + (h/2)*sum((x(1:N-2) - t(2:N-1)).^2) + (x(N-1) - t(N))^2) + beta1*sum(abs(x(5*N:6*N-1))) + beta2*sum(abs(x(6*N:7*N-1)));
% load('xsol.mat');
y0        = 1e-5*ones(1,699);
options   = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunEvals',300000,'MaxIter',200,'StepTolerance',1.0000e-20);%,'TolCon',1e-10,'TolX',1e-15);

% [x,fval] = fmincon(fun,         x0, A,b,Aeq,beq,lb,ub,   nonlcon,           options)
[x, fval] =   fmincon(@(x)f(x,h),y0,[],[],[],[],[],[],@(x) confun(x,x0,N,h),options);



figure(1)
%subplot(2,3,1)
plot([x0,x(1:N-1)],x(N:2*N-1),'k',t,yd,'k--','LineWidth',1.5)
legend({'actual','desired'},'FontSize',12)
xlabel({'x'},'Interpreter','tex','FontSize',12)
ylabel({'y'},'Interpreter','tex','FontSize',12)
title('Desired Path and Vehicle Motion','FontSize',14)
%xlim([0 T])
axis square
grid on

%subplot(2,3,2)
plot(-t,x(5*N:6*N-1),'k','LineWidth',1.5)
legend({'u_1'},'Interpreter','tex','FontSize',12)
xlabel({'\tau'},'Interpreter','tex','FontSize',12)
title('Control for Steering Change Rate','FontSize',14)
axis square
grid on

%subplot(2,3,3)
plot(-t,x(6*N:7*N-1),'k','LineWidth',1.5)
legend({'u_2'},'Interpreter','tex','FontSize',12)
xlabel({'\tau'},'Interpreter','tex','FontSize',12)
title('Control for Vehicle Acceleration Rate','FontSize',14)
axis square
grid on

%subplot(2,3,4)
plot(-t,x(4*N:5*N-1),'k','LineWidth',1.5)
legend({'\phi'},'Interpreter','tex','FontSize',12)
xlabel({'\tau'},'Interpreter','tex','FontSize',12)
title('Steering Angle','FontSize',14)
axis square
grid on

%subplot(2,3,5)
plot(-t,x(3*N:4*N-1),'k','LineWidth',1.5)
legend({'v'},'Interpreter','tex','FontSize',12)
xlabel({'\tau'},'Interpreter','tex','FontSize',12)
title('Vehicle Velocity','FontSize',14)
axis square
grid on

%subplot(2,3,6)
plot(-t,x(2*N:3*N-1),'k','LineWidth',1.5)
legend({'\theta'},'Interpreter','tex','FontSize',12)
xlabel({'\tau'},'Interpreter','tex','FontSize',12)
title('Orientation of the Vehicle','FontSize',14)
axis square
grid on


la = 2.578;  %wheelbase
lf = 0.965;
lr = 0.965;
w = 1.61/2; %half of track width
xx = [x0,x(1:N-1)];
yy = x(N:2*N-1);
th = x(2*N:3*N-1);

for i=1:length(xx)
figure(1)    
    x = xx(i);
    z = yy(i);
    %theta = pi/3-atan(yy(i+1)-yy(i)/xx(i+1)-xx(i));
    theta=th(i);

    Ax = x + sqrt(w^2+(la+lf)^2)*cos(theta+atan(w/(la+lf)));
    Ay = z+ sqrt(w^2+(la+lf)^2)*sin(theta+atan(w/(la+lf)));
    Bx = x+sqrt(w^2+(la+lf)^2)*cos(theta-atan(w/(la+lf)));
    By = z+sqrt(w^2+(la+lf)^2)*sin(theta-atan(w/(la+lf)));
    
    Cx = x + sqrt(w^2+lr^2)*cos(pi - theta-atan(w/lr));
    Cy = z - sqrt(w^2+lr^2)*sin(pi - theta-atan(w/lr));
    Dx = x - sqrt(w^2+lr^2)*cos(atan(w/lr) -theta);
    Dy = z + sqrt(w^2+lr^2)*sin(atan(w/lr) -theta);
    hold on

    plot(x,z,'b*',[Ax,Bx],[Ay,By],'k',[Cx,Dx],[Cy,Dy],'k',[Ax,Dx],[Ay,Dy],'k',[Cx,Bx],[Cy,By],'k');
    %plot(x,z,'b*');
    %axis([-8 4 -11 -2]); 
    pause(0.1);
    hold off
%     F(i) = getframe(gcf) ;
    drawnow;
end
% % create the video writer with 1 fps
% writerObj = VideoWriter('parallelparking.avi');
% writerObj.FrameRate = 10;
% % set the seconds per image
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(F)
%     % convert the image to a frame
%     frame = F(i) ;    
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);

end


function [c, ceq] = confun(x,x0,N,h)

% [x0,x(1:N-1)] position x
% x(N:2*N-1) position y
% x(2*N:3*N-1) theta
% x(3*N:4*N-1) v
% x(4*N:5*N-1) phi -angular acceleration
% x(5*N:6*N-1) u1
% x(6*N:7*N-1) u2
l  = 2.578;

% Nonlinear inequality constraints
c    = [-0.4-[x(5*N:6*N-1)]';
        [x(5*N:6*N-1)-0.4]';
        -11.5-[x(6*N:7*N-1)]';
        [x(6*N:7*N-1)-11.5]';
        -1.006-[x(4*N:5*N-1)]';
        [x(4*N:5*N-1)-1.006]';
        -13.6-[x(3*N:4*N-1)]';
        [x(3*N:4*N-1)-50.8]'];

% Nonlinear equality constraints
ceq  = [[-x(1:N-1) + [x0,x(1:N-2)] - (h/2)*(  x(3*N:4*N-2).*cos(x(2*N:3*N-2)) + x(3*N+1:4*N-1).*cos(x(2*N+1:3*N-1)) )]';
    [-x(N+1:2*N-1) + x(N:2*N-2) - (h/2)*(  x(3*N:4*N-2).*sin(x(2*N:3*N-2)) + x(3*N+1:4*N-1).*sin(x(2*N+1:3*N-1)) )]';
    [-x(2*N+1:3*N-1) + x(2*N:3*N-2) - (h/2)*(  (x(3*N:4*N-2)/l).*tan(x(4*N:5*N-2)) + (x(3*N+1:4*N-1)/l).*tan(x(4*N+1:5*N-1))  )]';
    [-x(3*N+1:4*N-1) + x(3*N:4*N-2) - (h/2)*(  x(6*N:7*N-2) + x(6*N+1:7*N-1)  )]'; %velocity = u2
    [-x(4*N+1:5*N-1) + x(4*N:5*N-2) - (h/2)*(  x(5*N:6*N-2) + x(5*N+1:6*N-1)  )]']; %phi -angular acceleration = u1

end