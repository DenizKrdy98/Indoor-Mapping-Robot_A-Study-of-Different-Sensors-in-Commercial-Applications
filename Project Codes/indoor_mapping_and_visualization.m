%%
%clear
close all


%% INITIALIZATIONS
%s = serialport("COM8",9600);    % Bluetooth Connection
%A = zeros(5,300);
d = zeros(2,300);
i = 1;
%s.writeline('start');           % Bluetooth Start Command
x=0;
y=0;

%% ** CHANGE HERE FOR EACH SET-UP **
% Arranges Plotting Parameters and Mapping Inputs According to Set-up
FrontSide = 2;
RightSide = 3;
BackSide = 4;
LeftSide = 1;
% colors
frontCol = "c";                 
backCol = "g";
rightCol = "r";
leftCol = "b";
setup = 'UV-IR-Lidar-Laser';    % for title

%% Single Figure
i=1;
figure
while i<128
    % Bluetooth Data Delivery and Assignments
%     i;     
%       data = readline(s);
%       trimmed_data = split(data);
%       trimmed_data(6) = [];
%       B = str2double(trimmed_data);
%       B = B'     
%     
%       % Bluetooth Data Delivery
%       A(1,i) = B(1); % Laser
%       A(2,i )= B(2); % UV
%       A(3,i) = B(3); % IR
%       A(4,i) = B(4); % LIDAR
%       A(5,i) = B(5); % State
    
    
    % MAPPING
    if i==1
        front1 = A(FrontSide,1)+5;
        right1 = A(RightSide,1)+5;
        back1 = A(BackSide,1)+5;
        left1 = A(LeftSide,1)+5;
        
        front_1 = front1;
        right_1 = right1;
        back_1 = back1;
        left_1 = left1;
        
        plot([0, right1],[-back1, -back1+1],rightCol,"LineWidth",2);
        hold on
        plot([0,-left1],[-back1,-back1+1],leftCol,"LineWidth",2);
        hold on
        plot([-left1,-left1],[-back1,0],leftCol,"LineWidth",2);
        hold on
        plot([right1,right1],[-back1,0],rightCol,"LineWidth",2);
    else
        front_1 = A(FrontSide,i-1)+5;
        right_1 = A(RightSide,i-1)+5;
        back_1 = A(BackSide,i-1)+5;
        left_1 = A(LeftSide,i-1)+5;
    end
    
    fronti = A(FrontSide,i)+5;
    righti = A(RightSide,i)+5;
    backi = A(BackSide,i)+5;
    lefti = A(LeftSide,i)+5;
    
    
    statei = A(5,i);
    if i ~= 1
        state_1 = A(5,i-1);
    else
        state_1 = 0;
    end
    
    % PLOTTING
    x_old = x;
    y_old = y;
    switch statei
        case 1            
            [x,y,backi,fronti,righti,lefti,front_1,back_1,right_1,left_1] = forward_y(x_old,y_old,x,y,front_1,back_1,right_1,left_1,fronti,backi,righti,lefti,statei,state_1,frontCol,backCol,rightCol,leftCol,i);            
            if(i==2)
                title(['Mapping With Setup: ', setup ]);
                xlabel('X (cm)');
                ylabel('Y (cm)');
                grid on                
            else
                scatter(x,y,20,"k^",'filled');
            end
            
        case 2
            [x,y,backi,fronti,righti,lefti,front_1,back_1,right_1,left_1] = forward_x(x_old,y_old,x,y,front_1,back_1,right_1,left_1,fronti,backi,righti,lefti,statei,state_1,frontCol,backCol,rightCol,leftCol);            
            scatter(x,y,20,"k>",'filled');
        case 3
            [x,y,backi,fronti,righti,lefti,front_1,back_1,right_1,left_1] = backward_y(x_old,y_old,x,y,front_1,back_1,right_1,left_1,fronti,backi,righti,lefti,statei,state_1,frontCol,backCol,rightCol,leftCol);            
            scatter(x,y,20,"kv",'filled');
        case 4
            [x,y,backi,fronti,righti,lefti,front_1,back_1,right_1,left_1] = backward_x(x_old,y_old,x,y,front_1,back_1,right_1,left_1,fronti,backi,righti,lefti,statei,state_1,frontCol,backCol,rightCol,leftCol);            
            scatter(x,y,20,"k<",'filled');
    end
    
    for j = 1:4
        axis([-100 250 -100 250]);
        hold on;
        plot([x_old,x],[y_old,y],"k:","LineWidth",1);
    end
    
    drawnow;
    i=i+1;
end






%% FUNCTIONS

%%
% Mapper for Paths in Y-Direction
function [x,y,backi,fronti,righti,lefti,front_1,back_1,right_1,left_1] = forward_y(x_old,y_old,x,y,front_1,back_1,right_1,left_1,fronti,backi,righti,lefti,statei,state_1,frontCol,backCol,rightCol,leftCol,i)
    
    % Map and Localization Calculations
    left_raw = lefti;
    right_raw = righti;
    x = x;
    if state_1 == statei
        y = y+front_1-fronti;
    else
        y = y;
    end
    backi = y-backi;
    fronti = y+fronti;
    righti = x + righti;
    lefti = (x - lefti);


    % Scatters    
    if(i==2)
        p1 = scatter(x,fronti,frontCol,"LineWidth",2,'DisplayName',sensName(frontCol));        
        hold on;
        p2 = scatter(x,backi,backCol,"LineWidth",2,'DisplayName',sensName(backCol));
        hold on;    
        p3 =  scatter(lefti,y,leftCol,"LineWidth",2,'DisplayName',sensName(leftCol));
        hold on;
        p4 = scatter(righti,y,rightCol,"LineWidth",2,'DisplayName',sensName(rightCol));
        hold on;
        p = scatter(x,y,20,"k^",'filled','DisplayName','Robot');        
        legend([p1 p2 p3 p4 p],{sensName(frontCol), sensName(backCol), sensName(leftCol), sensName(rightCol), 'Robot'},'AutoUpdate','off');
    else
        scatter(x,fronti,frontCol,"LineWidth",2);
        hold on;
        scatter(x,backi,backCol,"LineWidth",2);
        hold on;    
        scatter(lefti,y,leftCol,"LineWidth",2);
        hold on;
        scatter(righti,y,rightCol,"LineWidth",2);
        hold on;        
    end  
    
    % Plot Cases
    if abs(left_raw-left_1)<30 && (state_1 == statei)
        plot([(x_old-left_1),lefti],[y_old,y],leftCol,"LineWidth",2);
        hold on;
    end   
    
    if abs(right_raw-right_1)<30 && (state_1 == statei)
        plot([(x_old+right_1),righti],[y_old,y],rightCol,"LineWidth",2);
        hold on;
    end
    
    % Corner Cases
    if state_1 == 4
        plot([lefti,x],[y-1-left_1,y-left_1],leftCol,"LineWidth",2);
        hold on
        plot([x-left_raw-1,x-left_raw],[y-left_1,y],leftCol,"LineWidth",2);
        hold on
    end
    if state_1 == 2
        plot([righti,righti+1],[y-right_1,y],rightCol,"LineWidth",2);
        hold on;
        plot([x,righti],[y-1-right_1,y-right_1],rightCol,"LineWidth",2);
        hold on
    end
end

%%
% Mapper for Paths in (-Y)-Direction
function [x,y,backi,fronti,righti,lefti,front_1,back_1,right_1,left_1] = backward_y(x_old,y_old,x,y,front_1,back_1,right_1,left_1,fronti,backi,righti,lefti,statei,state_1,frontCol,backCol,rightCol,leftCol)
    % Map and Localization Calculations
    left_raw = lefti;
    right_raw = righti;
    x = x;
    if state_1 == statei
        y = y-(front_1-fronti);
    else
        y = y;
    end
    backi = y+backi;
    fronti= y-fronti;
    righti = x - righti;
    lefti = (lefti+x);
    
    % Scatters     
    scatter(x,fronti,frontCol,"LineWidth",2);
    hold on;    
    scatter(x,backi,backCol,"LineWidth",2);
    hold on;    
    scatter(lefti,y,leftCol,"LineWidth",2);
    hold on;    
    scatter(righti,y,rightCol,"LineWidth",2);
    hold on;
    
    % Plot Cases
    if abs(left_raw-left_1)<30 && (state_1 == statei)
        plot([(x_old+left_1),lefti],[y_old,y],leftCol,"LineWidth",2);
        hold on;
    end
    if abs(right_raw-right_1)<30 && (state_1 == statei)
        plot([(x_old-right_1),righti],[y_old,y],rightCol,"LineWidth",2);
        hold on;
    end
    
    % Corner Cases
    if state_1 == 2
        plot([lefti-1,lefti],[y,y+left_1],leftCol,"LineWidth",2);
        hold on;
        plot([x,x+left_raw],[y+left_1-1,y+left_1],leftCol,"LineWidth",2);
	    hold on
    end
     if state_1 == 4
        plot([x-right_raw,x],[y+right_1-1,y+right_1],rightCol,"LineWidth",2);
        hold on;
       plot([x-right_raw-1,x-right_raw],[y+right_1,y],rightCol,"LineWidth",2);
       hold on
    end
end

%%
% Mapper for Paths in X-Direction
function [x,y,backi,fronti,righti,lefti,front_1,back_1,right_1,left_1] = forward_x(x_old,y_old,x,y,front_1,back_1,right_1,left_1,fronti,backi,righti,lefti,statei,state_1,frontCol,backCol,rightCol,leftCol)
    % Map and Localization Calculations
    left_raw = lefti;
    right_raw = righti;
    if state_1 == statei
        x = x+front_1-fronti;
    else
        x = x;
    end
    lefti = y+lefti;
    righti = y-righti;
    fronti = x + fronti;
    backi = x-backi;
    
    % Scatters
    scatter(fronti,y,frontCol,"LineWidth",2);
    hold on;    
    scatter(backi,y,backCol,"LineWidth",2);
    hold on;
    scatter(x,lefti,leftCol,"LineWidth",2);
    hold on;,
    scatter(x,righti,rightCol,"LineWidth",2);
    hold on;
    
    % Plot Cases
    if abs(left_raw-left_1)<30 && (state_1 == statei)
        plot([x_old,x],[(y_old+left_1),lefti],leftCol,"LineWidth",2);
        hold on;
    end    
    if abs(right_raw-right_1)<30 && (state_1 == statei)
        plot([x_old,x],[(y_old-right_1),righti],rightCol,"LineWidth",2);
        hold on;
    end
    
    % Corner Cases
    if state_1 == 1
        plot([x,x-left_1],[lefti-1,lefti],leftCol,"LineWidth",2);
        hold on
        plot([x_old-left_1-1,x_old-left_1],[y,lefti],leftCol,"LineWidth",2);
        hold on
    end
    if state_1 == 3
        plot([x-right_1, x],[y-right_raw-1,y-right_raw],rightCol,"LineWidth",2);
        hold on
        plot([x-right_1-1,x-right_1],[y,y-right_raw],rightCol,"LineWidth",2);
        hold on
    end
end

%%
% Mapper for Paths in (-X)-Direction
function [x,y,backi,fronti,righti,lefti,front_1,back_1,right_1,left_1] = backward_x(x_old,y_old,x,y,front_1,back_1,right_1,left_1,fronti,backi,righti,lefti,statei,state_1,frontCol,backCol,rightCol,leftCol)
    % Map and Localization Calculations
    left_raw = lefti;
    right_raw = righti;
    y = y;
    if state_1 == statei
        x = x-(front_1-fronti);
    else
        x=x;
    end
    lefti = y-lefti;
    righti = y+righti;
    fronti = x - fronti;
    backi = x+backi;
    
   % Scatters
    scatter(fronti,y,frontCol,"LineWidth",2);
    hold on;   
    scatter(backi,y,backCol,"LineWidth",2);
    hold on;    
    scatter(x,lefti,leftCol,"LineWidth",2);
    hold on;
    scatter(x,righti,rightCol,"LineWidth",2);
    hold on;
    
    % Plot Cases
    if abs(left_raw-left_1)<30 && (state_1 == statei)
        plot([x_old,x],[(y_old-left_1),lefti],leftCol,"LineWidth",2);
        hold on;
    end    
    if abs(right_raw-right_1)<30 && (state_1 == statei)
        plot([x_old,x],[(y_old+right_1),righti],rightCol,"LineWidth",2);
        hold on;
    end
    
    % Corner Cases
    if state_1 == 1
        plot([x+right_1,x+right_1+1],[y,righti],rightCol,"LineWidth",2);
        hold on;
        plot([x,x+right_1],[righti-1,righti],rightCol,"LineWidth",2);
        hold on
    end
    if state_1 == 3
        plot([x+left_1,x+left_1+1],[y,lefti],leftCol,"LineWidth",2);
        hold on;
        plot([x+left_1,x],[lefti,lefti-1],leftCol,"LineWidth",2);
        hold on
    end
end

%%
% Name Assigner for Plot Legend
function name = sensName(color)
   switch color
       case "c"            
           name = 'UV';
       case "r"
           name = 'IR';
       case "b"
           name = 'Laser';
       case "g"
           name = 'Lidar';
       otherwise
           name = 'ERROR';
   end
end