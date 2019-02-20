clc; clear all; close all;

adams_q = load('adams_q_ik_output.txt');
adams_end = load('adams_end_ik_input.txt');
c_q = load('hj_inverse_kinematics_result.txt');

% adams_q = load('q_input.txt');
% adams_end = load('adams_end.txt');
% c_q = load('hj_kinematics_result.txt');

title_end = {'end point x','end point y','end point z','end point roll','end point pitch','end point yaw'};

%% result compare plot
figure
set(gcf,'Color',[1,1,1])
for i = 1 : 12
    subplot(4,3,i)
    if i <= 6
        plot(adams_q(:,1), adams_q(:,i+1),'b','LineWidth',2.5);
        hold on
        plot(c_q(:,1), c_q(:,i+1),'r--','LineWidth',2.5);
        grid on
        title(sprintf('q %d',i))
        xlabel('Time [s]')
        ylabel('Angle [rad]')
        set(gca,'FontSize',15)
    else
        plot(adams_end(:,1), adams_end(:,i-5),'b','LineWidth',2.5);
        hold on
        plot(c_q(:,1), c_q(:,i+1),'r--','LineWidth',2.5);
        grid on
        title(title_end{i-6})
        xlabel('Time [s]')
        if (i >= 7 && i <= 9)
            ylabel('Position [meter]')
        else
            ylabel('Angle [rad]')
        end
        xlim([0 1])
        set(gca,'FontSize',15)
    end
    if i == 3
        legend('ADAMS','Analysis')
    end
end

%% error pot
figure
set(gcf,'Color',[1,1,1])
for i = 1 : 12
    subplot(4,3,i)
    if i <= 6
        plot(adams_q(:,1), adams_q(:,i+1) - c_q(:,i+1),'b','LineWidth',2.5);
        grid on
        title(sprintf('q %d',i))
        xlabel('Time [s]')
        ylabel('Angle [rad]')
        set(gca,'FontSize',15)
    else
        plot(adams_end(:,1), adams_end(:,i-5) - c_q(:,i+1),'b','LineWidth',2.5);
        grid on
        title(title_end{i-6})
        xlabel('Time [s]')
        if (i >= 7 && i <= 9)
            ylabel('Position [meter]')
        else
            ylabel('Angle [rad]')
        end
        xlim([0 1])
        set(gca,'FontSize',15)
    end
end