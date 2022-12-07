function  animate_pendulum(f,stim_time)
%Animate the pendulum and observe it's behavior given f, the solved
%differential equation system. 
    r = 0.3;
    a2 = 0.09;
    a1 = 0.0436;
    x_pos = @(t) r*sin(deval(f,t,1));
    y_pos = @(t) r*cos(deval(f,t,1));
    
    if ~isempty(f.xe) 
        stim_time = f.xe;
    end
    fr = 100;
    figure;

    fanimator(@(t) plot(x_pos(t),y_pos(t),'ko','MarkerFaceColor','k'),'AnimationRange',[0 stim_time],'FrameRate',fr );
    hold on;
    fanimator(@(t) plot([0 x_pos(t)],[0 y_pos(t)],'k-'),'AnimationRange',[0 stim_time],'FrameRate',fr);
    fanimator(@(t) plot([-0.5 0.1],[0 0],'k-'),'AnimationRange',[0 stim_time],'FrameRate',fr );
    fanimator(@(t) plot([-a1 x_pos(t)*a2/r],[0 y_pos(t)*a2/r],'k-'),'AnimationRange',[0 stim_time],'FrameRate',fr);
    fanimator(@(t) plot([a1 x_pos(t)*a2/r],[0 y_pos(t)*a2/r],'k-'),'AnimationRange',[0 stim_time],'FrameRate',fr);
    fanimator(@(t) text(-0.3,0.7,"theta: "+num2str(deval(f,t,1)*180/pi)+"°"),'AnimationRange',[0 stim_time],'FrameRate',fr);
    fanimator(@(t) text(-0.3,0.6,"theta dot: "+num2str(deval(f,t,2))+"rad/sec"),'AnimationRange',[0 stim_time],'FrameRate',fr );
    fanimator(@(t) text(-0.3,0.5,"Timer: "+num2str(t,2)+" s"),'AnimationRange',[0 stim_time],'FrameRate',fr );
    
    xlim([-0.5 0.5])
    ylim([-0.2 0.8])
    title('Pendulum Muscle Model Animation')
    playAnimation 
end

