function joint_plots(fignum,xlabel_str,ylabel_str,t,q,qlimits)

figure(fignum)

[m,n] = size(q);
if m ~= 7
    q = q';
end

for k = 1:7
    
    if k == 7
        subplot(3,3,8)
        plot(t,q(k,:),'b','LineWidth',2)
        xlabel(xlabel_str,'Interpreter','Latex')
        ylabel(ylabel_str,'Interpreter','Latex')
        title(['q',num2str(k)])
        if ~isempty(qlimits)
           hold on
           plot(t,qlimits(k,1)*ones(1,length(t)),'k--')
           plot(t,qlimits(k,2)*ones(1,length(t)),'k--')
        end
        
        
    else
        subplot(3,3,k)
        plot(t,q(k,:),'b','LineWidth',2)
        xlabel(xlabel_str,'Interpreter','Latex')
        title(['q',num2str(k)])
        if (k ==1)||(k ==4)
            ylabel(ylabel_str,'Interpreter','Latex')
        end
        if ~isempty(qlimits)
           hold on
           plot(t,qlimits(k,1)*ones(1,length(t)),'k--')
           plot(t,qlimits(k,2)*ones(1,length(t)),'k--')
        end
    end
    
end



end