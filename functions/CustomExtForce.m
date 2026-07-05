%Function to calculate the custom external force and its Jacobian
%If dFext_dq is unavailable keep it [] and turn off Jacobian during simulation
%Last modified by Anup Teejo Mathew 05/02/2025

function [Fext,dFext_dq] = CustomExtForce(Linkage,q,g,J,t,qd,Jdot)
    %%%%NOTE%%%%
    %Linkage: Linkage element,
    %q and qd: joint coordinates and their time derivatives,
    %g, J, and Jd: transformation matrix, Jacobian, and time derivative of Jacobian at every significant point of the linkage
    %t: time
    
    %Fext should be 6*nsig column vector of point external wrenches. If distributed wrench, multiply with
    %corresponding quadrature weight to make it a point wrench!
    
    % Significant points: 1 for every joint (at X=1), 1 at the center of the rigid link, for soft links at every integration points
    
    % J   = S.Jacobian(q);         %geometric jacobian of the linkage calculated at every significant points
    % g   = S.FwdKinematics(q);    %transformation matrix of the linkage calculated at every significant points
    % eta = S.ScrewVelocity(q,qd); %Screwvelocity of the linkage calculated at every significant points
    % J   = S.Jacobiandot(q,qd);   %time derivative of geometric jacobian of the linkage calculated at every significant points
    
    %%%END OF NOTE%%%
    
    global load_tip;
    
    nsig = Linkage.nsig; %everything except rigid joints
    Fext = zeros(6*nsig,1);
    dFext_dq = zeros(6*nsig,Linkage.ndof);
    
    %% Add External Disturbances
    Fext(end-5:end) = load_tip;
end