classdef ROS_MPC < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % xr
        vmax = 1.0 % m / s
        amax = 5.0 % m / s / s
        dt = 0.5 % s

        vel_pub
        traj_sub

        init_flag = false
    end
    
    methods
        function obj = ROS_MPC()
            % publisher for velocity command
            obj.vel_pub = rospublisher("/rto/mpc_output", "geometry_msgs/PoseArray" , "DataFormat", "struct");

            % subsriber for MPC input
            obj.traj_sub = rossubscriber("/rto/mpc_input", @obj.traj_cb, "DataFormat", "struct");
        end
        
        %==========================================================
        %==========================================================
        %==========================================================
        % pose callback
        function traj_cb(self, src, msg)

            disp('traj_cb')
            if self.init_flag == false
                self.init_flag = true;
                % self.ti = double(msg.header.stamp.sec )+ double(msg.header.stamp.nsec)/1e9;
            end

            %get array length
            num = length(msg.Poses);

            %%%%%%%%%%%%%%%%%%%%%%
            %  p_left   p_right  %
            %     o       o      %
            %      \     /       %
            %       \   /        %
            %        \ /         %
            %         o          %
            %     p_center       %
            %%%%%%%%%%%%%%%%%%%%%%

            vi = [msg.Poses(1).Position.X, msg.Poses(1).Position.Y];
            p_center = [msg.Poses(2).Position.X, msg.Poses(2).Position.Y];
            p_right = [msg.Poses(3).Position.X, msg.Poses(3).Position.Y];
            p_left = [msg.Poses(4).Position.X, msg.Poses(4).Position.Y];

            N = num - 4 - 1;
            xr = zeros(N+1, 2);
            
            k = 5;
            for i = 1:(N+1)
                xr(i,:) = [msg.Poses(k).Position.X, msg.Poses(k).Position.Y];
                k = k + 1;
            end

            [x1,x2,v1,v2,u1,u2, exitflag] = self.run_mpc(xr, vi, self.dt, self.vmax, self.amax, p_center, p_right, p_left);
            
            [x1, x2, v1, v2, xr]

            if (exitflag >= 0)
                poseArrayMsg = rosmessage(self.vel_pub);            
                poseMsg = rosmessage("geometry_msgs/Pose" ,"DataFormat","struct");
    
                for i = 1:(N+1)
                    poseMsg.Position.X = v1(i);
                    poseMsg.Position.Y = v2(i);
                    poseMsg.Position.Z = 0;
    
                    % poseMsg
    
                    poseArrayMsg.Poses(i) = poseMsg; 
                end
    
                % poseArrayMsg
    
                send(self.vel_pub,poseArrayMsg);
            end
            
        end
        
        %==========================================================
        %==========================================================
        %==========================================================
        function [x1,x2,v1,v2,u1,u2, exitflag] = run_mpc(self, xr, vi, dt, vmax, amax, p_center, p_right, p_left)
        
        disp("[run_mpc()]");

        p_center
        p_left
        p_right

        vmax
        amax
        vi
        dt
        
        xr

        % dimensions
        n = 4;
        m = 2;
        % N = 6;
        N = size(xr,1)-1;
        
        k1 = n*(N+1);
        k2 = m*N;
        
        % control and velocity limits
        % amax = 3;
        % vmax = 5;
        
        % reference
        % x1r = linspace(0, 2, N+1)';
        % x2r = linspace(0, 6, N+1)';
        % xr = [x1r, x2r];
        % T = max(vecnorm(xr,2,2))/(vmax*0.5);
        % dt = T/(N+1);
        
        Xr = [xr, [vi(1);zeros(N,1)], [vi(2);zeros(N,1)]];
        Xr_r = Xr';
        Xr_r = Xr_r(:)
        
        % costs
        Qi = diag([1,1,0,0])*100;
        Q = diag([1,1,0,0])*100;
        Qf = diag([1,1,0,0])*100;
        R = diag([1,1])*1;
        
        tmpQ = {};
        tmpR = {};
        for i = 1:(N+1)
            if i == 1
                tmpQ{i} = Qi;
                tmpR{i} = R;
            elseif i < (N+1)
                tmpQ{i} = Q;
                tmpR{i} = R;
            else
                tmpQ{i} = Qf;
            end
        end
        
        Qp = blkdiag(tmpQ{:});
        Rp = blkdiag(tmpR{:});
        
        H = [[Qp, zeros(k1,k2)];
             [zeros(k2, k1), Rp]];
        f = (-1*[Xr_r'*Qp, zeros(1, k2)])';
        
        % system dynamics and equality constraints
        A1 = [0, 0, 1, 0;
              0, 0, 0, 1;
              0, 0, 0, 0;
              0, 0, 0, 0];
        B1 = [0, 0;
              0, 0;
              1, 0;
              0, 1];
        
        % A = dt*A1 + eye(n);
        % B = dt*B1;
        A = dt*A1 + 1/2*dt^2*A1^2 + eye(n);
        B = dt*B1 + 1/2*dt^2*B1;
        
        % tmpA = {};
        Ap = zeros(n*N, k1);
        tmpB = {};
        
        ind1 = 1:n;
        ind2 = 1:2*n;
        for i = 1:N
            % ind1
            % ind2
            % tmpA{i} = [A, -eye(n)];
            Ap(ind1, ind2) = [A, -eye(n)];
            tmpB{i} = B;
            ind1 = ind1 + n;
            ind2 = ind2 + n;
        end
        
        % Ap = blkdiag(tmpA{:});
        Bp = blkdiag(tmpB{:});
        Aeq = [[eye(n), zeros(n, k1+k2-n)]; Ap, Bp];
        beq = [Xr(1,:)'; zeros(k1-n, 1)];
        % Aeq = [Ap, Bp];
        % beq = [zeros(k1-n, 1)];
        
        
        % get inequality constraints
        % p_center = [0, -1];
        % p_right = [1, 1];
        % p_left = [-1, 1];
        x_ch = xr(1,:);
        [c1, d1] = self.gen_line1(p_center, p_right, x_ch);
        [c2, d2] = self.gen_line1(p_center, p_left, x_ch);
        
        tmpC1 = {};
        tmpC2 = {};
        
        for i = 1:(N+1)
            tmpC1{i} = [c1,0,0];
            tmpC2{i} = [c2,0,0];
        end
        
        C1p = blkdiag(tmpC1{:});
        C2p = blkdiag(tmpC2{:});
        AA = -[[C1p; C2p], zeros(2*(N+1), m*N)];
        bb = [ones(N+1, 1)*d1; ones(N+1, 1)*d2];
        
        % set bounds
        lb = -inf(k1+k2,1);
        ub = +inf(k1+k2,1);
        %     % get index for x
        % tmp = (1:n:(n*(N)+1))';
        % tmp = ([tmp, tmp]+[0,1])';
        % ind_x = tmp(:);
        %     % get index for v
        % tmp = (1:n:(n*(N)+1))';
        % tmp = ([tmp, tmp]+[2,3])';
        % ind_v = tmp(:);
        %     % get index for u
        % tmp = (n*(N+1)+1):1:(k1+k2);
        % ind_u = tmp';
        
            % get index for x1
        ind_x1 = (1:n:(n*(N)+1))';
            % get index for x2
        ind_x2 = (1:n:(n*(N)+1))'+1;
            % get index for v1
        ind_v1 = (1:n:(n*(N)+1))'+2;
            % get index for v2
        ind_v2 = (1:n:(n*(N)+1))'+3;
            % get index for u1
        ind_u1 = ( (n*(N+1)+1):m:(k1+k2) )';
            % get index for u2
        ind_u2 = ( ((n*(N+1)+1):m:(k1+k2)) )' + 1;
        
        ind_v = [ind_v1; ind_v2];
        ind_u = [ind_u1; ind_u2];
        
        lb(ind_v) = -vmax;
        ub(ind_v) = +vmax;
        lb(ind_u) = -amax;
        ub(ind_u) = +amax;
        
        x0 = [Xr_r; ones(k2,1)*0.5];
        options = optimoptions('quadprog','Algorithm','active-set');
        [sol,fval,exitflag,output,lambda] = quadprog(H,f,AA,bb,Aeq,beq,lb,ub,x0,options);
        
        exitflag

        % [sol(ind_x), sol(ind_v)]
        % [sol(ind_x1), sol(ind_x2), sol(ind_v1), sol(ind_v2)]
        % 
        % sol(ind_u)
        % [sol(ind_u1), sol(ind_u2)]
        
        x1 = sol(ind_x1);
        x2 = sol(ind_x2);
        v1 = sol(ind_v1);
        v2 = sol(ind_v2);
        u1 = sol(ind_u1);
        u2 = sol(ind_u2);
        
        % x1 = linspace(-5,5,31);
        % x2 = linspace(-5,5,31);
        % 
        % figure
        % hold on
        % 
        % plot_surf(c1,d1, x1, x2)
        % plot_surf(c2,d2, x1, x2)
        % 
        % plot([p_center(1), p_right(1)], [p_center(2), p_right(2)])
        % plot([p_center(1), p_left(1)], [p_center(2), p_left(2)])
        
        end

        %==========================================================
        %==========================================================
        %==========================================================
        function [c, d, Xu] = gen_line1(self, p1, p2, x_ch)
        
        % check for vertical line
        if abs(p1(1)-p2(1))<=1e-3
            c = [1, 0];
            d = -p1(1);
        
            num = 10;
            % generate points on the line
            x = ones(num,1)*p1(1);
            tmp = [p1(2), p2(2)];
            y = linspace(min(tmp), max(tmp), num)';
        
            Xu = [x,y];
            
            % check for which side to push
            val = sum(c.*x_ch) + d;
            if val < 0
                c = -c;
                d = -d;
            end
        
            % % push points
            % Xs = Xu + c*f;
            % 
            % % get center point
            % xp = p1(1);
            % yp = (p1(2)+p2(2))/2;
            % xyp = [xp, yp];
            % Xc = [xyp + f_rbf*c;
            %       xyp - f_rbf*c];
        
        else
            % get slope
            m = (p1(2)-p2(2))/(p1(1)-p2(1));
            b = p1(2) - m*p1(1);
            
            % get normalized line
            c = [-m, 1];
            d = -b;
            cn = norm(c);
            c = c/cn;
            d = d/cn;
            
            % check for which side to push
            val = sum(c.*x_ch) + d;
            if val < 0
                c = -c;
                d = -d;
            end
            num = 10;
            % generate points on the line
            tmp = [p1(1), p2(1)];
            x = linspace(min(tmp), max(tmp), num)';
            y = m*x + b;
        
            Xu = [x,y];
            % 
            % % push points
            % Xs = Xu + c*f;
            % 
            % % get center point
            % xp = (p1(1)+p2(1))/2;
            % yp = m*xp + b;
            % xyp = [xp, yp];
            % Xc = [xyp + f_rbf*c;
            %       xyp - f_rbf*c];
            
        end
        
        end
        

        
    end
end

