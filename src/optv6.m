%dh parameters
alpha = [0;90;0;0;0;0];
theta = zeros(6,1);
B = [0;0;0;1];

%create grid
w_vals = 3;
g_lim = 12;
cell_width = 1;
grid_pt = zeros(3,1,(length(w_vals) * ((g_lim - cell_width)/cell_width)^3),length(w_vals));
count = 1;
for gx = (cell_width/2): cell_width: (g_lim - cell_width)
    for gy = (cell_width/2): cell_width: (g_lim - cell_width)
        for gz = (cell_width/2): cell_width: (g_lim - cell_width)
            for w_ind = 1:length(w_vals)
                grid_pt(:,1,count,w_ind) = [gx + w_vals(w_ind); gy; gz];
                count = count + 1;
            end
        end
    end
end

%create lengths
l1_vals = 6:.125:15;
l2_vals = 6:.125:15;
l3_vals = 6:.125:15;
l4 = 3;
L = zeros(3,1,length(l1_vals)*length(l2_vals)*length(l3_vals));
count = 1;
cost = zeros(size(L,3),1);
for l3 = l3_vals
    for l2 = l2_vals
        for l1 = l1_vals
            L(:,:,count) = [l1;l2;l3];
            cost(count) = l2 + 2 * l3;
            count = count + 1;
        end
    end
end

[~,I] = sort(cost);
L(:,:,1:end) = L(:,:,I);



min_cost = intmax;
min_lengths = zeros(4,1);
flag = 0;
dist_thresh = .1;
%tic
%optimize
for l_ind = 1:size(L,3)
    for w_ind = 1:length(w_vals)
%         if(L(2,1,l_ind) + 2 * L(3,1,l_ind) > min_cost)
%             continue;
%         end
        J2 = [0;0;L(1,1,l_ind);1];
        a = [0;0;0;L(2,1,l_ind);L(3,1,l_ind);l4];
        d = [L(1,1,l_ind);0;0;0;0;0];
        for grid_ind = 1:length(grid_pt)
            %find desired grid pt
            p = grid_pt(:,1,grid_ind,w_ind) + [0;0;l4];
            %compute ik
            dist1 = norm(p - [0;0;L(1,1,l_ind)]) - (L(2,1,l_ind) + L(3,1,l_ind));
            if(dist1 > 0)
                flag = 1;
                break;
            end
            theta(1) = atan2d(p(2),p(1));
            c3 = (p(1)^2 + p(2)^2 + (p(3) - L(1,1,l_ind))^2 - L(1,1,l_ind)^2 - L(3,1,l_ind)^2) / (2*L(2,1,l_ind)*L(3,1,l_ind));
            %c3 = (p(1)^2 + p(2)^2 + (p(3) - L(1,1,l_ind))^2 - L(2,1,l_ind)^2 - L(3,1,l_ind)^2)/(2*L(2,1,l_ind)*L(3,1,l_ind));
            s3 = sqrt(1-c3^2);
            %check if Im(s3) ~= 0 
            if(imag(s3) ~= 0)
                flag = 1;
                break;
            end
            theta(4) = atan2d(s3,c3);
            theta(3) = atan2d(p(3)-L(1,1,l_ind),sqrt(p(1)^2+p(2)^2))-atan2d(L(3,1,l_ind)*s3,L(2,1,l_ind)+L(3,1,l_ind)*c3);
            if(theta(3)<0)
                s3 = -s3;
                theta(4) = atan2d(s3,c3);
                theta(3) = atan2d(p(3)-L(1,1,l_ind),sqrt(p(1)^2+p(2)^2))-atan2d(L(3,1,l_ind)*s3,L(2,1,l_ind)+L(3,1,l_ind)*c3);
            end

            theta(5) = -(90 + theta(3) + theta(4));
            T = dhTransform([a alpha d theta]);
            J3 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*B;
            J4 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*B;
            J5 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*B;
            %check distance from desired pt
            if(norm(p-J4(1:3))>dist_thresh)
                flag = 1;
                break
            end
            %check for collisions
            x_lim = [w_vals(w_ind);w_vals(w_ind) + g_lim];
            y_lim = [-g_lim/2;g_lim/2];
            z_lim = [0;p(3)-l4];
            %constant x faces
            n = [1;0;0];
            p1 = [x_lim(1);y_lim(1);z_lim(1)];
            p2 = [x_lim(2);y_lim(1);z_lim(1)];
            %l2
            if(line_rect_intersection(J2(1:3),J3(1:3),n,p1,x_lim,y_lim,z_lim)||line_rect_intersection(J2(1:3),J3(1:3),n,p2,x_lim,y_lim,z_lim))
                flag = 1;
                break;
            %l3
            elseif(line_rect_intersection(J3(1:3),J4(1:3),n,p1,x_lim,y_lim,z_lim)||line_rect_intersection(J3(1:3),J4(1:3),n,p2,x_lim,y_lim,z_lim))
                flag = 1;
                break;
            end
            %constant y faces
            n = [0;1;0];
            p1 = [x_lim(1);y_lim(1);z_lim(1)];
            p2 = [x_lim(1);y_lim(2);z_lim(1)];
            %l2
            if(line_rect_intersection(J2(1:3),J3(1:3),n,p1,x_lim,y_lim,z_lim)||line_rect_intersection(J2(1:3),J3(1:3),n,p2,x_lim,y_lim,z_lim))
                flag = 1;
                break;
            %l3
            elseif(line_rect_intersection(J3(1:3),J4(1:3),n,p1,x_lim,y_lim,z_lim)||line_rect_intersection(J3(1:3),J4(1:3),n,p2,x_lim,y_lim,z_lim))
                flag = 1;
                break;
            end
            %constant z faces
            n = [0;0;1];
            p1 = [x_lim(1);y_lim(1);z_lim(1)];
            p2 = [x_lim(1);y_lim(1);z_lim(2)];
            %l2
            if(line_rect_intersection(J2(1:3),J3(1:3),n,p1,x_lim,y_lim,z_lim)||line_rect_intersection(J2(1:3),J3(1:3),n,p2,x_lim,y_lim,z_lim))
                flag = 1;
                break;
            %l3
            elseif(line_rect_intersection(J3(1:3),J4(1:3),n,p1,x_lim,y_lim,z_lim)||line_rect_intersection(J3(1:3),J4(1:3),n,p2,x_lim,y_lim,z_lim))
                flag = 1;
                break;
            end
        end
        if(flag)
            flag = 0;
            continue
        end
        temp_cost = L(2,1,l_ind) + 2 * L(3,1,l_ind);
        if(temp_cost < min_cost)
            min_cost = temp_cost;
            min_lengths = [L(1,1,l_ind);L(2,1,l_ind);L(3,1,l_ind);w_vals(w_ind)];
%             flag = 0;
            error('found minimum cost');
            %toc
        end
    end
end