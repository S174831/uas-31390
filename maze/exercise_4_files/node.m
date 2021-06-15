classdef node
    properties
        % Node which is this node's parrent
        parent
        % The position of the node
        position
        % The values which the node uses to calculate
        g
        h
        f
    end
    
    methods
        function h = calc_dist(obj, end_pos)
            h = sqrt((obj.position(1)-end_pos(1))^2 + ...
                     (obj.position(2)-end_pos(2))^2);
        end

        function h = calc_dist_3d(obj, end_pos)
            h = sqrt((obj.position(1)-end_pos(1))^2 + ...
                     (obj.position(2)-end_pos(2))^2 + ...
                     (obj.position(3)-end_pos(3))^2);
        end
        
        function g = calc_dist_tostart_3d(obj, start)
            g = [obj.position];
            while  ~(obj.position(1) == start(1) && ...
                     obj.position(2) == start(2) && ...
                     obj.position(3) == start(3))
               % Update the route by going backwards through the parents
                     obj = obj.parent;
                     g = cat(1,g,obj.position);
            end
            g=length(g);
        end
    end
end