function open = findOpenList(map)
global EMPTY;
global SOLID;
global START;
global END;
open = [];
%generates a list of the open spaces to pick the start and end location
for i = 2:size(map,1)-1
    for j = 2:size(map,2)-1
        if map(i,j) == EMPTY
            open = cat(1,open,[i,j]);
        end
    end
end

end