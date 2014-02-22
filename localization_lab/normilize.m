%normlizes the probablity map to ensure that all probablitys add to 1
function map = normilize(map)
    map = map./sum(sum(sum(map))); %normilzes
end