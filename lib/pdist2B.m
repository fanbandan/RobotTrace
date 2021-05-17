function distances = pdist2B(Point,PMatrix)
    if exist('pdist2','file') ~= 0
        distances = pdist2(Point,PMatrix);
    else
        distances = NaN(1,size(PMatrix,1));
        for i = 1:size(PMatrix,1)
            distances(i) = norm(Point - PMatrix(i,:));
        end
    end
end